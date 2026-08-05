#!/usr/bin/env python3
"""Batched cascaded position-attitude controller for Crazyflie simulation."""

from __future__ import annotations

import math
import os

import numpy as np
import rowan

from .sim_data_types import Action


class VectorizedCascadedController:
    """Rate-explicit geometric/cascaded controller for a drone batch."""

    def __init__(self, backend) -> None:
        required = ("pos", "vel", "quat", "omega", "mass", "B0")
        missing = [name for name in required if not hasattr(backend, name)]
        if missing:
            raise ValueError(
                "Vectorized controller requires backend attributes: "
                + ", ".join(missing)
            )

        self.backend = backend
        self.node = backend.node
        self.count = backend.count
        self.mass = float(backend.mass)
        self.B0 = np.asarray(backend.B0, dtype=float)
        self.B0_inv = np.linalg.inv(self.B0)

        self.kp_pos = np.array([
            self._env_float("CF_SIM_KP_XY", 4.0),
            self._env_float("CF_SIM_KP_XY", 4.0),
            self._env_float("CF_SIM_KP_Z", 8.0),
        ])
        self.kd_pos = np.array([
            self._env_float("CF_SIM_KD_XY", 3.0),
            self._env_float("CF_SIM_KD_XY", 3.0),
            self._env_float("CF_SIM_KD_Z", 4.0),
        ])

        self.kR = np.array([
            self._env_float("CF_SIM_KR_XY", 0.0045),
            self._env_float("CF_SIM_KR_XY", 0.0045),
            self._env_float("CF_SIM_KR_Z", 0.0015),
        ])
        self.kW = np.array([
            self._env_float("CF_SIM_KW_XY", 0.00045),
            self._env_float("CF_SIM_KW_XY", 0.00045),
            self._env_float("CF_SIM_KW_Z", 0.00025),
        ])

        self.gravity = 9.81
        self.max_xy_accel = self._env_float("CF_SIM_MAX_XY_ACCEL", 2.0)
        self.max_z_feedback = self._env_float("CF_SIM_MAX_Z_FB", 4.0)
        self.max_tilt = math.radians(
            self._env_float("CF_SIM_MAX_TILT_DEG", 20.0)
        )
        self.max_torque = np.array([
            self._env_float("CF_SIM_MAX_TORQUE_XY", 0.0035),
            self._env_float("CF_SIM_MAX_TORQUE_XY", 0.0035),
            self._env_float("CF_SIM_MAX_TORQUE_Z", 0.0010),
        ])

        self.max_rpm = 0.326535711 * 65535.0 + 3374.95115
        self.min_nonzero_rpm = 0.326535711 * 10000.0 + 3374.95115
        self.max_motor_force = float(self._rpm_to_force(self.max_rpm))
        self.min_motor_force = float(
            self._rpm_to_force(self.min_nonzero_rpm)
        )

        self.step_count = 0
        self._saturation_accumulator = 0.0

        self.node.get_logger().info(
            "Vectorized cascaded controller enabled: "
            f"kp={self.kp_pos.tolist()} kd={self.kd_pos.tolist()} "
            f"kR={self.kR.tolist()} kW={self.kW.tolist()} "
            f"max_tilt={math.degrees(self.max_tilt):.1f}deg "
            f"max_xy_accel={self.max_xy_accel:.2f}m/s^2"
        )

    @staticmethod
    def _env_float(name: str, default: float) -> float:
        value = os.environ.get(name)
        if value is None:
            return default
        try:
            return float(value)
        except ValueError as exc:
            raise ValueError(f"{name} must be a float, got {value!r}") from exc

    @staticmethod
    def _rpm_to_force(rpm):
        force_grams = np.polyval(
            [2.55077341e-08, -4.92422570e-05, -1.51910248e-01],
            rpm,
        )
        return np.maximum(force_grams * 9.81 / 1000.0, 0.0)

    def _force_to_rpm(self, force: np.ndarray) -> np.ndarray:
        force = np.asarray(force, dtype=float)
        grams = force * 1000.0 / 9.81

        a = 2.55077341e-08
        b = -4.92422570e-05
        c = -1.51910248e-01

        discriminant = np.maximum(
            b * b - 4.0 * a * (c - grams),
            0.0,
        )
        rpm = (-b + np.sqrt(discriminant)) / (2.0 * a)
        rpm = np.where(force < self.min_motor_force, 0.0, rpm)
        return np.clip(rpm, 0.0, self.max_rpm)

    @staticmethod
    def _normalize(
        vectors: np.ndarray,
        fallback: np.ndarray,
    ) -> np.ndarray:
        norms = np.linalg.norm(vectors, axis=1, keepdims=True)
        safe = norms[:, 0] > 1.0e-9
        result = np.empty_like(vectors)
        result[safe] = vectors[safe] / norms[safe]
        result[~safe] = fallback
        return result

    def _allocate_motor_forces(
        self,
        collective: np.ndarray,
        torque: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        collective = np.clip(
            collective,
            0.0,
            4.0 * self.max_motor_force,
        )

        eta = np.column_stack((collective, torque))
        raw = eta @ self.B0_inv.T

        base = collective[:, None] / 4.0
        differential = raw - base
        base_full = np.broadcast_to(base, differential.shape)

        positive = differential > 1.0e-12
        positive_limit = np.full_like(differential, np.inf)
        positive_ratio = (
            self.max_motor_force - base_full
        ) / np.where(positive, differential, 1.0)
        positive_limit[positive] = positive_ratio[positive]

        negative = differential < -1.0e-12
        negative_limit = np.full_like(differential, np.inf)
        negative_ratio = (
            -base_full
        ) / np.where(negative, differential, -1.0)
        negative_limit[negative] = negative_ratio[negative]

        alpha = np.ones(self.count, dtype=float)
        alpha = np.minimum(alpha, np.min(positive_limit, axis=1))
        alpha = np.minimum(alpha, np.min(negative_limit, axis=1))
        alpha = np.clip(alpha, 0.0, 1.0)

        forces = base + alpha[:, None] * differential
        forces = np.clip(forces, 0.0, self.max_motor_force)
        return forces, alpha

    def step(self, states_desired, cf_list) -> list[Action]:
        pos_des = np.stack(
            [np.asarray(state.pos, dtype=float) for state in states_desired],
            axis=0,
        )
        vel_des = np.stack(
            [np.asarray(state.vel, dtype=float) for state in states_desired],
            axis=0,
        )
        quat_des = np.stack(
            [np.asarray(state.quat, dtype=float) for state in states_desired],
            axis=0,
        )
        return self.step_arrays(pos_des, vel_des, quat_des, cf_list)

    def step_arrays(
        self,
        pos_des: np.ndarray,
        vel_des: np.ndarray,
        quat_des: np.ndarray,
        cf_list,
    ) -> list[Action]:
        pos_des = np.asarray(pos_des, dtype=float)
        vel_des = np.asarray(vel_des, dtype=float)
        quat_des = np.asarray(quat_des, dtype=float)

        expected_vec = (self.count, 3)
        expected_quat = (self.count, 4)
        if pos_des.shape != expected_vec:
            raise ValueError(
                f"Expected desired position shape {expected_vec}, "
                f"got {pos_des.shape}"
            )
        if vel_des.shape != expected_vec:
            raise ValueError(
                f"Expected desired velocity shape {expected_vec}, "
                f"got {vel_des.shape}"
            )
        if quat_des.shape != expected_quat:
            raise ValueError(
                f"Expected desired quaternion shape {expected_quat}, "
                f"got {quat_des.shape}"
            )
        if len(cf_list) != self.count:
            raise ValueError(
                "Controller batch size does not match backend count"
            )

        pos = self.backend.pos
        vel = self.backend.vel
        quat = self.backend.quat
        omega = self.backend.omega

        position_error = pos_des - pos
        velocity_error = vel_des - vel

        accel_feedback = (
            self.kp_pos * position_error
            + self.kd_pos * velocity_error
        )

        xy = accel_feedback[:, :2]
        xy_norm = np.linalg.norm(xy, axis=1)
        xy_scale = np.ones(self.count)
        too_large = xy_norm > self.max_xy_accel
        xy_scale[too_large] = (
            self.max_xy_accel / xy_norm[too_large]
        )
        accel_feedback[:, :2] *= xy_scale[:, None]
        accel_feedback[:, 2] = np.clip(
            accel_feedback[:, 2],
            -self.max_z_feedback,
            self.max_z_feedback,
        )

        desired_force_direction = accel_feedback.copy()
        desired_force_direction[:, 2] += self.gravity

        vertical = np.maximum(desired_force_direction[:, 2], 0.5)
        max_horizontal = np.tan(self.max_tilt) * vertical
        horizontal = desired_force_direction[:, :2]
        horizontal_norm = np.linalg.norm(horizontal, axis=1)
        tilt_scale = np.ones(self.count)
        over_tilt = horizontal_norm > max_horizontal
        tilt_scale[over_tilt] = (
            max_horizontal[over_tilt] / horizontal_norm[over_tilt]
        )
        desired_force_direction[:, :2] *= tilt_scale[:, None]

        b3_des = self._normalize(
            desired_force_direction,
            np.array([0.0, 0.0, 1.0]),
        )

        R_reference = rowan.to_matrix(quat_des)
        yaw_des = np.arctan2(
            R_reference[:, 1, 0],
            R_reference[:, 0, 0],
        )
        yaw_des = np.nan_to_num(yaw_des)

        x_heading = np.column_stack((
            np.cos(yaw_des),
            np.sin(yaw_des),
            np.zeros(self.count),
        ))
        b2_des = self._normalize(
            np.cross(b3_des, x_heading),
            np.array([0.0, 1.0, 0.0]),
        )
        b1_des = self._normalize(
            np.cross(b2_des, b3_des),
            np.array([1.0, 0.0, 0.0]),
        )

        R_des = np.stack((b1_des, b2_des, b3_des), axis=2)
        R = rowan.to_matrix(quat)

        Rt_Rd = np.einsum("nji,njk->nik", R, R_des)
        RdT_R = np.einsum("nji,njk->nik", R_des, R)
        skew_error = 0.5 * (RdT_R - Rt_Rd)

        attitude_error = np.column_stack((
            skew_error[:, 2, 1],
            skew_error[:, 0, 2],
            skew_error[:, 1, 0],
        ))

        torque = -self.kR * attitude_error - self.kW * omega
        torque = np.clip(torque, -self.max_torque, self.max_torque)

        collective = self.mass * np.linalg.norm(
            desired_force_direction,
            axis=1,
        )

        motor_forces, alpha = self._allocate_motor_forces(
            collective,
            torque,
        )
        rpm = self._force_to_rpm(motor_forces)

        active = np.array(
            [cf.mode != cf.MODE_IDLE for cf in cf_list],
            dtype=bool,
        )
        rpm[~active] = 0.0

        self.step_count += 1
        self._saturation_accumulator += float(
            np.mean(alpha < 0.999)
        )

        if self.step_count % 1000 == 0:
            saturation_fraction = (
                self._saturation_accumulator / 1000.0
            )
            max_position_error = float(
                np.max(np.linalg.norm(position_error, axis=1))
            )
            self.node.get_logger().info(
                "VECTOR_CTRL "
                f"max_pos_error={max_position_error:.3f}m "
                f"allocation_saturation="
                f"{100.0 * saturation_fraction:.1f}% "
                f"max_rpm={float(np.max(rpm)):.0f}"
            )
            self._saturation_accumulator = 0.0

        return [Action(row.tolist()) for row in rpm]
