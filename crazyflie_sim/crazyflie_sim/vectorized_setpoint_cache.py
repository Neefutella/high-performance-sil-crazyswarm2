#!/usr/bin/env python3
"""Cached batched setpoint bridge for the Phase 5A vector controller."""

from __future__ import annotations

import os

import numpy as np


class VectorizedSetpointCache:
    """Sample firmware planners slowly and extrapolate desired arrays at 1 kHz.

    The firmware high-level planner remains the source of trajectory truth.
    Nine SWIG planner evaluations are performed at a configurable low rate
    (100 Hz by default), then position is linearly extrapolated using the
    sampled desired velocity between planner updates.
    """

    def __init__(self, backend) -> None:
        self.backend = backend
        self.node = backend.node
        self.count = backend.count

        requested_rate = self._env_float("CF_SIM_PLANNER_RATE_HZ", 100.0)
        if requested_rate <= 0.0:
            raise ValueError("CF_SIM_PLANNER_RATE_HZ must be positive")

        physics_rate = 1.0 / float(backend.dt)
        self.divider = max(1, int(round(physics_rate / requested_rate)))
        self.actual_rate = physics_rate / self.divider

        self.counter = 0
        self.sample_time = 0.0
        self.sample_states = None
        self.pos = None
        self.vel = None
        self.quat = None

        self.node.get_logger().info(
            "Vectorized planner cache enabled: "
            f"requested={requested_rate:.1f}Hz "
            f"actual={self.actual_rate:.1f}Hz "
            f"divider={self.divider}"
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

    def _sample(self, cf_list, sim_time: float) -> None:
        states = [cf.getSetpoint() for cf in cf_list]
        if len(states) != self.count:
            raise ValueError(
                f"Expected {self.count} desired states, got {len(states)}"
            )

        self.sample_states = states
        self.pos = np.stack(
            [np.asarray(state.pos, dtype=float) for state in states],
            axis=0,
        )
        self.vel = np.stack(
            [np.asarray(state.vel, dtype=float) for state in states],
            axis=0,
        )
        self.quat = np.stack(
            [np.asarray(state.quat, dtype=float) for state in states],
            axis=0,
        )
        self.sample_time = float(sim_time)

    def step(self, cf_list, sim_time: float):
        should_sample = (
            self.sample_states is None
            or self.counter % self.divider == 0
        )

        if should_sample:
            self._sample(cf_list, sim_time)

        self.counter += 1

        age = max(0.0, float(sim_time) - self.sample_time)
        max_age = self.divider * float(self.backend.dt)
        age = min(age, max_age)

        # A first-order reconstruction is smooth, cheap, and exact for
        # constant-velocity sections. The next 100 Hz firmware sample corrects
        # any acceleration-related approximation error.
        pos_des = self.pos + self.vel * age

        return (
            self.sample_states,
            pos_des,
            self.vel,
            self.quat,
        )
