from __future__ import annotations

import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
import rowan

from ..sim_data_types import Action, State


class Backend:
    """Vectorized Newton-Euler rigid-body dynamics for a Crazyflie swarm.

    This backend preserves the public interface of backend/np.py while storing
    all drone states in batched NumPy arrays. The original backend remains
    available as ``np``; this module should be installed as ``np_vectorized``.
    """

    def __init__(self, node: Node, names: list[str], states: list[State]):
        self.node = node
        self.names = names
        self.clock_publisher = node.create_publisher(Clock, 'clock', 10)

        self.t = 0.0
        # Preserve the original backend timestep for numerical stability.
        self.dt = 0.001

        # Keep physics integration at 2000 Hz, while publishing /clock at a
        # lower rate to reduce ROS serialization and DDS overhead.
        self.clock_rate_hz = 200.0
        self.clock_publish_steps = 5
        self.step_count = 0

        self.count = len(states)
        if self.count == 0:
            raise ValueError('np_vectorized requires at least one drone')

        # Keep the original State objects so the returned interface remains
        # identical to backend/np.py.
        self.states = list(states)

        self.pos = np.stack(
            [np.asarray(state.pos, dtype=float) for state in states],
            axis=0,
        ).copy()
        self.vel = np.stack(
            [np.asarray(state.vel, dtype=float) for state in states],
            axis=0,
        ).copy()
        self.quat = np.stack(
            [np.asarray(state.quat, dtype=float) for state in states],
            axis=0,
        ).copy()
        self.omega = np.stack(
            [np.asarray(state.omega, dtype=float) for state in states],
            axis=0,
        ).copy()

        # Crazyflie 2.0 parameters, matching backend/np.py.
        self.mass = 0.034
        self.J = np.array(
            [16.571710e-6, 16.655602e-6, 29.261652e-6],
            dtype=float,
        )
        self.inv_J = 1.0 / self.J

        arm_length = 0.046
        arm = 0.707106781 * arm_length
        thrust_to_torque = 0.006

        self.B0 = np.array(
            [
                [1.0, 1.0, 1.0, 1.0],
                [-arm, -arm, arm, arm],
                [-arm, arm, arm, -arm],
                [-thrust_to_torque, thrust_to_torque,
                 -thrust_to_torque, thrust_to_torque],
            ],
            dtype=float,
        )

        self.gravity = np.array([0.0, 0.0, -9.81], dtype=float)

        effective_clock_rate = 1.0 / (
            self.clock_publish_steps * self.dt
        )

        self.node.get_logger().info(
            f'Vectorized NumPy physics backend started for {self.count} drones '
            f'at dt={self.dt:.4f} s; '
            f'/clock rate={effective_clock_rate:.1f} Hz'
        )

    def time(self) -> float:
        return self.t

    def step(
        self,
        states_desired: list[State],
        actions: list[Action],
    ) -> list[State]:
        del states_desired  # Dynamics depend on motor actions, as in np.py.

        if len(actions) != self.count:
            raise ValueError(
                f'Expected {self.count} actions, received {len(actions)}'
            )

        if any(action is None for action in actions):
            raise ValueError(
                'np_vectorized requires a controller that produces motor actions'
            )

        self.t += self.dt
        self.step_count += 1

        # Shape: (number_of_drones, 4)
        rpm = np.stack(
            [np.asarray(action.rpm, dtype=float) for action in actions],
            axis=0,
        )

        if rpm.shape != (self.count, 4):
            raise ValueError(
                f'Expected RPM array shape {(self.count, 4)}, got {rpm.shape}'
            )

        # Convert motor RPM to thrust force. This is the same polynomial used
        # by the original scalar NumPy backend.
        force_in_grams = np.polyval(
            [2.55077341e-08, -4.92422570e-05, -1.51910248e-01],
            rpm,
        )
        force = np.maximum(force_in_grams * 9.81 / 1000.0, 0.0)

        # Batched mixer:
        # force: (N, 4), B0.T: (4, 4), eta: (N, 4)
        eta = force @ self.B0.T

        thrust_body = np.zeros((self.count, 3), dtype=float)
        thrust_body[:, 2] = eta[:, 0]
        torque_body = eta[:, 1:4]

        # Explicit integration, matching backend/np.py.
        pos_next = self.pos + self.vel * self.dt

        thrust_world = rowan.rotate(self.quat, thrust_body)
        acceleration = self.gravity + thrust_world / self.mass
        vel_next = self.vel + acceleration * self.dt

        omega_global = rowan.rotate(self.quat, self.omega)
        quat_next = rowan.normalize(
            rowan.calculus.integrate(
                self.quat,
                omega_global,
                self.dt,
            )
        )

        gyroscopic = np.cross(self.J * self.omega, self.omega)
        omega_next = self.omega + (
            self.inv_J * (gyroscopic + torque_body)
        ) * self.dt

        # Ground constraint, independently applied to each drone.
        grounded = pos_next[:, 2] < 0.0
        if np.any(grounded):
            pos_next[grounded, 2] = 0.0
            vel_next[grounded] = 0.0
            omega_next[grounded] = 0.0

        self.pos = pos_next
        self.vel = vel_next
        self.quat = quat_next
        self.omega = omega_next

        # Preserve the original ordered list[State] API. Copies prevent a State
        # object from retaining a mutable view into the batched arrays.
        for index, state in enumerate(self.states):
            state.pos = self.pos[index].copy()
            state.vel = self.vel[index].copy()
            state.quat = self.quat[index].copy()
            state.omega = self.omega[index].copy()

        # Publish /clock at a decimated rate using an integer step counter.
        # This avoids floating-point accumulation error.
        if (
            self.step_count == 1
            or self.step_count % self.clock_publish_steps == 0
        ):
            clock_message = Clock()
            clock_message.clock = Time(seconds=self.t).to_msg()
            self.clock_publisher.publish(clock_message)

        return self.states

    def shutdown(self):
        pass
