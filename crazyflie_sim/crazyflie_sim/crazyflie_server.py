#!/usr/bin/env python3

"""
A crazyflie server for simulation.

    2022 - Wolfgang Hönig (TU Berlin)
    2025 - Updated by Kimberly N. McGuire (Independent)
"""

from concurrent.futures import ThreadPoolExecutor
from functools import partial
import json
import math
import importlib
import os
import time

from crazyflie_interfaces.msg import FullState, Hover
from crazyflie_interfaces.srv import GoTo, Land, Takeoff
from crazyflie_interfaces.srv import NotifySetpointsStop, StartTrajectory, UploadTrajectory
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import rowan
from std_msgs.msg import String
from std_srvs.srv import Empty


# import BackendRviz from .backend_rviz
# from .backend import *
# from .backend.none import BackendNone
from .crazyflie_sil import CrazyflieSIL, TrajectoryPolynomialPiece
from .vectorized_cascaded_controller import VectorizedCascadedController
from .vectorized_setpoint_cache import VectorizedSetpointCache
from .sim_data_types import State


class CrazyflieServer(Node):

    def __init__(self):
        super().__init__(
            'crazyflie_server',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # Turn ROS parameters into a dictionary
        self._ros_parameters = self._param_to_dict(self._parameters)
        self.cfs = {}

        world_tf_name = 'world'
        robot_yaml_version = 0

        try:
            robot_yaml_version = self._ros_parameters['fileversion']
        except KeyError:
            self.get_logger().info('No fileversion found in crazyflies.yaml, assuming version 0')

        robot_data = self._ros_parameters['robots']

        # Parse robots
        names = []
        initial_states = []
        reference_frames = []
        for cfname in robot_data:
            if robot_data[cfname]['enabled']:
                type_cf = robot_data[cfname]['type']
                # do not include virtual objects
                connection = self._ros_parameters['robot_types'][type_cf].get(
                    'connection', 'crazyflie')
                if connection == 'crazyflie':
                    names.append(cfname)
                    pos = robot_data[cfname]['initial_position']
                    initial_states.append(State(pos))
                    # Get the current reference frame for the robot
                    reference_frame = world_tf_name
                    if robot_yaml_version >= 3:
                        try:
                            reference_frame = self._ros_parameters['all']['reference_frame']
                        except KeyError:
                            pass
                        try:
                            reference_frame = self._ros_parameters['robot_types'][
                                robot_data[cfname]['type']]['reference_frame']
                        except KeyError:
                            pass
                        try:
                            reference_frame = self._ros_parameters['robots'][
                                cfname]['reference_frame']
                        except KeyError:
                            pass
                    reference_frames.append(reference_frame)

        # initialize backend by dynamically loading the module
        backend_name = self._ros_parameters['sim']['backend']
        module = importlib.import_module(
            '.backend.' + backend_name, package='crazyflie_sim'
        )
        class_ = getattr(module, 'Backend')
        self.backend = class_(self, names, initial_states)

        # initialize visualizations by dynamically loading the modules
        self.visualizations = []
        for vis_key in self._ros_parameters['sim']['visualizations']:
            if self._ros_parameters['sim']['visualizations'][vis_key]['enabled']:
                module = importlib.import_module(
                    '.visualization.' + str(vis_key), package='crazyflie_sim'
                )
                class_ = getattr(module, 'Visualization')
                if vis_key == 'rviz':
                    # special case for rviz, which needs the reference frames
                    vis = class_(
                        self,
                        self._ros_parameters['sim']['visualizations'][vis_key],
                        names,
                        initial_states,
                        reference_frames
                    )
                else:
                    vis = class_(
                        self,
                        self._ros_parameters['sim']['visualizations'][vis_key],
                        names,
                        initial_states
                    )
                self.visualizations.append(vis)

        controller_name = backend_name = self._ros_parameters['sim']['controller']

        # create robot SIL objects
        for name, initial_state in zip(names, initial_states):
            self.cfs[name] = CrazyflieSIL(
                name,
                initial_state.pos,
                controller_name,
                self.backend.time)

        # Parallelize only the independent Mellinger controller calls.
        if hasattr(os, 'sched_getaffinity'):
            available_cpus = len(os.sched_getaffinity(0))
        else:
            available_cpus = os.cpu_count() or 1

        worker_count = max(1, min(len(self.cfs), available_cpus))
        self._executor = ThreadPoolExecutor(max_workers=worker_count)
        self.get_logger().info(
            f'Controller thread pool started with {worker_count} workers'
        )

        self._use_vector_controller = os.environ.get(
            'CF_SIM_VECTOR_CONTROLLER',
            '1',
        ).strip().lower() not in ('0', 'false', 'off', 'no')
        self._vector_controller = (
            VectorizedCascadedController(self.backend)
            if self._use_vector_controller
            else None
        )
        self._vector_feedback_counter = 0
        self._vector_feedback_divider = 10
        self.get_logger().info(
            'Controller path: '
            + (
                'vectorized cascaded controller'
                if self._use_vector_controller
                else 'firmware controller'
            )
        )  # CF_SIM_VECTOR_CONTROLLER_PHASE5A

        self._vector_setpoint_cache = (
            VectorizedSetpointCache(self.backend)
            if self._use_vector_controller
            else None
        )
        self._vector_vis_counter = 0
        requested_vis_rate = float(
            os.environ.get('CF_SIM_VIS_RATE_HZ', '50.0')
        )
        if requested_vis_rate <= 0.0:
            raise ValueError('CF_SIM_VIS_RATE_HZ must be positive')
        physics_rate = 1.0 / float(self.backend.dt)
        self._vector_vis_divider = max(
            1,
            int(round(physics_rate / requested_vis_rate)),
        )
        self.get_logger().info(
            'Vectorized visualization rate: '
            f'{physics_rate / self._vector_vis_divider:.1f}Hz'
        )  # CF_SIM_CACHED_VECTOR_PLANNER_PHASE5B

        self._profile_report_steps = 1000
        self._profile_steps = 0
        self._profile_ns = {
            'setpoint': 0,
            'controller': 0,
            'physics': 0,
            'setstate': 0,
            'visualization': 0,
            'total': 0,
        }

        for name, _ in self.cfs.items():
            pub = self.create_publisher(
                    String,
                    name + '/robot_description',
                    rclpy.qos.QoSProfile(
                        depth=1,
                        durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL))

            msg = String()
            msg.data = self._ros_parameters['robot_description'].replace('$NAME', name)
            pub.publish(msg)

            self.create_service(
                Empty,
                name + '/emergency',
                partial(self._emergency_callback, name=name)
            )
            self.create_service(
                Takeoff,
                name + '/takeoff',
                partial(self._takeoff_callback, name=name)
            )
            self.create_service(
                Land,
                name + '/land',
                partial(self._land_callback, name=name)
            )
            self.create_service(
                GoTo,
                name + '/go_to',
                partial(self._go_to_callback, name=name)
            )
            self.create_service(
                StartTrajectory,
                name + '/start_trajectory',
                partial(self._start_trajectory_callback, name=name)
            )
            self.create_service(
                UploadTrajectory,
                name + '/upload_trajectory',
                partial(self._upload_trajectory_callback, name=name)
            )
            self.create_service(
                NotifySetpointsStop,
                name + '/notify_setpoints_stop',
                partial(self._notify_setpoints_stop_callback, name=name)
            )
            self.create_subscription(
                Twist,
                name + '/cmd_vel_legacy',
                partial(self._cmd_vel_legacy_changed, name=name),
                10
            )
            self.create_subscription(
                Hover,
                name + '/cmd_hover',
                partial(self._cmd_hover_changed, name=name),
                10
            )
            self.create_subscription(
                FullState,
                name + '/cmd_full_state',
                partial(self._cmd_full_state_changed, name=name),
                10
            )

        # Create services for the entire swarm and each individual crazyflie
        self.create_service(Takeoff, 'all/takeoff', self._takeoff_callback)
        self.create_service(Land, 'all/land', self._land_callback)
        self.create_service(GoTo, 'all/go_to', self._go_to_callback)
        self.create_service(StartTrajectory,
                            'all/start_trajectory',
                            self._start_trajectory_callback)

        # This is the last service to announce.
        # Can be used to check if the server is fully available.
        self.create_service(Empty, 'all/emergency', self._emergency_callback)

        # Batched formation updates for SIL choreography.
        # One ROS topic message replaces N synchronous per-drone go_to services.
        self._formation_last_targets = {}
        self._formation_frame_count = 0
        self.create_subscription(
            String,
            '/formation_targets',
            self._formation_targets_callback,
            1,
        )

        # Step as fast as possible. Multiple complete simulation steps may
        # be executed per ROS timer dispatch to amortize rclpy executor
        # overhead. This does not decimate control or physics.
        max_dt = 0.0 if 'max_dt' not in self._ros_parameters['sim'] \
            else self._ros_parameters['sim']['max_dt']

        try:
            self._burst_steps = int(
                os.environ.get('CF_SIM_BURST_STEPS', '4')
            )
        except ValueError as exc:
            raise ValueError(
                'CF_SIM_BURST_STEPS must be an integer'
            ) from exc

        if not 1 <= self._burst_steps <= 32:
            raise ValueError(
                'CF_SIM_BURST_STEPS must be between 1 and 32'
            )

        self.get_logger().info(
            f'Simulation burst size: {self._burst_steps} complete '
            'step(s) per ROS timer dispatch'
        )  # CF_SIM_BURST_STEPPING_PHASE5C

        self.timer = self.create_timer(max_dt, self._timer_callback)
        self.is_shutdown = False

    def on_shutdown_callback(self):
        if not self.is_shutdown:
            self._executor.shutdown(wait=True)
            self.backend.shutdown()
            for visualization in self.visualizations:
                visualization.shutdown()

            self.is_shutdown = True

    def _timer_callback(self):
        for _ in range(self._burst_steps):
            self._simulation_step()

    def _simulation_step(self):
        profile_start = time.perf_counter_ns()

        cf_list = list(self.cfs.values())

        setpoint_start = time.perf_counter_ns()
        desired_arrays = None
        if self._use_vector_controller:
            (
                states_desired,
                desired_pos,
                desired_vel,
                desired_quat,
            ) = self._vector_setpoint_cache.step(
                cf_list,
                self.backend.time(),
            )
            desired_arrays = (
                desired_pos,
                desired_vel,
                desired_quat,
            )
        else:
            states_desired = [cf.getSetpoint() for cf in cf_list]
        setpoint_end = time.perf_counter_ns()

        controller_start = setpoint_end
        if self._use_vector_controller:
            actions = self._vector_controller.step_arrays(
                *desired_arrays,
                cf_list,
            )
        else:
            actions = [
                    CrazyflieSIL.executeController(cf)
                    for cf in cf_list
                ]  # CF_SIM_CONTROLLER_MODE=serial_direct
        controller_end = time.perf_counter_ns()

        physics_start = controller_end
        states_next = self.backend.step(states_desired, actions)
        physics_end = time.perf_counter_ns()

        state_start = physics_end
        if self._use_vector_controller:
            # The custom controller reads backend arrays directly. Keep the
            # firmware state synchronized at 100 Hz for command compatibility,
            # rather than paying nine SWIG conversions every 1 ms step.
            self._vector_feedback_counter += 1
            if (
                self._vector_feedback_counter
                % self._vector_feedback_divider
                == 0
            ):
                for cf, state in zip(cf_list, states_next):
                    cf.setState(state)
        else:
            for cf, state in zip(cf_list, states_next):
                cf.setState(state)
        state_end = time.perf_counter_ns()

        visualization_start = state_end
        run_visualization = True
        if self._use_vector_controller:
            self._vector_vis_counter += 1
            run_visualization = (
                self._vector_vis_counter
                % self._vector_vis_divider
                == 0
            )

        if run_visualization:
            for vis in self.visualizations:
                vis.step(
                    self.backend.time(),
                    states_next,
                    states_desired,
                    actions,
                )
        visualization_end = time.perf_counter_ns()

        values = {
            'setpoint': setpoint_end - setpoint_start,
            'controller': controller_end - controller_start,
            'physics': physics_end - physics_start,
            'setstate': state_end - state_start,
            'visualization': visualization_end - visualization_start,
            'total': visualization_end - profile_start,
        }

        for key, value in values.items():
            self._profile_ns[key] += value

        self._profile_steps += 1

        if self._profile_steps >= self._profile_report_steps:
            total_ns = max(self._profile_ns['total'], 1)
            total_ms = total_ns / 1.0e6
            mean_ms = total_ms / self._profile_steps
            callback_rate = self._profile_steps / (total_ns / 1.0e9)

            def stage_text(key):
                stage_ns = self._profile_ns[key]
                stage_ms = stage_ns / 1.0e6 / self._profile_steps
                percentage = 100.0 * stage_ns / total_ns
                return f'{key}={stage_ms:.3f}ms/{percentage:.1f}%'

            self.get_logger().info(
                'SIM_PROFILE '
                f'N={len(cf_list)} '
                f'steps={self._profile_steps} '
                f'mean={mean_ms:.3f}ms '
                f'callback_rate={callback_rate:.1f}Hz | '
                + ' '.join(
                    stage_text(key)
                    for key in (
                        'setpoint',
                        'controller',
                        'physics',
                        'setstate',
                        'visualization',
                    )
                )
            )

            for key in self._profile_ns:
                self._profile_ns[key] = 0
            self._profile_steps = 0

    def _param_to_dict(self, param_ros):
        """Turn ROS 2 parameters from the node into a dict."""
        tree = {}
        for item in param_ros:
            t = tree
            for part in item.split('.'):
                if part == item.split('.')[-1]:
                    t = t.setdefault(part, param_ros[item].value)
                else:
                    t = t.setdefault(part, {})
        return tree

    def _emergency_callback(self, request, response, name='all'):
        self.get_logger().info(f'[{name}] emergency not yet implemented')

        return response

    def _takeoff_callback(self, request, response, name='all'):
        """Service callback to takeoff the crazyflie."""
        duration = float(request.duration.sec) + \
            float(request.duration.nanosec / 1e9)
        self.get_logger().info(
            f'[{name}] takeoff(height={request.height} m,'
            + f'duration={duration} s,'
            + f'group_mask={request.group_mask})'
        )
        cfs = self.cfs if name == 'all' else {name: self.cfs[name]}
        for _, cf in cfs.items():
            cf.takeoff(request.height, duration, request.group_mask)

        return response

    def _land_callback(self, request, response, name='all'):
        """Service callback to land the crazyflie."""
        duration = float(request.duration.sec) + \
            float(request.duration.nanosec / 1e9)
        self.get_logger().info(
            f'[{name}] land(height={request.height} m,'
            + f'duration={duration} s,'
            + f'group_mask={request.group_mask})'
        )
        cfs = self.cfs if name == 'all' else {name: self.cfs[name]}
        for _, cf in cfs.items():
            cf.land(request.height, duration, request.group_mask)

        return response

    def _go_to_callback(self, request, response, name='all'):
        """Service callback to have the crazyflie go to a position."""
        duration = float(request.duration.sec) + \
            float(request.duration.nanosec / 1e9)

        self.get_logger().info(
            """[%s] go_to(position=%f,%f,%f m,
             yaw=%f rad,
             duration=%f s,
             relative=%d,
             group_mask=%d)"""
            % (
                name,
                request.goal.x,
                request.goal.y,
                request.goal.z,
                request.yaw,
                duration,
                request.relative,
                request.group_mask,
            )
        )
        cfs = self.cfs if name == 'all' else {name: self.cfs[name]}
        for _, cf in cfs.items():
            cf.goTo([request.goal.x, request.goal.y, request.goal.z],
                    request.yaw, duration, request.relative, request.group_mask)

        return response


    def _formation_targets_callback(self, message):
        # Apply one batched formation frame directly to the SIL planners.
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError) as error:
            self.get_logger().warning(
                f'Ignoring invalid /formation_targets payload: {error}'
            )
            return

        targets = payload.get('targets')
        if not isinstance(targets, dict):
            self.get_logger().warning(
                'Ignoring /formation_targets payload without a targets object'
            )
            return

        try:
            duration = float(payload.get('duration', 0.35))
            yaw = float(payload.get('yaw', 0.0))
            relative = bool(payload.get('relative', False))
        except (TypeError, ValueError):
            self.get_logger().warning(
                'Ignoring /formation_targets payload with invalid metadata'
            )
            return

        duration = min(max(duration, 0.10), 10.0)
        changed_count = 0

        for name, raw_goal in targets.items():
            if name not in self.cfs:
                continue

            if not isinstance(raw_goal, (list, tuple)) or len(raw_goal) != 3:
                continue

            try:
                goal = tuple(float(value) for value in raw_goal)
            except (TypeError, ValueError):
                continue

            if not all(math.isfinite(value) for value in goal):
                continue

            previous = self._formation_last_targets.get(name)
            if previous is not None and max(
                abs(goal[index] - previous[index]) for index in range(3)
            ) <= 1.0e-6:
                continue

            try:
                self.cfs[name].goTo(
                    goal,
                    yaw,
                    duration,
                    relative,
                    0,
                )
            except Exception as error:
                self.get_logger().error(
                    f'Formation target rejected for {name}: {error}'
                )
                continue

            self._formation_last_targets[name] = goal
            changed_count += 1

        if changed_count:
            self._formation_frame_count += 1
            if (
                self._formation_frame_count == 1
                or self._formation_frame_count % 100 == 0
            ):
                self.get_logger().info(
                    'Applied batched formation frame '
                    f'{self._formation_frame_count}: '
                    f'{changed_count} changed drones, '
                    f'duration={duration:.3f}s'
                )

    def _notify_setpoints_stop_callback(self, request, response, name='all'):
        self.get_logger().info(f'[{name}] Notify setpoint stop not yet implemented')
        return response

    def _upload_trajectory_callback(self, request, response, name='all'):
        self.get_logger().info('[%s] Upload trajectory(id=%d)' % (name, request.trajectory_id))

        cfs = self.cfs if name == 'all' else {name: self.cfs[name]}
        for _, cf in cfs.items():
            pieces = []
            for piece in request.pieces:
                poly_x = piece.poly_x
                poly_y = piece.poly_y
                poly_z = piece.poly_z
                poly_yaw = piece.poly_yaw
                duration = float(piece.duration.sec) + \
                    float(piece.duration.nanosec / 1e9)
                pieces.append(TrajectoryPolynomialPiece(
                    poly_x,
                    poly_y,
                    poly_z,
                    poly_yaw,
                    duration))
            cf.uploadTrajectory(request.trajectory_id, request.piece_offset, pieces)

        return response

    def _start_trajectory_callback(self, request, response, name='all'):
        self.get_logger().info(
            '[%s] start_trajectory(id=%d, timescale=%f, reverse=%d, relative=%d, group_mask=%d)'
            % (
                name,
                request.trajectory_id,
                request.timescale,
                request.reversed,
                request.relative,
                request.group_mask,
            )
        )
        cfs = self.cfs if name == 'all' else {name: self.cfs[name]}
        for _, cf in cfs.items():
            cf.startTrajectory(
                request.trajectory_id,
                request.timescale,
                request.reversed,
                request.relative,
                request.group_mask)

        return response

    def _cmd_vel_legacy_changed(self, msg, name=''):
        """
        Topic update callback.

        Controls the attitude and thrust of the crazyflie with teleop.
        """
        self.get_logger().info('cmd_vel_legacy not yet implemented')

    def _cmd_hover_changed(self, msg, name=''):
        """
        Topic update callback for hover command.

        Used from the velocity multiplexer (vel_mux).
        """
        self.get_logger().info('cmd_hover not yet implemented')

    def _cmd_full_state_changed(self, msg, name):
        q = [msg.pose.orientation.w,
             msg.pose.orientation.x,
             msg.pose.orientation.y,
             msg.pose.orientation.z]
        rpy = rowan.to_euler(q, convention='xyz')

        self.cfs[name].cmdFullState(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
            [msg.acc.x, msg.acc.y, msg.acc.z],
            rpy[2],
            [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])


def main(args=None):

    rclpy.init(args=args)
    crazyflie_server = CrazyflieServer()
    rclpy.get_default_context().on_shutdown(crazyflie_server.on_shutdown_callback)

    try:
        rclpy.spin(crazyflie_server)
    except KeyboardInterrupt:
        crazyflie_server.on_shutdown_callback()
    finally:
        rclpy.try_shutdown()
        crazyflie_server.destroy_node()


if __name__ == '__main__':
    main()
