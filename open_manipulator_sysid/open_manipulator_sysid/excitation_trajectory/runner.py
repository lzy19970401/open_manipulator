"""Online excitation: start → q₀ → C² ingress → Fourier → C² egress → hold q₀."""

from __future__ import annotations

import subprocess
from typing import Dict, Optional, Sequence

import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from open_manipulator_sysid.excitation_trajectory import (
    ARM_JOINTS,
    ExcitationTrajectory,
    TrajectorySample,
    build_sysid_runtime_trajectory_messages,
    default_config_path,
    describe_runtime_schedule,
    required_c2_transition_duration,
    required_quintic_duration,
    runtime_config_from_yaml,
)


def build_excitation_trajectory(
    trajectory: ExcitationTrajectory,
    *,
    start_positions: Dict[str, float],
    runtime: Dict[str, float],
    num_periods: int,
    sample_dt_s: float,
) -> JointTrajectory:
    """Build ROS JointTrajectory for arm_controller FollowJointTrajectory."""
    samples, _schedule = build_sysid_runtime_trajectory_messages(
        trajectory,
        start_positions=start_positions,
        to_q0_duration_s=runtime['to_q0_duration_s'],
        ingress_spline_duration_s=runtime['ingress_spline_duration_s'],
        num_periods=num_periods,
        egress_spline_duration_s=runtime['egress_spline_duration_s'],
        q0_hold_duration_s=runtime['q0_hold_duration_s'],
        sample_dt_s=sample_dt_s,
        discard_periods=int(runtime['discard_periods']),
    )

    traj = JointTrajectory()
    traj.joint_names = list(ARM_JOINTS)

    for sample in samples:
        point = JointTrajectoryPoint()
        point.positions = [sample.position[joint] for joint in ARM_JOINTS]
        point.velocities = [sample.velocity[joint] for joint in ARM_JOINTS]
        point.accelerations = [sample.acceleration[joint] for joint in ARM_JOINTS]
        sec = int(sample.time_s)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = int((sample.time_s - sec) * 1e9)
        traj.points.append(point)

    return traj


def stop_excitation_bag_record() -> None:
    subprocess.run(
        [
            'bash',
            '-c',
            'pkill -INT -f "ros2 bag record" 2>/dev/null || true',
        ],
        check=False,
    )


def stop_joint_states_bag_record() -> None:
    """Backward-compatible alias for excitation bag recording shutdown."""
    stop_excitation_bag_record()


class ExcitationRunner(Node):
    """Send sysid excitation trajectory to /arm_controller/follow_joint_trajectory."""

    def __init__(self) -> None:
        super().__init__('excitation_runner')
        self.declare_parameter('config', str(default_config_path()))
        self.declare_parameter('action_topic', '/arm_controller/follow_joint_trajectory')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('sample_dt_s', 0.01)
        self.declare_parameter('excitation_periods', 3)
        self.declare_parameter('position_tolerance', 0.02)
        self.declare_parameter('stop_bag_on_complete', True)

        config_path = self.get_parameter('config').get_parameter_value().string_value
        with open(config_path, 'r', encoding='utf-8') as handle:
            config = yaml.safe_load(handle)

        self._trajectory = ExcitationTrajectory.from_yaml(config_path)
        self._trajectory.validate_period()
        self._runtime = runtime_config_from_yaml(config)
        safety_cfg = config.get('safety', {})
        self._velocity_abort_margin = float(
            safety_cfg.get('velocity_abort_margin_rad_s', 0.05)
        )

        self._sample_dt_s = float(self.get_parameter('sample_dt_s').value)
        self._excitation_periods = int(self.get_parameter('excitation_periods').value)
        self._position_tolerance = float(self.get_parameter('position_tolerance').value)
        self._stop_bag_on_complete = bool(
            self.get_parameter('stop_bag_on_complete').value
        )
        self._action_topic = self.get_parameter('action_topic').value
        self._joint_states_topic = self.get_parameter('joint_states_topic').value

        self._action_client = ActionClient(
            self, FollowJointTrajectory, self._action_topic
        )
        self._joint_states: Optional[JointState] = None
        self._goal_send_attempted = False
        self._goal_accepted = False
        self._motion_complete = False
        self._aborted = False

        self.create_subscription(
            JointState,
            self._joint_states_topic,
            self._joint_state_callback,
            10,
        )
        self.create_timer(0.5, self._try_start_excitation)

        self.get_logger().info(
            f'Excitation config: {config_path} '
            f'(coefficients_source={self._trajectory.coefficients_source})'
        )
        self.get_logger().info('Waiting for action server and joint states...')
        self.get_logger().info(f'Action topic: {self._action_topic}')

    def _joint_state_callback(self, msg: JointState) -> None:
        if self._aborted:
            return
        if not set(ARM_JOINTS).issubset(set(msg.name)):
            return

        self._joint_states = msg
        self._check_live_safety(msg)

    def _check_live_safety(self, msg: JointState) -> None:
        if not self._goal_accepted or self._motion_complete:
            return
        position = {joint: msg.position[msg.name.index(joint)] for joint in ARM_JOINTS}
        velocity = {
            joint: msg.velocity[msg.name.index(joint)] if msg.velocity else 0.0
            for joint in ARM_JOINTS
        }
        for joint in ARM_JOINTS:
            limits = self._trajectory._limits[joint]  # noqa: SLF001
            q = position[joint]
            qd = velocity[joint]
            if q < limits.hard_lower or q > limits.hard_upper:
                self._abort(f'{joint} position {q:.4f} rad outside joint limits')
                return
            abort_cap = limits.max_velocity + self._velocity_abort_margin
            if abs(qd) > abort_cap + 1e-6:
                self._abort(
                    f'{joint} velocity {qd:.4f} rad/s exceeds cap '
                    f'{limits.max_velocity:.4f} rad/s '
                    f'(margin {self._velocity_abort_margin:.4f})'
                )

    def _abort(self, reason: str) -> None:
        if self._aborted:
            return
        self._aborted = True
        self.get_logger().error(f'Excitation aborted: {reason}')
        if self._stop_bag_on_complete:
            stop_joint_states_bag_record()
        rclpy.shutdown()

    def _current_arm_positions(self) -> Optional[Dict[str, float]]:
        if self._joint_states is None:
            return None
        msg = self._joint_states
        if not set(ARM_JOINTS).issubset(set(msg.name)):
            return None
        return {joint: msg.position[msg.name.index(joint)] for joint in ARM_JOINTS}

    def _try_start_excitation(self) -> None:
        if self._goal_send_attempted or self._aborted:
            return
        if not self._action_client.server_is_ready():
            return

        start_positions = self._current_arm_positions()
        if start_positions is None:
            return

        runtime = dict(self._runtime)
        max_vel = self._trajectory._max_velocity  # noqa: SLF001
        max_acc = self._trajectory._max_acceleration  # noqa: SLF001
        q0 = self._trajectory.q0

        runtime['to_q0_duration_s'] = required_quintic_duration(
            start_positions,
            q0,
            joints=ARM_JOINTS,
            max_velocity=max_vel,
            min_duration_s=runtime['to_q0_duration_s'],
        )

        excitation_start, _excitation_end = self._trajectory.period_boundary_states()
        q0_state = TrajectorySample(
            time_s=0.0,
            position=dict(q0),
            velocity={joint: 0.0 for joint in ARM_JOINTS},
            acceleration={joint: 0.0 for joint in ARM_JOINTS},
        )
        runtime['ingress_spline_duration_s'] = required_c2_transition_duration(
            q0_state,
            excitation_start,
            joints=ARM_JOINTS,
            max_velocity=max_vel,
            max_acceleration=max_acc,
            min_duration_s=runtime['ingress_spline_duration_s'],
        )
        runtime['egress_spline_duration_s'] = required_c2_transition_duration(
            self._trajectory.sample(
                self._excitation_periods * self._trajectory.period_s,
                wrap=False,
            ),
            q0_state,
            joints=ARM_JOINTS,
            max_velocity=max_vel,
            max_acceleration=max_acc,
            min_duration_s=runtime['egress_spline_duration_s'],
        )

        self._goal_send_attempted = True
        schedule = describe_runtime_schedule(
            self._trajectory,
            start_positions=start_positions,
            to_q0_duration_s=runtime['to_q0_duration_s'],
            ingress_spline_duration_s=runtime['ingress_spline_duration_s'],
            num_periods=self._excitation_periods,
            egress_spline_duration_s=runtime['egress_spline_duration_s'],
            q0_hold_duration_s=runtime['q0_hold_duration_s'],
            discard_periods=int(runtime['discard_periods']),
        )

        self.get_logger().info(
            f'Starting sysid motion: {self._excitation_periods} excitation period(s) '
            f'of {self._trajectory.period_s:.1f}s, total {schedule.total_duration_s:.1f}s'
        )
        self.get_logger().info(
            'Runtime schedule: '
            f"current→q₀ [0,{schedule.to_q0_duration_s:.1f}s) → "
            f"C² ingress [{schedule.to_q0_duration_s:.1f},"
            f'{schedule.excitation_start_s:.1f}s) → '
            f'Fourier [{schedule.excitation_start_s:.1f},'
            f'{schedule.excitation_end_s:.1f}s) → '
            f'C² egress [{schedule.excitation_end_s:.1f},'
            f'{schedule.excitation_end_s + schedule.egress_spline_duration_s:.1f}s) → '
            f'hold q₀'
        )
        self.get_logger().info(
            f'Bag record window (relative to motion t=0): '
            f'[{schedule.bag_record_start_s:.1f}, {schedule.bag_record_end_s:.1f}) s'
        )
        self.get_logger().info(
            f'Identification window (discard {schedule.discard_periods} period(s)): '
            f'[{schedule.identification_start_s:.1f}, {schedule.identification_end_s:.1f}) s'
        )

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = build_excitation_trajectory(
            self._trajectory,
            start_positions=start_positions,
            runtime=runtime,
            num_periods=self._excitation_periods,
            sample_dt_s=self._sample_dt_s,
        )
        goal_msg.path_tolerance = []
        goal_msg.goal_tolerance = []
        goal_msg.goal_time_tolerance.sec = 0
        goal_msg.goal_time_tolerance.nanosec = 0

        send_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback,
        )
        send_future.add_done_callback(self._goal_response_callback)

    def _feedback_callback(self, feedback_msg) -> None:
        if self._aborted or self._motion_complete:
            return
        actual = feedback_msg.feedback.actual
        if not actual or not actual.positions:
            return
        msg = JointState()
        msg.name = list(ARM_JOINTS)
        msg.position = list(actual.positions[: len(ARM_JOINTS)])
        msg.velocity = (
            list(actual.velocities[: len(ARM_JOINTS)])
            if actual.velocities
            else []
        )
        self._check_live_safety(msg)

    def _goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('FollowJointTrajectory goal rejected')
            if self._stop_bag_on_complete:
                stop_joint_states_bag_record()
            rclpy.shutdown()
            return

        self._goal_accepted = True
        self.get_logger().info('FollowJointTrajectory goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future) -> None:
        if self._aborted:
            return
        result = future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f'Excitation finished with error code {result.error_code}: '
                f'{result.error_string}'
            )
            if self._stop_bag_on_complete:
                stop_joint_states_bag_record()
            rclpy.shutdown()
            return

        self._motion_complete = True
        self.get_logger().info('Sysid excitation trajectory completed successfully')
        if self._stop_bag_on_complete:
            stop_joint_states_bag_record()
            self.get_logger().info('Stopped rosbag recording (excitation bag)')
        self.get_logger().info(
            'Arm holding at q₀ with position control active. '
            'Press Ctrl+C in this terminal when finished.'
        )


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = ExcitationRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutdown requested — controllers remain active until launch exits.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
