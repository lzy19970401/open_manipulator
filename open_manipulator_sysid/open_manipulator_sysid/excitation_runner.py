"""Online excitation: approach q₀, hold, stream Fourier trajectory to arm_controller."""

from __future__ import annotations

from typing import Dict, Optional, Sequence

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from open_manipulator_sysid.excitation_trajectory import (
    ARM_JOINTS,
    ExcitationTrajectory,
    build_excitation_trajectory_messages,
    default_config_path,
    required_approach_duration,
    validate_approach_segment,
)


def build_excitation_trajectory(
    trajectory: ExcitationTrajectory,
    *,
    start_positions: Dict[str, float],
    approach_duration_s: float,
    hold_duration_s: float,
    num_periods: int,
    sample_dt_s: float,
    settle_duration_s: float = 0.0,
) -> JointTrajectory:
    """Build ROS JointTrajectory for arm_controller FollowJointTrajectory."""
    traj = JointTrajectory()
    traj.joint_names = list(ARM_JOINTS)

    for sample in build_excitation_trajectory_messages(
        trajectory,
        start_positions=start_positions,
        approach_duration_s=approach_duration_s,
        hold_duration_s=hold_duration_s,
        num_periods=num_periods,
        sample_dt_s=sample_dt_s,
        settle_duration_s=settle_duration_s,
    ):
        point = JointTrajectoryPoint()
        point.positions = [sample.position[joint] for joint in ARM_JOINTS]
        point.velocities = [sample.velocity[joint] for joint in ARM_JOINTS]
        point.accelerations = [sample.acceleration[joint] for joint in ARM_JOINTS]
        sec = int(sample.time_s)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = int((sample.time_s - sec) * 1e9)
        traj.points.append(point)

    return traj


class ExcitationRunner(Node):
    """Send sysid excitation trajectory to /arm_controller/follow_joint_trajectory."""

    def __init__(self) -> None:
        super().__init__('excitation_runner')
        self.declare_parameter('config', str(default_config_path()))
        self.declare_parameter('action_topic', '/arm_controller/follow_joint_trajectory')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('sample_dt_s', 0.01)
        self.declare_parameter('excitation_periods', 1)
        self.declare_parameter('position_tolerance', 0.02)

        config_path = self.get_parameter('config').get_parameter_value().string_value
        self._trajectory = ExcitationTrajectory.from_yaml(config_path)
        self._trajectory.validate_period()

        with open(config_path, 'r', encoding='utf-8') as handle:
            import yaml

            config = yaml.safe_load(handle)
        approach_cfg = config.get('approach', {})
        settle_cfg = config.get('settle', {})
        safety_cfg = config.get('safety', {})
        self._min_approach_duration_s = float(approach_cfg.get('duration_s', 5.0))
        self._hold_duration_s = float(approach_cfg.get('hold_duration_s', 2.0))
        self._settle_duration_s = float(settle_cfg.get('hold_duration_s', 2.0))
        self._velocity_abort_margin = float(
            safety_cfg.get('velocity_abort_margin_rad_s', 0.0)
        )

        self._sample_dt_s = float(self.get_parameter('sample_dt_s').value)
        self._excitation_periods = int(self.get_parameter('excitation_periods').value)
        self._position_tolerance = float(self.get_parameter('position_tolerance').value)
        self._action_topic = self.get_parameter('action_topic').value
        self._joint_states_topic = self.get_parameter('joint_states_topic').value

        self._action_client = ActionClient(
            self, FollowJointTrajectory, self._action_topic
        )
        self._joint_states: Optional[JointState] = None
        self._goal_sent = False
        self._aborted = False

        self.create_subscription(
            JointState,
            self._joint_states_topic,
            self._joint_state_callback,
            10,
        )
        self.create_timer(0.5, self._try_start_excitation)

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
        position = {joint: msg.position[msg.name.index(joint)] for joint in ARM_JOINTS}
        velocity = {
            joint: msg.velocity[msg.name.index(joint)] if msg.velocity else 0.0
            for joint in ARM_JOINTS
        }
        for joint in ARM_JOINTS:
            limits = self._trajectory._limits[joint]  # noqa: SLF001 — safety uses same limits
            q = position[joint]
            qd = velocity[joint]
            if q < limits.soft_lower or q > limits.soft_upper:
                self._abort(f'{joint} position {q:.4f} rad outside soft limits')
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
        rclpy.shutdown()

    def _current_arm_positions(self) -> Optional[Dict[str, float]]:
        if self._joint_states is None:
            return None
        msg = self._joint_states
        if not set(ARM_JOINTS).issubset(set(msg.name)):
            return None
        return {joint: msg.position[msg.name.index(joint)] for joint in ARM_JOINTS}

    def _try_start_excitation(self) -> None:
        if self._goal_sent or self._aborted:
            return
        if not self._action_client.server_is_ready():
            return

        start_positions = self._current_arm_positions()
        if start_positions is None:
            return

        q0 = self._trajectory.q0
        approach_duration_s = required_approach_duration(
            start_positions,
            q0,
            joints=ARM_JOINTS,
            max_velocity=self._trajectory._max_velocity,  # noqa: SLF001
            min_duration_s=self._min_approach_duration_s,
        )
        validate_approach_segment(
            start_positions,
            q0,
            joints=ARM_JOINTS,
            approach_duration_s=approach_duration_s,
            max_velocity=self._trajectory._max_velocity,  # noqa: SLF001
            max_acceleration=self._trajectory._max_acceleration,  # noqa: SLF001
        )

        self._goal_sent = True
        if approach_duration_s > self._min_approach_duration_s + 1e-6:
            self.get_logger().info(
                f'Extended approach duration to {approach_duration_s:.1f}s '
                f'(min {self._min_approach_duration_s:.1f}s) for velocity cap'
            )
        self.get_logger().info(
            f'Starting excitation: approach {approach_duration_s:.1f}s, '
            f'hold {self._hold_duration_s:.1f}s, '
            f'{self._excitation_periods} period(s) of {self._trajectory.period_s:.1f}s, '
            f'settle {self._settle_duration_s:.1f}s'
        )

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = build_excitation_trajectory(
            self._trajectory,
            start_positions=start_positions,
            approach_duration_s=approach_duration_s,
            hold_duration_s=self._hold_duration_s,
            num_periods=self._excitation_periods,
            sample_dt_s=self._sample_dt_s,
            settle_duration_s=self._settle_duration_s,
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
        if self._aborted:
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
            rclpy.shutdown()
            return

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
        else:
            self.get_logger().info('Excitation trajectory completed successfully')
        rclpy.shutdown()


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = ExcitationRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
