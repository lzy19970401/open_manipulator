#!/usr/bin/env python3
#
# Copyright 2024 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Shutdown guard for OpenMANIPULATOR-X gravity compensation control mode.
# Disables torque on Arm Dynamixel actuators (IDs 11-14) when GC mode exits.

import time

import rclpy
from dynamixel_interfaces.srv import SetDataToDxl
from lifecycle_msgs.msg import State
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.msg import TransitionEvent
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Header

DEFAULT_ARM_DXL_IDS = (11, 12, 13, 14)
TORQUE_ENABLE_ITEM = 'Torque Enable'
TORQUE_DISABLED = 0


class GcShutdownTorqueDisable(Node):
    """Disable Arm torque when GC mode stops."""

    def __init__(self):
        super().__init__('gc_shutdown_torque_disable')
        self.declare_parameter('arm_dxl_ids', list(DEFAULT_ARM_DXL_IDS))
        self.declare_parameter(
            'set_dxl_data_service', 'dynamixel_hardware_interface/set_dxl_data')
        self.declare_parameter(
            'controller_transition_topic',
            '/gravity_compensation_controller/transition_event')
        self.declare_parameter('hardware_write_settle_sec', 0.3)

        self._arm_dxl_ids = self.get_parameter('arm_dxl_ids').value
        self._service_name = self.get_parameter('set_dxl_data_service').value
        self._settle_sec = self.get_parameter('hardware_write_settle_sec').value
        self._torque_disabled = False

        callback_group = ReentrantCallbackGroup()
        self._set_dxl_data_client = self.create_client(
            SetDataToDxl, self._service_name, callback_group=callback_group)

        transition_topic = self.get_parameter('controller_transition_topic').value
        self.create_subscription(
            TransitionEvent,
            transition_topic,
            self._on_controller_transition,
            10,
            callback_group=callback_group,
        )

        self.get_logger().info(
            'GC shutdown guard active. After stopping the launch (Ctrl+C), wait until '
            'shutdown completes before manually moving the Arm.')

    def _on_controller_transition(self, msg: TransitionEvent) -> None:
        if (
            msg.transition.id == Transition.TRANSITION_DEACTIVATE
            and msg.goal_state.id == State.PRIMARY_STATE_INACTIVE
        ):
            self.disable_arm_torque('controller deactivate')

    def disable_arm_torque(self, reason: str) -> None:
        if self._torque_disabled:
            return
        self._torque_disabled = True

        if not self._set_dxl_data_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning(
                f'Skip Arm torque disable ({reason}): {self._service_name} unavailable.')
            return

        for dxl_id in self._arm_dxl_ids:
            request = SetDataToDxl.Request()
            request.header = Header()
            request.id = int(dxl_id)
            request.item_name = TORQUE_ENABLE_ITEM
            request.item_data = TORQUE_DISABLED

            future = self._set_dxl_data_client.call_async(request)
            start = time.monotonic()
            while not future.done() and (time.monotonic() - start) < 2.0:
                time.sleep(0.01)

            if not future.done():
                self.get_logger().error(
                    f'Timed out disabling torque on Dynamixel ID {dxl_id} ({reason}).')
                continue

            response = future.result()
            if response is not None and response.result:
                self.get_logger().info(
                    f'Torque disabled on Dynamixel ID {dxl_id} ({reason}).')
            else:
                self.get_logger().error(
                    f'Failed to disable torque on Dynamixel ID {dxl_id} ({reason}).')

        time.sleep(self._settle_sec)


def main(args=None):
    rclpy.init(args=args)
    node = GcShutdownTorqueDisable()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.disable_arm_torque('launch shutdown')
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
