#!/usr/bin/env python3

"""
Preset Motion Control Example Script

Description:
  This script demonstrates how to execute preset motions (like waving or handshaking) using the SetMcPresetMotion service.
  Automatically handles state machine transitions for safe motion execution.

Prerequisites:
  - MC (Motion Control) service must be running
  - Robot must be in a safe environment for motion testing
  - The shared action switcher will transition to BIPED_WHOLE_BODY_CTRL

Usage:
  python3 set_preset_motion.py

Example:
  python3 set_preset_motion.py

Parameters:
  - None 
"""

import sys
import time

import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.srv import SetMcPresetMotion
from aimdk_msgs.msg import (
    McPresetMotion, RequestHeader
)

from common.mc_action_switcher import McActionSwitchOptions, McActionSwitcher

SERVICE_CALL_TIMEOUT_SEC = 2.0
MAX_RETRY_COUNT = 3

class SetMcPresetMotionClient(Node):
    def __init__(self):
        super().__init__('preset_motion_client')
        
        self.preset_client = self.create_client(
            SetMcPresetMotion, '/aimdk_5Fmsgs/srv/SetMcPresetMotion')
            
        self.get_logger().info('SetMcPresetMotion client node created.')
        self.wait_for_services()

    def wait_for_services(self):
        clients = [
            (self.preset_client, '/aimdk_5Fmsgs/srv/SetMcPresetMotion'),
        ]
        for client, name in clients:
            while not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f'Waiting for service {name}...')
        self.get_logger().info('All required services are available.')

    def call_service_with_retry(self, client, request, service_name: str, timeout_sec=None, max_retries=None):
        if timeout_sec is None:
            timeout_sec = SERVICE_CALL_TIMEOUT_SEC
        if max_retries is None:
            max_retries = MAX_RETRY_COUNT
            
        for i in range(max_retries):
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

            if future.done():
                return future

            self.get_logger().info(f'{service_name} attempt {i+1}/{max_retries} timed out, retrying...')
            time.sleep(0.2)
        
        self.get_logger().error(f'{service_name} failed after {max_retries} attempts')
        return None

    def ensure_ready_state(self) -> bool:
        switcher = McActionSwitcher(self)
        options = McActionSwitchOptions(
            source="preset_motion",
            total_timeout=30.0,
        )
        result = switcher.switch_to("BIPED_WHOLE_BODY_CTRL", options)
        if not result.success:
            current_action = result.current_action or "(unknown)"
            self.get_logger().error(
                "Failed to switch from "
                f"{current_action} to BIPED_WHOLE_BODY_CTRL: {result.message}"
            )
        return result.success

    def send_motion_request(self, motion_id: int) -> bool:
        if not self.ensure_ready_state():
            self.get_logger().error('Failed to prepare robot state for preset motion.')
            return False

        request = SetMcPresetMotion.Request()
        request.header = RequestHeader()
        request.header.stamp = self.get_clock().now().to_msg()
        request.source = 'preset_motion'
        request.motion = McPresetMotion()
        request.motion.value = motion_id
        request.interrupt = True

        self.get_logger().info(f'Sending preset motion request: ID={motion_id}')
        
        future = self.call_service_with_retry(
            self.preset_client, request, "SetMcPresetMotion"
        )
        if future is None:
            return False
            
        res = future.result()
        if res and res.response.header.code == 0:
            self.get_logger().info(f'Motion request accepted. Task ID: {res.response.task_id}')
            return True
        
        if res:
            self.get_logger().error(
                f'SetMcPresetMotion failed. '
                f'code={res.response.header.code} status={res.response.header.status.value} '
                f'reason={res.response.header.status.reason}'
            )
        
        return False


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        # Prompt user to refer to documentation
        print("\nPlease refer to the interface documentation for the list of supported motions for this model.")
        print("If you haven't found the motion list, you can choose recommended motions based on the robot model.")
        # Determine robot series (Q or T). In a real scenario this could be obtained from a parameter or config.
        robot_series = input("\nEnter robot series (Q/T): ").strip().upper()
        if robot_series == "T":
            motion_map = {1001: "raise", 1002: "wave", 1003: "handshake", 1004: "airkiss"}
        elif robot_series == "Q":
            motion_map = {3001: "wave", 3002: "handshake", 3003: "bump", 3004: "wave_hand"}
        else:
            print("Unknown series. Please enter 'Q' or 'T'.")
            sys.exit(1)
        print("\nAvailable Preset Motions:")
        for k, v in motion_map.items():
            print(f"  {k}: {v}")
        motion_id = int(input("\nEnter preset motion ID: "))
        node = SetMcPresetMotionClient()
        node.send_motion_request(motion_id)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if node: node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
