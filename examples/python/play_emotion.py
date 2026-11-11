#!/usr/bin/env python3

"""
Emotion Playback Example Script

Description:
  This script demonstrates how to play robot emotions using the PlayEmotion service.
  Supports built-in emotion IDs, custom emotion file paths, and image display.

Prerequisites:
  - Robot emotion service must be running
  - Emotion files must be available on the robot

Usage:
  python3 play_emotion.py --ros-args -p type:=<type> -p emotion_ids:="[...]" -p file_paths:="[...]"

Example:
  # Play built-in emotion with ID 3003
  python3 play_emotion.py --ros-args -p type:=emotion -p emotion_ids:="[3003]"

  # Play custom emotion from file
  python3 play_emotion.py --ros-args -p type:=file -p file_paths:="['/path/to/emotion.mp4']"

  # Display an image
  python3 play_emotion.py --ros-args -p type:=image -p file_paths:="['/path/to/image.png']"

Parameters:
  - type: Emotion type, 'emotion', 'file' or 'image' (default: emotion)
  - emotion_ids: List of built-in emotion IDs to play (default: [3003])
  - file_paths: List of custom emotion file paths or image URLs (optional)
"""

import rclpy
import time

import rclpy.logging
from rclpy.node import Node
from rclpy.parameter import Parameter

from aimdk_msgs.msg import CommonRequest, CommonState
from aimdk_msgs.srv import PlayEmotion


class PlayEmotionClient(Node):
    def __init__(self):
        super().__init__("play_emotion_client")
        self.declare_parameter("type", "emotion")
        self.declare_parameter("emotion_ids", [3003])
        self.declare_parameter("file_paths", Parameter.Type.STRING_ARRAY)

        self.type = self.get_parameter("type").value
        self.emotion_ids = self.get_parameter_or(
            "emotion_ids",
            Parameter("emotion_ids", Parameter.Type.INTEGER_ARRAY, [3003]),
        ).value
        self.file_paths = self.get_parameter_or(
            "file_paths",
            Parameter("file_paths", Parameter.Type.STRING_ARRAY, []),
        ).value
        self.priority = 0
        self.loop_count = 1

        self.client = self.create_client(PlayEmotion, "/aimdk_5Fmsgs/srv/PlayEmotion")
        self.get_logger().info("PlayEmotion client node created.")

        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for service...")
        self.get_logger().info("Service available, ready to send request.")

    def send_request(self) -> bool:
        try:
            if not self.validate_parameters():
                return False

            ok = self._call_service(self.type, list(self.emotion_ids), list(self.file_paths))

            if not ok:
                self.get_logger().error("PlayEmotion request failed after all attempts.")
                return False

            return True
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"Exception occurred: {error}")
            return False

    def _call_service(self, msg_type, emotion_ids, file_paths) -> bool:
        request = PlayEmotion.Request()
        request.header = CommonRequest()
        request.header.header.stamp = self.get_clock().now().to_msg()
        request.type = msg_type
        request.priority = self.priority
        request.loop_count = self.loop_count
        request.emotion_ids = emotion_ids
        request.file_paths = file_paths

        self.get_logger().info(
            f"Sending PlayEmotion request: type={msg_type}, "
            f"emotion_ids={emotion_ids}, file_paths={file_paths}"
        )

        for i in range(3):
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            if future.done():
                break

            self.get_logger().info(f'trying ... [{i}]')
            time.sleep(0.2)

        if not future.done():
            return False

        response = future.result()
        if response is None:
            return False

        code = response.header.header.code
        status = response.header.status.value
        if code == 0 or status == CommonState.SUCCESS:
            self.get_logger().info(f"Request accepted (code={code}, status={status}).")
            return True

        self.get_logger().error(
            f"PlayEmotion failed. "
            f"code={code} status={status} "
            f"msg={response.header.message}"
        )
        return False

    def validate_parameters(self) -> bool:
        if self.type not in ("emotion", "file", "image"):
            self.get_logger().error(
                f"Invalid parameter 'type': {self.type}. Use 'emotion', 'file' or 'image'."
            )
            return False

        if self.type == "emotion" and not self.emotion_ids:
            self.get_logger().error(
                "Parameter 'emotion_ids' must be set when type=emotion."
            )
            return False

        if self.type in ("file", "image") and not self.file_paths:
            self.get_logger().error(
                "Parameter 'file_paths' must be set when type=file or type=image."
            )
            return False

        return True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PlayEmotionClient()
        ok = node.send_request()
        return 0 if ok else 1
    except Exception as error:  # noqa: BLE001
        rclpy.logging.get_logger("main").error(
            f"Program exited with exception: {error}"
        )
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
