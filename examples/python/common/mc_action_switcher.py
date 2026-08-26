#!/usr/bin/env python3

"""Q1 MC action switcher.

This module mirrors ``examples/cpp/common/mc_action_switcher.hpp``.  It plans
the state-machine route from the current MC action and waits for every
intermediate action before continuing to the requested target.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum, auto
import time
from typing import Deque, Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node

from aimdk_msgs.msg import (
    CommonRequest,
    CommonState,
    McActionCommand,
    McActionStatus,
    RequestHeader,
)
from aimdk_msgs.srv import GetMcAction, SetMcAction


@dataclass
class McActionSwitchOptions:
    """Timeouts and request settings for :class:`McActionSwitcher`.

    All durations are expressed in seconds.
    """

    source: str = "sdk_node"
    service_wait_timeout: float = 5.0
    service_call_timeout: float = 3.0
    poll_interval: float = 0.5
    minimum_set_action_interval: float = 0.5
    total_timeout: float = 20.0


class McActionSwitchError(Enum):
    """Reason why an action switch did not complete."""

    NONE = auto()
    INVALID_ARGUMENT = auto()
    SERVICE_UNAVAILABLE = auto()
    GET_ACTION_FAILED = auto()
    TARGET_NOT_ACTIVE = auto()
    NO_TRANSITION_PATH = auto()
    SET_ACTION_FAILED = auto()
    TIMEOUT = auto()
    SHUTDOWN = auto()


@dataclass
class McActionSwitchResult:
    """Outcome returned by :meth:`McActionSwitcher.switch_to`."""

    success: bool = False
    error: McActionSwitchError = McActionSwitchError.NONE
    current_action: str = ""
    target_action: str = ""
    message: str = ""


@dataclass
class _ActionState:
    action_desc: str
    status: int


@dataclass
class _PlannedTransition:
    command: str
    expected_action: str


class McActionSwitcher:
    """Synchronously switch MC to a target ``action_desc`` state."""

    _ACTION_RULES: Dict[str, Tuple[str, ...]] = {
        "PASSIVE_DEFAULT": (
            "BIPED_STAND_DEFAULT", "LYING_DEFAULT", "STAND_UP",
            "STORE_DEFAULT", "BIPED_GROUND_POSE", "LIE_FACE_DOWN",
            "LIE_FACE_UP",
        ),
        "DAMPING_DEFAULT": ("PASSIVE_DEFAULT",),
        "BIPED_STAND_DEFAULT": (
            "BIPED_WALK_RUN", "BIPED_WALK_RUN_DEFAULT", "STORE_DEFAULT",
            "BIPED_BLIND_TERRAIN",
        ),
        "BIPED_WHOLE_BODY_CTRL": (
            "BIPED_STAND_DEFAULT", "BIPED_WALK_RUN",
            "BIPED_WALK_RUN_DEFAULT", "BIPED_BLIND_TERRAIN", "LIE_FACE_UP",
            "LIE_FACE_DOWN", "STORE_DEFAULT", "BIPED_RECORD_UPPER",
        ),
        "BIPED_CUSTOM_UPPER": ("BIPED_STAND_DEFAULT", "BIPED_WALK_RUN"),
        "STORE_DEFAULT": (
            "BIPED_GROUND_POSE", "LIE_FACE_DOWN", "LIE_FACE_UP",
        ),
        "BIPED_WALK_RUN": (
            "BIPED_WHOLE_BODY_CTRL", "BIPED_STAND_DEFAULT",
            "BIPED_WALK_RUN_DEFAULT", "LIE_FACE_UP", "LIE_FACE_DOWN",
            "BIPED_BLIND_TERRAIN", "STORE_DEFAULT", "BIPED_RECORD_UPPER",
            "BIPED_CUSTOM_UPPER",
        ),
        "BIPED_WALK_RUN_DEFAULT": (
            "BIPED_STAND_DEFAULT", "BIPED_WHOLE_BODY_CTRL", "BIPED_WALK_RUN",
        ),
        "BIPED_BLIND_TERRAIN": ("BIPED_WALK_RUN",),
        "LYING_DEFAULT": ("STORE_DEFAULT",),
        "LIE_FACE_UP": ("LYING_DEFAULT",),
        "LIE_FACE_DOWN": ("LYING_DEFAULT",),
        "STAND_UP": ("BIPED_WALK_RUN", "BIPED_BLIND_TERRAIN"),
        "BIPED_GROUND_POSE": ("STAND_UP", "STORE_DEFAULT"),
        "BIPED_RECORD_UPPER": (
            "BIPED_WHOLE_BODY_CTRL", "BIPED_BLIND_TERRAIN",
            "BIPED_WALK_RUN", "BIPED_WALK_RUN_DEFAULT",
        ),
    }

    _SKIP_ACTIONS: Set[str] = {
        "PASSIVE_DEFAULT", "BIPED_WALK_RUN", "BIPED_BLIND_TERRAIN",
        "LIE_FACE_UP", "LIE_FACE_DOWN", "STAND_UP", "STORE_DEFAULT",
        "BIPED_STAND_DEFAULT", "BIPED_WALK_RUN_DEFAULT",
    }
    _AUTOMATIC_NEXT_ACTIONS: Dict[str, str] = {
        "STAND_UP": "BIPED_WALK_RUN",
    }

    def __init__(
        self,
        node: Optional[Node],
        set_action_service: str = "/aimdk_5Fmsgs/srv/SetMcAction",
        get_action_service: str = "/aimdk_5Fmsgs/srv/GetMcAction",
    ) -> None:
        self._node = node
        self._set_action_service = set_action_service
        self._get_action_service = get_action_service
        self._set_action_client = None
        self._get_action_client = None
        if node is not None:
            self._set_action_client = node.create_client(
                SetMcAction, set_action_service
            )
            self._get_action_client = node.create_client(
                GetMcAction, get_action_service
            )

    def switch_to(
        self,
        target_action: str,
        options: Optional[McActionSwitchOptions] = None,
    ) -> McActionSwitchResult:
        """Switch to ``target_action`` and wait until it is active."""

        options = options or McActionSwitchOptions()
        result = McActionSwitchResult(target_action=target_action)
        if self._node is None or not target_action:
            return self._fail(
                result,
                McActionSwitchError.INVALID_ARGUMENT,
                "Node and target_action must be provided.",
            )
        if not self._wait_for_services(options, result):
            return result

        current = self._get_current_action(options)
        if current is None:
            return self._fail(
                result,
                McActionSwitchError.GET_ACTION_FAILED,
                "GetMcAction did not return the current action.",
            )
        result.current_action = current.action_desc
        if current.action_desc == target_action:
            if self._is_active(current):
                result.success = True
                result.message = "Robot is already in the requested action."
                return result
            return self._fail(
                result,
                McActionSwitchError.TARGET_NOT_ACTIVE,
                "MC reports the target action as IDLE; refusing to leave "
                f"{target_action} through a fallback transition.",
            )

        path = self._plan_path(current.action_desc, target_action)
        if not path:
            return self._fail(
                result,
                McActionSwitchError.NO_TRANSITION_PATH,
                "No Q1 action_ruler path from "
                f"{current.action_desc} to {target_action}.",
            )

        deadline = time.monotonic() + options.total_timeout
        next_request_time = time.monotonic()
        for transition in path:
            if not rclpy.ok():
                return self._fail(
                    result,
                    McActionSwitchError.SHUTDOWN,
                    "ROS shutdown while switching actions.",
                )
            if time.monotonic() >= deadline:
                return self._fail(
                    result,
                    McActionSwitchError.TIMEOUT,
                    "Timed out while switching actions.",
                )

            time.sleep(max(0.0, next_request_time - time.monotonic()))
            if not self._request_action(transition.command, options):
                return self._fail(
                    result,
                    McActionSwitchError.SET_ACTION_FAILED,
                    f"SetMcAction rejected {transition.command}.",
                )
            next_request_time = time.monotonic() + options.minimum_set_action_interval

            reached = self._wait_for_action(
                transition.expected_action, options, deadline
            )
            if reached is None:
                if not rclpy.ok():
                    return self._fail(
                        result,
                        McActionSwitchError.SHUTDOWN,
                        "ROS shutdown while waiting for "
                        f"{transition.expected_action}.",
                    )
                return self._fail(
                    result,
                    McActionSwitchError.TIMEOUT,
                    "Timed out waiting for "
                    f"{transition.expected_action} after requesting "
                    f"{transition.command}.",
                )
            current = reached
            result.current_action = current.action_desc

        if current.action_desc != target_action:
            return self._fail(
                result,
                McActionSwitchError.TIMEOUT,
                "MC did not reach the requested target action.",
            )
        result.success = True
        result.message = "Robot reached the requested action."
        return result

    def _fail(
        self,
        result: McActionSwitchResult,
        error: McActionSwitchError,
        message: str,
    ) -> McActionSwitchResult:
        result.success = False
        result.error = error
        result.message = message
        return result

    def _is_active(self, action: _ActionState) -> bool:
        return action.status != McActionStatus.IDLE

    def _is_bridge_action(self, action: str) -> bool:
        return action == "BIPED_WALK_RUN"

    def _plan_path(
        self, current_action: str, target_action: str
    ) -> List[_PlannedTransition]:
        pending: Deque[str] = deque([current_action])
        visited = {current_action}
        previous: Dict[str, Tuple[str, _PlannedTransition]] = {}

        while pending:
            current = pending.popleft()
            candidates = list(self._ACTION_RULES.get(current, ()))
            if target_action in ("PASSIVE_DEFAULT", "DAMPING_DEFAULT"):
                candidates.append(target_action)

            for candidate in candidates:
                if candidate == target_action:
                    transition = _PlannedTransition(candidate, candidate)
                    arrival = candidate
                elif candidate in self._AUTOMATIC_NEXT_ACTIONS:
                    arrival = self._AUTOMATIC_NEXT_ACTIONS[candidate]
                    transition = _PlannedTransition(candidate, arrival)
                elif (
                    current == "DAMPING_DEFAULT"
                    and candidate == "PASSIVE_DEFAULT"
                ):
                    transition = _PlannedTransition(candidate, candidate)
                    arrival = candidate
                elif (
                    self._is_bridge_action(candidate)
                    or candidate not in self._SKIP_ACTIONS
                ):
                    transition = _PlannedTransition(candidate, candidate)
                    arrival = candidate
                else:
                    continue

                if arrival in visited:
                    continue
                visited.add(arrival)
                previous[arrival] = (current, transition)
                if arrival == target_action:
                    path: List[_PlannedTransition] = []
                    cursor = target_action
                    while cursor != current_action:
                        previous_entry = previous.get(cursor)
                        if previous_entry is None:
                            return []
                        cursor, transition = previous_entry
                        path.append(transition)
                    path.reverse()
                    return path
                pending.append(arrival)
        return []

    def _wait_for_services(
        self, options: McActionSwitchOptions, result: McActionSwitchResult
    ) -> bool:
        assert self._set_action_client is not None
        assert self._get_action_client is not None
        if not self._set_action_client.wait_for_service(
            timeout_sec=options.service_wait_timeout
        ):
            self._fail(
                result,
                McActionSwitchError.SERVICE_UNAVAILABLE,
                f"SetMcAction service is unavailable: {self._set_action_service}",
            )
            return False
        if not self._get_action_client.wait_for_service(
            timeout_sec=options.service_wait_timeout
        ):
            self._fail(
                result,
                McActionSwitchError.SERVICE_UNAVAILABLE,
                f"GetMcAction service is unavailable: {self._get_action_service}",
            )
            return False
        return True

    def _get_current_action(
        self, options: McActionSwitchOptions
    ) -> Optional[_ActionState]:
        assert self._node is not None
        assert self._get_action_client is not None
        for attempt in range(1, 4):
            request = GetMcAction.Request()
            request.request = CommonRequest()
            request.request.header.stamp = self._node.get_clock().now().to_msg()
            future = self._get_action_client.call_async(request)
            rclpy.spin_until_future_complete(
                self._node, future, timeout_sec=options.service_wait_timeout
            )
            if future.done():
                try:
                    response = future.result()
                except Exception as error:  # Service exceptions are retryable.
                    self._node.get_logger().warning(
                        f"GetMcAction attempt {attempt}/3 failed: {error}"
                    )
                else:
                    if response is None or response.header.code != 0:
                        return None
                    return _ActionState(
                        action_desc=response.info.action_desc,
                        status=response.info.status.value,
                    )
            else:
                self._node.get_logger().warning(
                    f"GetMcAction attempt {attempt}/3 timed out or was interrupted."
                )

            if attempt < 3:
                time.sleep(options.poll_interval)
        return None

    def _request_action(
        self, action_desc: str, options: McActionSwitchOptions
    ) -> bool:
        assert self._node is not None
        assert self._set_action_client is not None
        time.sleep(0.5)
        request = SetMcAction.Request()
        request.header = RequestHeader()
        request.header.stamp = self._node.get_clock().now().to_msg()
        request.source = options.source
        request.command = McActionCommand()
        request.command.action_desc = action_desc

        self._node.get_logger().info(f"Requesting MC action: {action_desc}")
        future = self._set_action_client.call_async(request)
        rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=options.service_call_timeout
        )
        if not future.done():
            return False
        try:
            response = future.result()
        except Exception as error:
            self._node.get_logger().warning(
                f"SetMcAction request for {action_desc} failed: {error}"
            )
            return False
        return (
            response is not None
            and response.response.header.code == 0
            and response.response.state.value == CommonState.SUCCESS
        )

    def _wait_for_action(
        self,
        expected_action: str,
        options: McActionSwitchOptions,
        deadline: float,
    ) -> Optional[_ActionState]:
        assert self._node is not None
        while rclpy.ok() and time.monotonic() < deadline:
            current = self._get_current_action(options)
            if (
                current is not None
                and current.action_desc == expected_action
                and self._is_active(current)
            ):
                self._node.get_logger().info(
                    "MC reached action: "
                    f"{expected_action} (status={current.status})"
                )
                return current
            time.sleep(options.poll_interval)
        return None
