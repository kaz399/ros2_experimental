#!/usr/bin/env python3
import random
import time
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion, Vector3
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class RandomNavigator(Node):
    TIMEOUT = 20

    def __init__(self):
        super().__init__("navigator")
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._timer_period = 1.0
        self._timer = self.create_timer(self._timer_period, self.perodic_watcher)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info("Navigation start")
        self.first_call = True
        self.timeout_counter = 0
        self.robot_is_on_target = False
        self.target_num = 0

    def perodic_watcher(self):
        if not self._client.server_is_ready():
            self.get_logger().warning("**** Server is not ready (waiting...)")
            return

        if self.first_call:
            self.first_call = False
            self.get_logger().warning("**** Set the first goal")
            self.send_random_goal()
            return

        if self.timeout_counter >= self.TIMEOUT:
            self.timeout_counter = 0
            self.get_logger().warning("**** Timed out (force to send next goal)")
            self.send_random_goal()
        else:
            self.timeout_counter += 1

    def send_random_goal(self):
        if self.robot_is_on_target:
            # get current position
            trans = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )

            current = trans.transform.rotation
            position = trans.transform.translation
        else:
            current = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            position = Vector3(x=0.0, y=0.0)

        appropriate_distance = False
        while not appropriate_distance:
            goal_x = random.uniform(-2.0, 2.0)
            goal_y = random.uniform(-2.0, 2.0)
            distance = math.sqrt(
                (goal_x - position.x) * (goal_x - position.x)
                + (goal_y - position.y) * (goal_y - position.y)
            )
            if 0.2 < distance < 1.0:
                appropriate_distance = True
            else:
                self.get_logger().info(
                    "re-calcurate next target: distance %f" % distance
                )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation = current

        self.get_logger().info(
            "Set the next position: target num: %d, pos: (%f, %f) distance: %f"
            % (self.target_num, goal_x, goal_y, distance)
        )
        self.robot_is_on_target = False
        send_goal_future = self._client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Rejected")
            # set next target
            self.timeout_counter = self.TIMEOUT
            return

        self.target_num += 1
        self.get_logger().info("Goal accepted by server.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Navigation result: SUCSEEDED")
            self.robot_is_on_target = True
            self.send_random_goal()
        else:
            self.get_logger().info("Navigation result: FAILED(%d)" % result.status)


def main(args=None):
    rclpy.init(args=args)
    node = RandomNavigator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
