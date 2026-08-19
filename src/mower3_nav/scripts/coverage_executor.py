#!/usr/bin/env python3

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Path


class CoverageExecutor(Node):
    def __init__(self):
        super().__init__("coverage_executor")

        self.follow_path_client = ActionClient(self, FollowPath, "follow_path")
        self.compute_path_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")

        latched_qos = QoSProfile(depth = 1)
        latched_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Path, "/coverage_path", self.on_path, latched_qos)

        self.path = None

    def on_path(self, msg: Path):
        if self.path is not None:
            return

        self.path = msg
        self.get_logger().info(f"received coverage path: {len(msg.poses)} points, starting")
        self.compute_path_to_start()

    def compute_path_to_start(self):
        self.compute_path_client.wait_for_server()
        goal = ComputePathToPose.Goal()
        goal.goal = self.path.poses[0]
        goal.planner_id = "GridBased"
        goal.use_start = False
        self.get_logger().info("computing approach path to coverage path start")
        future = self.compute_path_client.send_goal_async(goal)
        future.add_done_callback(self.on_compute_response)

    def on_compute_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("compute_path_to_pose goal rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_compute_result)

    def on_compute_result(self, future):
        result_wrapper = future.result()
        result = result_wrapper.result
        if (result_wrapper.status != GoalStatus.STATUS_SUCCEEDED
                or result.error_code != 0 or not result.path.poses):
            self.get_logger().error(
                f"failed to compute approach path to coverage start "
                f"(status = {result_wrapper.status}, error_code = {result.error_code})")
            return

        # controller_server's FollowPath action appears to only survive one
        # goal per process lifetime: across every test so far, the first
        # FollowPath goal always succeeds and the second one always segfaults
        # controller_server, regardless of controller_id, path shape/length,
        # or which client sends it. So the approach path and the entire
        # coverage sweep are combined into exactly one FollowPath goal here,
        # instead of one goal per leg/row.
        combined = Path()
        combined.header = self.path.header
        combined.header.stamp = self.get_clock().now().to_msg()
        # self.path.poses[0] is the same point the approach path was targeted
        # at (goal.goal above), so it's already the approach path's last
        # pose - skip it here to avoid a duplicate, zero-length segment at
        # the seam, which breaks the pure-pursuit lookahead search there.
        combined.poses = list(result.path.poses) + list(self.path.poses[1:])
        for pose in combined.poses:
            pose.header = combined.header

        self.get_logger().info(
            f"approach path: {len(result.path.poses)} points; "
            f"following combined path of {len(combined.poses)} points in a single goal"
        )
        self.follow_path(combined)

    def follow_path(self, path: Path):
        self.follow_path_client.wait_for_server()
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "goal_checker"
        goal.progress_checker_id = "progress_checker"
        future = self.follow_path_client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("follow_path goal rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_result(self, future):
        result_wrapper = future.result()
        result = result_wrapper.result
        if result_wrapper.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("coverage path complete")
        else:
            self.get_logger().error(
                f"follow_path stopped before completion (status = {result_wrapper.status}, "
                f"error_code = {result.error_code}, error_msg = '{result.error_msg}'); "
                f"not sending a second goal, since that is what has been crashing "
                f"controller_server"
            )


def main():
    rclpy.init()
    node = CoverageExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
