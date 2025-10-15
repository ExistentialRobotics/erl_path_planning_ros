#!/usr/bin/env python3

import os

import cv2
import numpy as np
import rclpy.node
import rclpy.qos
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Float64, Float64MultiArray, Int64, MultiArrayDimension, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

from erl_geometry_msgs.msg import GridMapMsg


class TestAstar2DNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("test_astar_2d_ltl_node")

        self.declare_parameter("astar_node_name", "astar_2d_ltl_node")
        self.declare_parameter("automaton_path", "")
        self.declare_parameter("label_map_image_path", "")
        self.astar_node_name = self.get_parameter("astar_node_name").get_parameter_value().string_value
        self.automaton_path = self.get_parameter("automaton_path").get_parameter_value().string_value
        self.label_map_path = self.get_parameter("label_map_image_path").get_parameter_value().string_value
        if self.automaton_path == "":
            self.get_logger().error("automaton_path parameter is not set!")
            return
        if self.label_map_path == "":
            self.get_logger().error("label_map_image_path parameter is not set!")
            return
        assert os.path.exists(self.automaton_path), f"Automaton file does not exist: {self.automaton_path}"
        assert os.path.exists(self.label_map_path), f"Label map file does not exist: {self.label_map_path}"
        self.get_logger().info(f"Using automaton file: {self.automaton_path}")
        self.get_logger().info(f"Using label map image file: {self.label_map_path}")

        qos = rclpy.qos.QoSProfile(depth=1)
        qos.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE
        qos.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL

        start = Float64MultiArray()
        start.layout.dim.append(MultiArrayDimension(size=3, stride=1))
        start.data = [-2, 3, 0]  # the FSA state will be set as the automaton initial state
        self.start_pub = self.create_publisher(Float64MultiArray, "start", qos)
        self.start_pub.publish(start)
        self.get_logger().info("Published test start point")

        start_marker = Marker()
        start_marker.header.frame_id = "map"
        start_marker.header.stamp = self.get_clock().now().to_msg()
        start_marker.ns = "test_astar_2d_ltl_node"
        start_marker.id = 0
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = start.data[0]
        start_marker.pose.position.y = start.data[1]
        start_marker.pose.position.z = 0.0
        start_marker.pose.orientation.x = 0.0
        start_marker.pose.orientation.y = 0.0
        start_marker.pose.orientation.z = 0.0
        start_marker.pose.orientation.w = 1.0
        start_marker.scale.x = 0.2
        start_marker.scale.y = 0.2
        start_marker.scale.z = 0.2
        start_marker.color.a = 1.0
        start_marker.color.r = 0.0
        start_marker.color.g = 1.0
        start_marker.color.b = 0.0
        self.start_marker_pub = self.create_publisher(Marker, "start_marker", qos)
        self.start_marker_pub.publish(start_marker)
        self.get_logger().info("Published test start marker")

        goal = Float64MultiArray()
        goal.layout.dim.append(MultiArrayDimension(size=3, stride=1))
        goal.data = [0, 0, 0]  # don't care about the xy position, only care about the FSA state
        self.goal_pub = self.create_publisher(Float64MultiArray, "goals", qos)
        self.goal_pub.publish(goal)
        self.get_logger().info("Published test goal point")

        # goal_marker = Marker()
        # goal_marker.header.frame_id = "map"
        # goal_marker.header.stamp = self.get_clock().now().to_msg()
        # goal_marker.ns = "test_astar_2d_ltl_node"
        # goal_marker.id = 0
        # goal_marker.type = Marker.SPHERE
        # goal_marker.action = Marker.ADD
        # goal_marker.pose.position.x = goal.data[0]
        # goal_marker.pose.position.y = goal.data[1]
        # goal_marker.pose.position.z = 0.0
        # goal_marker.pose.orientation.x = 0.0
        # goal_marker.pose.orientation.y = 0.0
        # goal_marker.pose.orientation.z = 0.0
        # goal_marker.pose.orientation.w = 1.0
        # goal_marker.scale.x = 0.2
        # goal_marker.scale.y = 0.2
        # goal_marker.scale.z = 0.2
        # goal_marker.color.a = 1.0
        # goal_marker.color.r = 1.0
        # goal_marker.color.g = 0.0
        # goal_marker.color.b = 0.0
        # self.goal_marker_pub = self.create_publisher(Marker, "goal_marker", qos)
        # self.goal_marker_pub.publish(goal_marker)
        # self.get_logger().info("Published test goal marker")

        goal_tolerance = Float64MultiArray()
        goal_tolerance.layout.dim.append(MultiArrayDimension(size=3, stride=1))
        goal_tolerance.data = [float("inf"), float("inf"), 0]
        self.goal_tolerance_pub = self.create_publisher(Float64MultiArray, "goal_tolerances", qos)
        self.goal_tolerance_pub.publish(goal_tolerance)
        self.get_logger().info("Published test goal tolerance")

        terminal_cost = Float64MultiArray()
        terminal_cost.layout.dim.append(MultiArrayDimension(size=1, stride=1))
        terminal_cost.data = [0.0]
        self.terminal_cost_pub = self.create_publisher(Float64MultiArray, "terminal_costs", qos)
        self.terminal_cost_pub.publish(terminal_cost)
        self.get_logger().info("Published test terminal cost")

        # free everywhere
        occ_map = OccupancyGrid()
        occ_map.header.frame_id = "map"
        occ_map.header.stamp = self.get_clock().now().to_msg()
        occ_map.info.resolution = 0.1
        occ_map.info.height = 261  # y
        occ_map.info.width = 251  # x
        occ_map.info.origin.position.x = -5.05
        occ_map.info.origin.position.y = -5.05
        occ_map.info.origin.position.z = 0.0
        occ_map.info.origin.orientation.x = 0.0
        occ_map.info.origin.orientation.y = 0.0
        occ_map.info.origin.orientation.z = 0.0
        occ_map.info.origin.orientation.w = 1.0
        occ_map.data = [0] * (occ_map.info.height * occ_map.info.width)
        self.occ_map_pub = self.create_publisher(OccupancyGrid, "occ_map", qos)
        self.occ_map_pub.publish(occ_map)
        self.get_logger().info("Published test occupancy grid map")

        # load the label map
        label_map = GridMapMsg()
        label_map.header.frame_id = "map"
        label_map.header.stamp = self.get_clock().now().to_msg()
        label_map.info.height = 261  # y
        label_map.info.width = 251  # x
        label_map.info.resolution = 0.1
        label_map.info.origin.position.x = occ_map.info.origin.position.x
        label_map.info.origin.position.y = occ_map.info.origin.position.y
        label_map.info.origin.position.z = 0.0
        label_map.info.origin.orientation.x = 0.0
        label_map.info.origin.orientation.y = 0.0
        label_map.info.origin.orientation.z = 0.0
        label_map.info.origin.orientation.w = 1.0
        label_map.encoding = 6  # UINT32
        label_map_data: np.ndarray = cv2.imread(self.label_map_path, cv2.IMREAD_UNCHANGED).T.astype(np.uint32)
        self.get_logger().info(f"Loaded label map with shape {label_map_data.shape} and dtype {label_map_data.dtype}")
        label_map.data = label_map_data.tobytes()
        self.label_map_pub = self.create_publisher(GridMapMsg, "label_map", qos)
        self.label_map_pub.publish(label_map)
        self.get_logger().info("Published test label map")

        # load automaton
        automaton_msg = String()
        with open(self.automaton_path, "r") as f:
            automaton_msg.data = f.read()

        self.automaton_pub = self.create_publisher(String, "aut_spot", qos)
        self.automaton_pub.publish(automaton_msg)
        self.get_logger().info("Published test automaton")

        self.ap_dict_sub = self.create_subscription(String, "ap_dict", self.ap_dict_callback, qos)
        self.path_sub = self.create_subscription(Path, "path", self.path_callback, qos)
        self.cost_sub = self.create_subscription(Float64, "cost", self.cost_callback, qos)
        self.goal_idx_sub = self.create_subscription(Int64, "goal_index", self.goal_idx_callback, qos)

        self.trigger_client = self.create_client(Trigger, f"plan_path")
        while not self.trigger_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for plan_path service...")

        self.get_logger().info("TestAstar2dLtlNode initialized")

        self.future = None
        # send a trigger to plan after a short delay to ensure all topics are set up
        self.timer = self.create_timer(2.0, self.trig_plan)

    def trig_plan(self):
        trigger = Trigger.Request()
        self.future = self.trigger_client.call_async(trigger)
        self.future.add_done_callback(self.get_plan_callback)
        self.timer.cancel()  # only trigger once

    def get_plan_callback(self, future: rclpy.Future):
        try:
            response: Trigger.Response = future.result()
            if response.success:
                self.get_logger().info(f"Service call succeeded: {response.message}")
            else:
                self.get_logger().error(f"Service call failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def ap_dict_callback(self, msg: String):
        self.get_logger().info(f"Received AP dictionary: {msg.data}")

    def path_callback(self, msg: Path):
        self.get_logger().info(f"Received path with {len(msg.poses)} poses")
        for i, pose in enumerate(msg.poses):
            self.get_logger().info(
                f"  Pose {i}: ({pose.pose.position.x}, {pose.pose.position.y})"
            )

    def cost_callback(self, msg: Float64):
        self.get_logger().info(f"Received path cost: {msg.data}")
        if abs(msg.data - 20.417808532714844) > 1e-5:
            self.get_logger().error("Path cost is incorrect!")
        else:
            self.get_logger().info("Path cost is correct!")

    def goal_idx_callback(self, msg: Int64):
        self.get_logger().info(f"Received goal index: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = TestAstar2DNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
