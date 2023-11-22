#!/usr/bin/env python
# license removed for brevity
from typing import List
import rclpy
import time
from geometry_msgs.msg import Polygon, Point32
from rclpy.context import Context
from rclpy.parameter import Parameter
from slic3r_coverage_planner.srv import PlanPath
from nav_msgs.msg import Path
from std_msgs.msg import Header
from rclpy.node import Node

# Example of how to call slic3r_coverage_planner PlanPath service.
# It takes a polyon as outline and an arary of polygones for inner holes (isles)
# It returns a list of Slic3r pathes. Each Slic3r path object consists of a nav_msgs/Path object
#
# This program calls the Slic3r path planner for a simple polygon and publishes the first
# path segment as nav_msgs/Path


class Slic3rClient(Node):
    def __init__(self):
        super().__init__("slic3r_client")
        self.slic3r_client = self.create_client(
            PlanPath, "slic3r_coverage_planner/plan_path"
        )
        while not self.slic3r_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("wait for service")

        self.pub = self.create_publisher(Path, "slic3r_path", 10)

    def send_request(self):
        p = Polygon()  # outline
        holes = []
        hole = Polygon()

        hole.points = [
            Point32(x=2.0, y=1.0),
            Point32(x=-2.0, y=1.0),
            Point32(x=-2.0, y=0.0),
            Point32(x=-1.0, y=1.0),
            Point32(x=2.0, y=2.0),
        ]
        holes.append(hole)
        p.points = [
            Point32(x=0.0, y=5.0),
            Point32(x=4.0, y=4.0),
            Point32(x=4.0, y=1.0),
            Point32(x=3.0, y=-2.0),
            Point32(x=1.0, y=-3.0),
            Point32(x=-2.0, y=0.0),
            Point32(x=-3.0, y=-1.3),
            Point32(x=-4.0, y=-4.0),
            Point32(x=-3.0, y=-2.0),
            Point32(x=-2.0, y=-1.0),
            Point32(x=-1.0, y=3.0),
            Point32(x=0.0, y=5.0),
        ]
        path_req = PlanPath.Request()
        path_req.angle = 20.0
        path_req.outline_count = 2
        path_req.distance = 0.2
        path_req.fill_type = 0
        path_req.outline = p
        path_req.holes = holes

        # ['fill_type', 'angle', 'distance', 'outer_offset', 'outline_count', 'outline_overlap count', 'outline', 'holes']
        # resp1 = slic3r_client(0, 20.0, 0.2, 0, 2, 0, p, holes)
        self.future = self.slic3r_client.call_async(path_req)
        rclpy.spin_until_future_complete(self, self.future)

        nav_path = Path()
        header = Header()
        while rclpy.ok():
            # wait for subscribers
            connections = self.pub.get_subscription_count()
            # at least one subscriber? send the generated path
            if connections > 0:
                header.frame_id = "map"
                header.stamp = self.get_clock().now().to_msg()

                nav_path.header = header
                for path in self.future.result().paths:
                    nav_path.poses = path.path.poses
                    self.pub.publish(nav_path)
                    time.sleep(5)
                   # rclpy.spin()  # infinite loop ensures to send only once
            


def main():
    rclpy.init()

    client = Slic3rClient()
    client.send_request()

    client.destroy_node()
    rclpy.shutdown


if __name__ == "__main__":
    main()
