import rclpy
from rclpy.node import Node

from visualization_msgs.msg import MarkerArray, Marker

from .common import *


def collision_sphere(link, radius):
    collision_sphere = Marker()
    collision_sphere.header.frame_id = link
    collision_sphere.type = Marker.SPHERE
    collision_sphere.action = Marker.ADD
    collision_sphere.color.r = 1.0
    collision_sphere.color.a = 0.5
    collision_sphere.scale.x = 2.0 * radius
    collision_sphere.scale.y = 2.0 * radius
    collision_sphere.scale.z = 2.0 * radius
    return collision_sphere


class VisualizationNode(Node):
    def __init__(self):
        super().__init__("visualization_node")
        self.markers_pub = self.create_publisher(
            MarkerArray, "visualization", qos_profile=1
        )

        # Create marker array
        self.marker_array = MarkerArray()

        for link, radii in enumerate(link_radius):
            self.marker_array.markers.append(
                collision_sphere(f"lbr_link_{link}", radii)
            )
        self.marker_array.markers.append(collision_sphere("lbr_link_ee", ee_radii))

        # Update marker id's
        for index, marker in enumerate(self.marker_array.markers):
            marker.id = index

        hz = 20
        dt = 1.0 / float(hz)
        self.create_timer(dt, self.callback)

    def callback(self):
        self.markers_pub.publish(self.marker_array)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(VisualizationNode())
    rclpy.shutdown()
