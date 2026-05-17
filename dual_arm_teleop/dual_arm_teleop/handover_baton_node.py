import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class HandoverBaton(Node):
    def __init__(self):
        super().__init__('handover_baton_node')
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("x", 0.5)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.7)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.z = float(self.get_parameter("z").value)

        self.pub = self.create_publisher(Marker, '/handover_baton_marker', 10)
        self.timer = self.create_timer(0.5, self.publish_marker)
        self.get_logger().info(
            f"Publishing handover baton at {self.frame_id}: "
            f"x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}"
        )

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "handover"
        marker.id = 0
        marker.frame_locked = True
        
        # Make it a cylinder (Baton)
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        # Dimensions: 4cm thick, 20cm tall
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.20

        # Color: Bright Neon Green
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Position it between the arms, clearly above the table surface.
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = self.z
        marker.pose.orientation.w = 1.0

        self.pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = HandoverBaton()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
