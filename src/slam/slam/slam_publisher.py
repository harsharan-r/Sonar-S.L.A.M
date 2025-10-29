import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import PointCloud, Imu
from geometry_msgs.msg import Point32


class SlamPublisher(Node):

    def __init__(self):
        super().__init__('slam_publisher')

        self.sonar_subscription_ = self.create_subscription(
            PointCloud,
            '/occupancy_data',
            self.sonar_callback,
            10)
        
        self.imu_subscription_ = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10)

    def sonar_callback(self, msg):
        # point_data = [[point.x, point.y, point.z] for point in msg.points]
        # self.get_logger().info(f"points: {point_data}")
        self.get_logger().info("\n--- Point Cloud Message Recieved ---\n")

    def imu_callback(self, msg):
        # self.get_logger().info(
        #     f"\n--- IMU Message ---\n"
        #     f"Orientation (quat): "
        #     f"x={msg.orientation.x:.3f}, "
        #     f"y={msg.orientation.y:.3f}, "
        #     f"z={msg.orientation.z:.3f}, "
        #     f"w={msg.orientation.w:.3f}\n"
        #     f"Angular Vel: "
        #     f"x={msg.angular_velocity.x:.3f}, "
        #     f"y={msg.angular_velocity.y:.3f}, "
        #     f"z={msg.angular_velocity.z:.3f}\n"
        #     f"Linear Accel: "
        #     f"x={msg.linear_acceleration.x:.3f}, "
        #     f"y={msg.linear_acceleration.y:.3f}, "
        #     f"z={msg.linear_acceleration.z:.3f}"
        # )
        self.get_logger().info("\n--- IMU Message Recieved ---\n")

def main(args=None):
    rclpy.init(args=args)

    slam_publisher = SlamPublisher()

    rclpy.spin(slam_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    slam_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()