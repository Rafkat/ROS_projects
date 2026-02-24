import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
PI = 3.1415926535897

def rotate():
    rclpy.init()
    node = Node("robot_cleaner")
    velocity_publisher = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    vel_msg = Twist()

    print("Let's rotate your robot")
    speed = float(input("Input your speed (degrees/sec):"))
    distance = float(input("Input your distance (degrees):"))
    clockwise = bool(input("Clockwise?: "))

    angular_speed = speed * 2.0 * PI / 360.0
    relative_angle = distance * 2.0 * PI / 360.0

    vel_msg.linear.x = 0.0
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0

    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0

    if clockwise:
        vel_msg.angular.z = abs(angular_speed)
    else:
        vel_msg.angular.z = -abs(angular_speed)

    t0 = node.get_clock().now().nanoseconds / 1e9
    current_angle = 0.0

    while current_angle < relative_angle:
        velocity_publisher.publish(vel_msg)
        t1 = node.get_clock().now().nanoseconds / 1e9
        current_angle = angular_speed * (t1 - t0)

    vel_msg.angular.z = 0.0
    velocity_publisher.publish(vel_msg)

    rclpy.spin(node)


if __name__ == "__main__":
    try:
        rotate()
    except KeyboardInterrupt:
        pass

