import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_cleaner')
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        #self.get_logger().info("Let's move your robot")

    
def move():
    rclpy.init()
    node = RobotMover()

    print("Let's move your robot")
    speed = float(input("Input your speed:"))
    distance = float(input("Type your distance:"))
    isForward = bool(input("Forward?: "))

    vel_msg = Twist()

    if isForward:
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)

    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 0.0

    while rclpy.ok():
        t0 = node.get_clock().now().nanoseconds / 1e9
        current_distance = 0.0

        while current_distance < distance:
            node.velocity_publisher.publish(vel_msg)
            t1 = node.get_clock().now().nanoseconds / 1e9
            current_distance = speed * (t1 - t0)
            #rclpy.spin_once(node)
        vel_msg.linear.x = 0.0
        node.velocity_publisher.publish(vel_msg)

    rclpy.spin(node)


if __name__ == '__main__':
    try:
        move()
    except KeyboardInterrupt:
        pass


