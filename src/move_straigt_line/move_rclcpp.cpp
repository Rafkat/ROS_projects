#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

int main(int argc, char * argv[])
{
        rclcpp::init(argc, argv);
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("move_publisher");
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher =
                node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        auto vel_msg = geometry_msgs::msg::Twist();

        std::cout << "Let's move your robot";
        double speed {};
        double distance {};
        bool isForward {false};

        std::cout << "Input your speed: ";
        std::cin >> speed;
        std::cout << "Type your distance: ";
        std::cin >> distance;
        std::cout << "Forward?: ";
        std::cin >> isForward;

        if (isForward) {
                vel_msg.linear.x = std::abs(speed);
        } else {
                vel_msg.linear.x = -std::abs(speed);
        }

        vel_msg.linear.y = 0.0;
        vel_msg.linear.z = 0.0;
        vel_msg.angular.x = 0.0;
        vel_msg.angular.y = 0.0;
        vel_msg.angular.z = 0.0;

        while (rclcpp::ok()) {
                const auto t0 = rclcpp::Clock().now().seconds();
                double current_distance {0.0};

                while (current_distance < distance) {
                        velocity_publisher->publish(vel_msg);
                        const auto t1 = rclcpp::Clock().now().seconds();
                        current_distance = speed * (t1 - t0);
                }
                vel_msg.linear.x = 0;
                velocity_publisher->publish(vel_msg);
        }

        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
}
