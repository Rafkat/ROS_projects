#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
double PI = 3.1415926535897;

int main(int argc, char * argv [])
{
	rclcpp::init(argc, argv);
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("rotate_publisher");
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

	auto vel_msg = geometry_msgs::msg::Twist();

	std::cout << "Let's rotate your robot\n";
	double speed {};
	double angle {};
	bool clockwise {};
	
	std::cout << "Input your speed (degrees/sec): ";
	std::cin >> speed;
	std::cout << "Input your distance (degrees): ";
	std::cin >> angle;
	std::cout << "Clockwise?: ";
	std::cin >> clockwise;

	double angular_speed = speed * 2.0 * PI / 360.0;
	double relative_angle = angle * 2.0 * PI / 360.0;
	
	vel_msg.linear.x = 0.0;
	vel_msg.linear.y = 0.0;
	vel_msg.linear.z = 0.0;

	vel_msg.angular.x = 0.0;
	vel_msg.angular.y = 0.0;
	
	if (clockwise) {
		vel_msg.angular.z = std::abs(angular_speed);
	} else {
		vel_msg.angular.z = -std::abs(angular_speed);
	}

	auto t0 = rclcpp::Clock().now().seconds();
	double current_angle = 0.0;

	while (current_angle < relative_angle) {
		velocity_publisher->publish(vel_msg);
		auto t1 = rclcpp::Clock().now().seconds();
		current_angle = angular_speed * (t1 - t0);
	}
	vel_msg.angular.z = 0.0;
	velocity_publisher->publish(vel_msg);

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
