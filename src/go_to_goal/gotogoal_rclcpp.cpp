#include <chrono>
#include <memory>
#include <string>
#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <cmath>

using namespace std::chrono_literals;


class TurtleBot : public rclcpp::Node, public std::enable_shared_from_this<TurtleBot>
{
	public:
		TurtleBot() : Node("turtlebot_controller"), rate_(10)
	{
		velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
		pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&TurtleBot::updatePoseCallback, this, std::placeholders::_1));

		pose_ = turtlesim::msg::Pose();
//		rate_ = 100ms;
//		rate_ = 10;
	}

		~TurtleBot() 
		{
			geometry_msgs::msg::Twist vel_msg;
			vel_msg.linear.x = 0.0;
			vel_msg.angular.z = 0.0;
			velocity_publisher_->publish(vel_msg);
		}

		void updatePoseCallback(const turtlesim::msg::Pose::SharedPtr msg) 
		{
			pose_ = *msg;
			pose_.x = round(pose_.x * 10000) / 10000;
			pose_.y = round(pose_.y * 10000) / 10000;
		}

		double euclidean_distance(const turtlesim::msg::Pose &goal_pose)
		{
			return sqrt(pow(goal_pose.x - pose_.x, 2) + pow(goal_pose.y - pose_.y, 2));
		}

		double linear_vel(const turtlesim::msg::Pose &goal_pose, double constant = 1.5)
		{
			return constant * euclidean_distance(goal_pose);
		}

		double steering_angle(const turtlesim::msg::Pose &goal_pose)
		{
			return atan2(goal_pose.y - pose_.y, goal_pose.x - pose_.x);
		}

		double angular_vel(const turtlesim::msg::Pose &goal_pose, double constant = 6.0)
		{
			return constant * (steering_angle(goal_pose) - pose_.theta);
		}

		void move2goal() 
		{
			turtlesim::msg::Pose goal_pose;
			double tolerance;

			std::cout << "Set your x goal: ";
			std::cin >> goal_pose.x;
			std::cout << "Set your y goal: ";
			std::cin >> goal_pose.y;
			std::cout << "Set your distance tolerance: ";
			std::cin >> tolerance;

			geometry_msgs::msg::Twist vel_msg;
			while (euclidean_distance(goal_pose) >= tolerance)
			{
				vel_msg.linear.x = linear_vel(goal_pose);
				vel_msg.linear.y = 0.0;
				vel_msg.linear.z = 0.0;

				vel_msg.angular.x = 0.0;
				vel_msg.angular.y = 0.0;
				vel_msg.angular.z = angular_vel(goal_pose);

				velocity_publisher_->publish(vel_msg);
				RCLCPP_INFO(this->get_logger(), "Moving to goal...");

				rate_.sleep();
			}

			vel_msg.linear.x = 0.0;
			vel_msg.angular.z = 0.0;
			velocity_publisher_->publish(vel_msg);
			RCLCPP_INFO(this->get_logger(), "Goal reached");

			rclcpp::spin(shared_from_this());

		}

	private:
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
		rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
		turtlesim::msg::Pose pose_;
//		std::chrono::milliseconds rate_;
		rclcpp::Rate rate_;
};


int main(int argc, char * argv[]) 
{
	rclcpp::init(argc, argv);

	try {
		auto turtle_bot = std::make_shared<TurtleBot>();
		turtle_bot->move2goal();
	} catch (const std::exception &e) {
		std::cerr << "Exception caught: " << e.what() << std::endl;
		return 1;
	}

	rclcpp::shutdown();
	return 0;
}


