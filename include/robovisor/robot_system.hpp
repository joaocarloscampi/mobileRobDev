#ifndef ROBOTSYSTEM_H_
#define ROBOTSYSTEM_H_

#include <iostream>
#include <fstream>
#include <cstdlib>

#include <cstdio>
#include <cstring>

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include "robovisor/robot.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std;


namespace ROBOT {

	class System : public rclcpp::Node {

		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
		rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_pub;
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr msg_sub;
		
		rclcpp::Time current_time;
		rclcpp::Time last_time;

		std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
		std::string port_name;
		int serial_port;
		unsigned int baudrate;
		int num_bytes;
		double x, y, alpha, dt;
		char inChar;
		std::array<char, 27> inMessage;

		unsigned int action;

	private:

	public:
		System();
		~System ();
		Robot robot;
		void loadTopics();
		void loadSettings();
		void openSerial();
		void closeSerial();
		int getSerialPort();
		void setPortName(const std::string& portPathInput);
		void setBaudRate(const int& baudRateInput);
		void messageCallback(const std_msgs::msg::String::SharedPtr message);
		void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr cmdVel);
		void loop();
		geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw);
	};

} // namespace ROBOT


#endif /* MOBILE_ROB_DEV_INCLUDE_ROBOTSYSTEM_H_ */
