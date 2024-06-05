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

#include "zlac/robot.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.h"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/msg/path.hpp"

#include <std_srvs/srv/empty.hpp>

using namespace std;


namespace ROBOT {

	class System : public rclcpp::Node {
	private:
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;

		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
		nav_msgs::msg::Path path;

		rclcpp::TimerBase::SharedPtr fault_timer;
		rclcpp::TimerBase::SharedPtr path_timer;
		
		rclcpp::Time current_time;
		rclcpp::Time last_time;
		double dt;

		std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
		geometry_msgs::msg::Quaternion odom_quat;

		float rpm_l_ref;
		float rpm_r_ref;
		float rpm_l_prev;
		float rpm_r_prev;

		double elapsed_time = 0;
		enum dump_state {NOT_DUMPING, DUMPING} dump_state = dump_state::NOT_DUMPING;
		FILE *dump_file;
		rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_dump_srv;
		rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_dump_srv;

	public:
		System();
		~System ();
		Robot robot;
		void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmdVel);
		void odom();
		geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw);

		// services
		void start_dump(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response);
		void stop_dump(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response);

		// timers
		void path_timer_callback();
		void fault_timer_callback();
	};

} // namespace ROBOT


#endif /* MOBILE_ROB_DEV_INCLUDE_ROBOTSYSTEM_H_ */
