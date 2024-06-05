#include "zlac/robot_system.hpp"

namespace ROBOT {

	/**
	 * @brief Constructs a System object.
	 *
	 * This constructor initializes the System object by setting up the Node with the given name,
	 * logging a starting message, initializing time variables, setting the port name and baudrate,
	 * initializing the action variable, creating a TransformBroadcaster, loading topics and settings,
	 * and opening the serial connection.
	 */
	System::System() : Node("zlac") {
		RCLCPP_INFO(rclcpp::get_logger("zlac"), "Starting Mobile Robot Node");
		current_time = rclcpp::Clock().now();
		last_time = rclcpp::Clock().now();
		path.header.frame_id = "odom";

		RCLCPP_INFO(rclcpp::get_logger("zlac"), "Starting publishers and subscribers");

		// tf broadcasters
		odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

		// find out if smoothed_cmd_vel is available
		std::string cmd_vel_topic = "/cmd_vel";
		auto nodes = this->get_node_names();
		if (std::find(nodes.begin(), nodes.end(), "/velocity_smoother") != nodes.end()) {
			cmd_vel_topic = "/smoothed_cmd_vel";
		}
		RCLCPP_INFO(rclcpp::get_logger("zlac"), "Using %s topic for cmd_vel", cmd_vel_topic.c_str());

		// publishers
		odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 1);
		cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
			cmd_vel_topic, 1, std::bind(&System::cmd_vel_callback, this, std::placeholders::_1));
		path_pub = this->create_publisher<nav_msgs::msg::Path>("/odom_path", 1);

		// servers
		start_dump_srv = this->create_service<std_srvs::srv::Empty>(
			"start_dump", 
			std::bind(&System::start_dump, this, std::placeholders::_1, std::placeholders::_2));
		stop_dump_srv = this->create_service<std_srvs::srv::Empty>(
			"stop_dump", 
			std::bind(&System::stop_dump, this, std::placeholders::_1, std::placeholders::_2));

		// timers
		path_timer = this->create_wall_timer(
			std::chrono::milliseconds(1000),
			std::bind(&System::path_timer_callback, this));
		fault_timer = this->create_wall_timer(
			std::chrono::milliseconds(5000),
			std::bind(&System::fault_timer_callback, this));
		
		RCLCPP_INFO(rclcpp::get_logger("zlac"), "Setting initial parameters");

		robot.motors.set_accel_time(200, 200);
		robot.motors.set_decel_time(200, 200);
		robot.motors.set_mode(3);
		robot.motors.enable_motor();
		if (robot.motors.get_main_voltage() < 20) {
			RCLCPP_ERROR(rclcpp::get_logger("zlac"), "Main voltage is too low, shutting down");
			robot.motors.disable_motor();
			rclcpp::shutdown();
		}

		RCLCPP_INFO(rclcpp::get_logger("zlac"), "Success starting ZLAC node");
	}

	/**
	 * @brief Destructor for the System class.
	 * 
	 * This destructor is responsible for closing the serial connection.
	 */
	System::~System () {
		robot.motors.disable_motor();
		RCLCPP_INFO(rclcpp::get_logger("zlac"), "Closing Mobile Robot Node");
	}

	/**
	 * @brief Callback function for cmd_vel topic.
	 * 
	 * This function calculates the angular velocities of the robot's wheels based on the linear 
	 * and angular velocities received from the cmd_vel topic. It then converts the angular 
	 * velocities to RPM and sends the calculated values to the serial port.
	 * 
	 * @param cmd_vel A shared pointer to the geometry_msgs::msg::Twist message containing the 
	 * linear and angular velocities.
	 */
	void System::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel) {
		double vx = cmd_vel->linear.x;
		double wz = cmd_vel->angular.z;

		double vl = vx - wz*robot.b;
		double vr = vx + wz*robot.b;

		rpm_l_ref = (-vl * 60)/(2 * M_PI * robot.r);
		rpm_r_ref = (vr * 60)/(2 * M_PI * robot.r);

		robot.motors.set_rpm(rpm_l_ref, rpm_r_ref);
	}

	/**
	 * @brief Creates a geometry_msgs::msg::Quaternion message from a given yaw angle.
	 * 
	 * @param yaw The yaw angle in radians.
	 * @return The geometry_msgs::msg::Quaternion message representing the yaw angle.
	 */
	geometry_msgs::msg::Quaternion System::createQuaternionMsgFromYaw(double yaw) {
		tf2::Quaternion q;
		q.setRPY(0.0, 0.0, yaw);
		return tf2::toMsg(q);
	}

	/**
	 * @brief Publishes the odometry information.
	 * 
	 * This function calculates the odometry values for the robot and publishes the odometry
	 * transform and message to the corresponding topics.
	 */
	void System::odom() {
		// get rpm values, apply two complement and divide by 10
		float l_rpm = robot.motors.get_l_rpm();
		float r_rpm = robot.motors.get_r_rpm();
		l_rpm = l_rpm > 32767 ? l_rpm - 65536 : l_rpm;
		r_rpm = r_rpm > 32767 ? r_rpm - 65536 : r_rpm;
		l_rpm /= 10;
		r_rpm /= 10;

		// cap values to 1000
		if (l_rpm > 1000 || l_rpm < -1000) l_rpm = rpm_l_prev;
		if (r_rpm > 1000 || l_rpm < -1000) r_rpm = rpm_r_prev;

		// apply low pas filter
		// l_rpm = robot.low_pass_filter(0.9, l_rpm, rpm_l_prev);
		// r_rpm = robot.low_pass_filter(0.9, r_rpm, rpm_r_prev);

		// set values
		rpm_l_prev = l_rpm;
		rpm_r_prev = r_rpm;

		double v_l = -l_rpm * 2 * M_PI * robot.r / 60;
		double v_r = r_rpm * 2 * M_PI * robot.r / 60;

		double v = (v_l + v_r) / 2.0;
		double w = (v_r - v_l) / (robot.b * 2.0);

		current_time = rclcpp::Clock().now();
		dt = (current_time - last_time).seconds();

		robot.dx = v*cos(robot.theta);
		robot.dy = v*sin(robot.theta);
		robot.dtheta = w;

		robot.theta += robot.dtheta * dt;
		robot.x += robot.dx * dt;
		robot.y += robot.dy * dt;

		if (dump_state == dump_state::DUMPING) {
			elapsed_time += dt;
			fprintf(dump_file, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
				elapsed_time,
				robot.x,
				robot.y,
				robot.theta,
				robot.dx,
				robot.dy,
				robot.dtheta,
				dt,
				l_rpm,
				r_rpm,
				rpm_l_ref,
				rpm_r_ref);
		}

		odom_quat = System::createQuaternionMsgFromYaw(robot.theta);
		
		geometry_msgs::msg::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";

		odom_trans.transform.translation.x = robot.x;
		odom_trans.transform.translation.y = robot.y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		(*odom_broadcaster).sendTransform(odom_trans);

		nav_msgs::msg::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		odom.pose.pose.position.x = robot.x;
		odom.pose.pose.position.y = robot.y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = robot.dx;
		odom.twist.twist.linear.y = robot.dy;
		odom.twist.twist.angular.z = robot.dtheta;

		odom_pub->publish(odom);

		last_time = current_time;
	}

	void System::start_dump(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response) {
		if (dump_state == dump_state::DUMPING) {
			RCLCPP_INFO(rclcpp::get_logger("zlac"), "Node is already dumping");
		} else {
			elapsed_time = 0;
			std::time_t now = std::time(nullptr);
			std::tm* timeinfo = std::localtime(&now);
			char buffer[80];
			std::strftime(buffer, sizeof(buffer), "%Y_%m_%d_%H_%M_%S.csv", timeinfo);
			std::string filename(buffer);
			RCLCPP_INFO(rclcpp::get_logger("zlac"), "Starting dump to file: %s", filename.c_str());
			dump_state = dump_state::DUMPING;
			
			dump_file = fopen(filename.c_str(), "w");
			fprintf(dump_file, "time,x,y,theta,dx,dy,dtheta,dt,l_rpm,r_rpm,rpm_l_ref,rpm_r_ref\n");
		}
	}

	void System::stop_dump(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response) {
		if (dump_state == dump_state::DUMPING) {
			RCLCPP_INFO(rclcpp::get_logger("zlac"), "Stopping dump");
			dump_state = dump_state::NOT_DUMPING;
			fclose(dump_file);
		} else {
			RCLCPP_INFO(rclcpp::get_logger("zlac"), "Node is not dumping");
		}
	}

	void System::fault_timer_callback() {
		int fault_l = robot.motors.get_l_fault();
		int fault_r = robot.motors.get_r_fault();

		switch (fault_l) {
			case 0x0000:
				break;
			case 0x0001:
				RCLCPP_INFO(rclcpp::get_logger("zlac"), "Left wheel has over voltage fault");
				break;
			case 0x0002:
				RCLCPP_INFO(rclcpp::get_logger("zlac"), "Left wheel has under voltage fault");
				break;
			case 0x0004:
				RCLCPP_INFO(rclcpp::get_logger("zlac"), "Left wheel has over current fault");
				break;
			case 0x0008:
				RCLCPP_INFO(rclcpp::get_logger("zlac"), "Left wheel has over load fault");
				break;
			case 0x0010:
				RCLCPP_INFO(rclcpp::get_logger("zlac"), "Left wheel has current output tolerance fault");
				break;
			case 0x0020:
				RCLCPP_INFO(rclcpp::get_logger("zlac"), "Left wheel has encoder output tolerance fault");
				break;
			case 0x0040:
				RCLCPP_INFO(rclcpp::get_logger("zlac"), "Left wheel has motor bad fault");
				break;
			case 0x0080:
				RCLCPP_INFO(rclcpp::get_logger("zlac"), "Left wheel has reference voltage error fault");
				break;
			case 0x0100:
				RCLCPP_INFO(rclcpp::get_logger("zlac"), "Left wheel has EEPROM error fault");
				break;
			case 0x0200:
				RCLCPP_INFO(rclcpp::get_logger("zlac"), "Left wheel has wall error fault");
				break;
			case 0x0400:
				RCLCPP_INFO(rclcpp::get_logger("zlac"), "Left wheel has high temperature fault");
				break;
			default:
				break;
		}

		switch (fault_r) {
			case 0x0000:
				break;
			case 0x0001:
				RCLCPP_INFO(rclcpp::get_logger("zlac"), "Right wheel has over voltage fault");
				break;
			case 0x0002:
				RCLCPP_INFO(rclcpp::get_logger("zlac"), "Right wheel has under voltage fault");
				break;
			case 0x0004:
				RCLCPP_INFO(rclcpp::get_logger("zlac"), "Right wheel has over current fault");
				break;
			case 0x0008:
				RCLCPP_INFO(rclcpp::get_logger("zlac"), "Right wheel has over load fault");
				break;
			case 0x0010:
				RCLCPP_INFO(rclcpp::get_logger("zlac"), "Right wheel has current output tolerance fault");
				break;
			case 0x0020:
				RCLCPP_INFO(rclcpp::get_logger("zlac"), "Right wheel has encoder output tolerance fault");
				break;
			case 0x0040:
				RCLCPP_INFO(rclcpp::get_logger("zlac"), "Right wheel has motor bad fault");
				break;
			case 0x0080:
				RCLCPP_INFO(rclcpp::get_logger("zlac"), "Right wheel has reference voltage error fault");
				break;
			case 0x0100:
				RCLCPP_INFO(rclcpp::get_logger("zlac"), "Right wheel has EEPROM error fault");
				break;
			case 0x0200:
				RCLCPP_INFO(rclcpp::get_logger("zlac"), "Right wheel has wall error fault");
				break;
			case 0x0400:
				RCLCPP_INFO(rclcpp::get_logger("zlac"), "Right wheel has high temperature fault");
				break;
			default:
				break;
		}
	}

	void System::path_timer_callback() {
		geometry_msgs::msg::PoseStamped pose;
		pose.header.stamp = current_time;
		pose.header.frame_id = "odom";
		pose.pose.position.x = robot.x;
		pose.pose.position.y = robot.y;
		pose.pose.position.z = 0.0;
		pose.pose.orientation = odom_quat;

		path.header.stamp = current_time;
		path.poses.push_back(pose);

		path_pub->publish(path);
	}

} // namespace ROBOT