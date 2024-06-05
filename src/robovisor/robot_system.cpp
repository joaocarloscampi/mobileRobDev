#include "robovisor/robot_system.hpp"

namespace ROBOT {

	/**
	 * @brief Constructs a System object.
	 *
	 * This constructor initializes the System object by setting up the Node with the given name,
	 * logging a starting message, initializing time variables, setting the port name and baudrate,
	 * initializing the action variable, creating a TransformBroadcaster, loading topics and settings,
	 * and opening the serial connection.
	 */
	System::System() : Node("robovisor_system") {
		RCLCPP_INFO(rclcpp::get_logger("System"), "Starting Mobile Robot Node");
		current_time = rclcpp::Clock().now();
		last_time = rclcpp::Clock().now();
		port_name = "/dev/ttyACM0";
		baudrate = 115200;
		action = 0;

		odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

		loadTopics();
		loadSettings();

		openSerial();
	}

	/**
	 * @brief Destructor for the System class.
	 * 
	 * This destructor is responsible for closing the serial connection.
	 */
	System::~System () {
		closeSerial();
	}

	/**
	 * @brief Loads the topics for the robot system.
	 * 
	 * This function creates publishers and subscriptions for the robot system topics.
	 * It creates a publisher for the "/odom" topic of type nav_msgs::msg::Odometry,
	 * a publisher for the "/pose2d" topic of type geometry_msgs::msg::Pose2D,
	 * a subscription for the "/cmd_vel" topic of type geometry_msgs::msg::Twist,
	 * and a subscription for the "/message" topic of type std_msgs::msg::String.
	 * 
	 * @return void
	 */
	void System::loadTopics() {
		odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 1);
		pose_pub = this->create_publisher<geometry_msgs::msg::Pose2D>("/pose2d", 1);
		cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
			"/cmd_vel", 1, std::bind(&System::cmdVelCallback, this, std::placeholders::_1));
		msg_sub = this->create_subscription<std_msgs::msg::String>(
			"/message", 1, std::bind(&System::messageCallback, this, std::placeholders::_1));
	}

	/**
	 * @brief Loads the settings for the robot system.
	 * 
	 * This function loads various parameters from the ROS parameter server,
	 * such as the serial port name, baud rate, wheel radius, and robot radius.
	 * It sets the corresponding values in the robot system and logs the loaded
	 * settings using the RCLCPP_INFO macro.
	 */
	void System::loadSettings() {
		RCLCPP_INFO(rclcpp::get_logger("System"), "+----------------------------------+");
		RCLCPP_INFO(rclcpp::get_logger("System"), "|        Loading ROS PARAMS        |");
		RCLCPP_INFO(rclcpp::get_logger("System"), "+----------------------------------+");

		// Loading Serial Port Path
		std::string portName;
		if (this->get_parameter("/robot/portName", portName)) {
			setPortName(portName);
			RCLCPP_INFO(rclcpp::get_logger("System"), " - Serial Port Name = %s", portName.c_str());
		}

		// Loading Serial Baud Rate
		int baudRate;
		if (this->get_parameter("/robot/baudRate", baudRate)) {
			setBaudRate(baudRate);
			RCLCPP_INFO(rclcpp::get_logger("System"), " - Serial Port Baudrate = %d", baudRate);
		}

		// Loading Robot Wheel Radius (r)
		double wheelRadius;
		if (this->get_parameter("/robot/wheelRadius", wheelRadius)) {
			robot.setWheelRadius(wheelRadius);
			RCLCPP_INFO(rclcpp::get_logger("System"), " - Wheel Radius = %f", wheelRadius);
		}

		// Loading Robot Plataform Radius (b)
		double robotRadius;
		if (this->get_parameter("/robot/robotRadius", robotRadius)) {
			robot.setRobotRadius(robotRadius);
			RCLCPP_INFO(rclcpp::get_logger("System"), " - Robot Radius = %f", robot.getRobotRadius());
		}
		RCLCPP_INFO(rclcpp::get_logger("System"), "====================================");
	}

	/**
	 * @brief Opens the serial port and configures its settings.
	 * 
	 * This function opens the serial port specified by `port_name` and configures its settings, 
	 * such as baud rate. If an error occurs during the configuration, an error message is logged.
	 */
	void System::openSerial() {
		serial_port = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
		struct termios tty;

		if(tcgetattr(serial_port, &tty) != 0) {
			RCLCPP_ERROR(rclcpp::get_logger("System"), "Error %i from tcgetattr: %s", errno, strerror(errno));
			return;
		}

		cfsetispeed(&tty, baudrate);
		cfsetospeed(&tty, baudrate);
	}

	/**
	 * @brief Closes the serial port.
	 */
	void System::closeSerial() {
		close(serial_port);

	}

	/**
	 * @brief Gets the serial port number.
	 * 
	 * @return The serial port number.
	 */
	int System::getSerialPort() {
		return serial_port;
	}

	/**
	 * @brief Sets the port name for the system.
	 * 
	 * This function sets the port name for the system. If the provided port path is not empty, 
	 * it sets the port name to the provided path. Otherwise, it sets the port name to the 
	 * default path "/dev/ttyACM0".
	 * 
	 * @param portPathInput The port path input.
	 */
	void System::setPortName(const std::string& portPathInput) {
		std::string DEFAULT_PORT = "/dev/ttyACM0";

		if (portPathInput.compare(" ") != 0) {
			port_name = portPathInput;
		} else {
			port_name = DEFAULT_PORT;
		}
	}

	/**
	 * @brief Sets the baud rate for the serial port.
	 *
	 * This function sets the baud rate for the serial port used by the system. If the provided
	 * baud rate is negative, the default baud rate of 115200 will be used.
	 *
	 * @param baudRateInput The desired baud rate for the serial port.
	 */
	void System::setBaudRate(const int& baudRateInput) {
		int DEFAULT_BAUDRATE = 115200;

		if (baudRateInput < 0) {
			baudrate = DEFAULT_BAUDRATE;
		} else {
			baudrate = baudRateInput;
		}

		RCLCPP_INFO(rclcpp::get_logger("System"), " - Serial Port Baudrate = %d", baudrate);
	}

	/**
	 * @brief Callback function for handling incoming messages.
	 *
	 * This function is called when a new message is received. It checks the content of the message
	 * and performs corresponding actions based on the message content. If the message is "break",
	 * it sets the action variable to 1, sends a formatted message to the serial port, and logs a
	 * "Break" message. If the message is "start", it sets the action variable to 0 and logs a "Start"
	 * message.
	 *
	 * @param message The incoming message.
	 */
	void System::messageCallback(const std_msgs::msg::String::SharedPtr message) {
		char messageBreak[] = "break";
		char messageStart[] = "start";
		char msg[50];
		int n;

		if (strcmp(message->data.c_str(), messageBreak) == 0) {
			action = 1;
			n = sprintf(msg, "$%8.3f,%8.3f,%d#", 0.0, 0.0, action);
			write(serial_port, msg, sizeof(msg));
			RCLCPP_INFO(rclcpp::get_logger("System"), "Break");
		} else if (strcmp(message->data.c_str(), messageStart) == 0) {
			action = 0;
			RCLCPP_INFO(rclcpp::get_logger("System"), "Start");
		}
	}

	/**
	 * @brief Callback function for cmd_vel topic.
	 * 
	 * This function calculates the angular velocities of the robot's wheels based on the linear 
	 * and angular velocities received from the cmd_vel topic. It then converts the angular 
	 * velocities to RPM and sends the calculated values to the serial port.
	 * 
	 * @param cmdVel A shared pointer to the geometry_msgs::msg::Twist message containing the 
	 * linear and angular velocities.
	 */
	void System::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr cmdVel) {
		double dx, dy, dalpha, dthR, dthL, dthR_RPM, dthL_RPM;
		double r, b;

		char msg[50];
		int n;

		r = robot.getWheelRadius();
		b = robot.getRobotRadius();

		dx = cmdVel->linear.x;
		dy = cmdVel->linear.y;
		dalpha = cmdVel->angular.z;

		dthR = (dx + b * dalpha) / r;
		dthL = (dx - b * dalpha) / r;

		dthR_RPM = dthR / (M_PI) * 30;
		dthL_RPM = dthL / (M_PI) * 30;

		n = std::snprintf(msg, sizeof(msg), "$%8.3f,%8.3f,%d#", dthL_RPM, dthR_RPM, action);

		// RCLCPP_INFO(rclcpp::get_logger("System"), "msg: %s", msg);

		write(serial_port, msg, sizeof(msg));
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
	 * @brief Executes the main loop of the robot system.
	 * 
	 * This function is responsible for reading data from the serial port, updating the robot's state,
	 * and publishing the odometry and pose information.
	 */
	void System::loop() {
		current_time = rclcpp::Clock().now();
		dt = (current_time - last_time).seconds();
		robot.setRobotdt(dt);

		num_bytes = read(serial_port, &inChar, sizeof(inChar));

		if (inChar == '#') {
			num_bytes = read(serial_port, &inMessage, sizeof(inMessage));
			tcflush(serial_port, TCIFLUSH);
			if (inMessage[26] == '$') {
				x = std::atof(&inMessage[0]);
				y = std::atof(&inMessage[9]);
				alpha = robot.normalizeAngle(std::atof(&inMessage[18]) * (M_PI / 30), -M_PI);
			} else {
				tcflush(serial_port, TCIFLUSH);
			}
		}
		
		geometry_msgs::msg::Quaternion odom_quat = System::createQuaternionMsgFromYaw(alpha);
		
		geometry_msgs::msg::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		(*odom_broadcaster).sendTransform(odom_trans);

		nav_msgs::msg::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = robot.getRobotdx();
		odom.twist.twist.linear.y = robot.getRobotdy();
		odom.twist.twist.angular.z = robot.getRobotdalpha();

		odom_pub->publish(odom);

		geometry_msgs::msg::Pose2D pose;
		pose.x = x;
		pose.y = y;
		pose.theta = alpha;

		pose_pub->publish(pose);

		last_time = current_time;
	}
	
} // namespace ROBOT