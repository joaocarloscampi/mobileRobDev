#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/empty.hpp"

class OdomDump : public rclcpp::Node
{
public:
	OdomDump(int argc, char **argv) : Node("odom_dump") {
		start_dump_client = this->create_client<std_srvs::srv::Empty>("start_dump");
		stop_dump_client = this->create_client<std_srvs::srv::Empty>("stop_dump");
		auto request = std::make_shared<std_srvs::srv::Empty::Request>();
		if (!start_dump_client->wait_for_service(std::chrono::seconds(1))) {
			RCLCPP_ERROR(this->get_logger(), "/start_dump service not available. zlac not running?");
			exit(1);
		}
		auto result = start_dump_client->async_send_request(request);

		validate_args(argc, argv);
		amplitude = amplitude * 0.069 * M_PI / 30;

		cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
	}

	~OdomDump() {
		auto request = std::make_shared<std_srvs::srv::Empty::Request>();
		auto result = stop_dump_client->async_send_request(request);

		geometry_msgs::msg::Twist stop_msg;
		stop_msg.linear.x = 0;
		stop_msg.angular.z = 0;
		cmd_vel_pub->publish(stop_msg);

		result.wait_for(std::chrono::seconds(2));
	}

	void loop() {
		geometry_msgs::msg::Twist msg;
		msg.angular.z = 0;
		t = this->now().seconds() - start_time;
		if (function == "step") {
			msg.linear.x = amplitude;
		}
		else if (function == "sine") {
			msg.linear.x = amplitude * sin(M_PI * frequency * t);
		}
		else if (function == "triangle") {
			msg.linear.x = 4 * amplitude * frequency * abs(fmod(t - 4 * frequency, 4) - 1 / (2 *frequency)) - amplitude;
		}
		cmd_vel_pub->publish(msg);
	}

private:
	void validate_args(int argc, char **argv) {
		if (argc < 3) {
			RCLCPP_ERROR(this->get_logger(), "Usage: odom_dump <function> <parameters>");
			exit(1);
		}

		function = argv[1];
		if (function != "step" && function != "sine" && function != "triangle") {
			RCLCPP_ERROR(this->get_logger(), "Invalid function argument. Function must be step, sine, or triangle.");
			exit(1);
		}

		if (function == "step") {
			if (argc != 3) {
				RCLCPP_ERROR(this->get_logger(), "Usage: odom_dump step <amplitude>");
				exit(1);
			}
			RCLCPP_INFO(this->get_logger(), "Step function with amplitude %f", std::stod(argv[2]));
			amplitude = std::stod(argv[2]);
		}

		if (function == "sine" || function == "triangle") {
			if (argc != 4) {
				RCLCPP_ERROR(this->get_logger(), "Usage: odom_dump <sine/triangle> <amplitude> <frequency>");
				exit(1);
			}
			RCLCPP_INFO(this->get_logger(), "%s function with amplitude %f and frequency %f", function.c_str(), std::stod(argv[2]), std::stod(argv[3]));
			amplitude = std::stod(argv[2]);
			frequency = std::stod(argv[3]);
		}
	}

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
	rclcpp::Client<std_srvs::srv::Empty>::SharedPtr start_dump_client;
	rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_dump_client;
	std::string function;
	double amplitude;
	double frequency;
	double start_time = this->now().seconds();
	double t;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	try {
		auto node = std::make_shared<OdomDump>(argc, argv);
		rclcpp::Rate loop_rate(100);
		rclcpp::sleep_for(std::chrono::seconds(1));

		while (rclcpp::ok()) {
			node->loop();
			rclcpp::spin_some(node);
			loop_rate.sleep();
		}
	}
	catch (const std::exception &e) {
		RCLCPP_FATAL_STREAM(rclcpp::get_logger("robot_main"), "An error has occurred: " << e.what());
		exit(1);
	}


	rclcpp::shutdown();
	return 0;
}