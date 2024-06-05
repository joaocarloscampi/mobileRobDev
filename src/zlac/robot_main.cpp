#include "zlac/robot_system.hpp"

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	try {
		auto node = std::make_shared<ROBOT::System>();
		rclcpp::Rate loop_rate(100);

		while (rclcpp::ok()) {
			node->odom();
			rclcpp::spin_some(node);
			loop_rate.sleep();
		}
		rclcpp::spin(node);
	}

	catch (const std::exception &e) {
		RCLCPP_FATAL_STREAM(rclcpp::get_logger("robot_main"), "An error has occurred: " << e.what());
		exit(1);
	}

	rclcpp::shutdown();

	return 0;
}