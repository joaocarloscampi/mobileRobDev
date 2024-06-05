#include "robovisor/robot_system.hpp"

/**
 * @file robot_main.cpp
 * @brief Main entry point for the robot application.
 * 
 * This file contains the main function that initializes the ROS 2 node, creates the robot system object,
 * and runs the main loop of the robot application.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<ROBOT::System>();
        rclcpp::Rate loop_rate(100);

        while (rclcpp::ok())
        {
            node->loop();
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