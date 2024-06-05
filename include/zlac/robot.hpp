#ifndef ROBOT_H_
#define ROBOT_H_

#include "rclcpp/rclcpp.hpp"
#include "definitions.hpp"
#include "zlac/zlac.hpp"

using namespace ROBOT::Types;

namespace ROBOT {
	
	class Robot {
		public:
			double r;
			double b;

			double x;
			double y;
			double theta;

			double dx;
			double dy;
			double dtheta;
			double x_previous;
			double y_previous;
			double theta_previous;

			Controller motors;
			Robot();
			double normalize_angle(double ang, double low);
			double low_pass_filter(double alpha, double x, double xpf);
	};
}

#endif /* ROBOT_H_ */