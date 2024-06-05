#ifndef ROBOT_H_
#define ROBOT_H_

#include "rclcpp/rclcpp.hpp"
#include "definitions.hpp"

using namespace ROBOT::Types;

namespace ROBOT {
	
	class Robot {
		public:
				double r;
				double b;
				double x;
				double y;
				double alpha;

				double dt;

				double xo;
				double yo;
				double alphao;

				double x_offset;
				double y_offset;
				double alpha_offset;

				double dx;
				double dy;
				double dalpha;
				double dthR;
				double dthL;
				double distFront;
				double distBack;
				double dist_L;	
				double dist_R;
				double distC;
				double x_previous;
				double y_previous;
				double alpha_previous;

			Robot();
			void setWheelRadius(double& wheelRaidusInput);
			void setRobotRadius(double& robotRadiusInput);
			double getWheelRadius();
			double getRobotRadius();
			double getRobotdx();
			double getRobotdy();
			double getRobotdalpha();
			double normalizeAngle(double ang, double low);
			double lowPassFilter(double alpha, double x, double xpf);		
			void setRobotdt(double & dtValue);
			void setRobotPosition(double& xValue, double& yValue, double& alphaValue);
	
	};
}

#endif /* ROBOT_H_ */