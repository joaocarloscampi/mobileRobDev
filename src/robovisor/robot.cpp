#include "robovisor/robot.hpp"

namespace ROBOT {

	/**
	 * @brief Constructor for the Robot class.
	 * 
	 * Initializes the Robot object with default values for its attributes.
	 * 
	 * @details The constructor sets the values for the wheel diameter, robot diameter,
	 * initial position (x, y), initial orientation (alpha), time step (dt), initial offset
	 * position (xo, yo), initial offset orientation (alphao), offset values (x_offset, y_offset,
	 * alpha_offset), velocity components (dx, dy, dalpha), wheel angular velocities (dthR, dthL),
	 * front and back distance sensors (distFront, distBack), left and right distance sensors (dist_L, dist_R),
	 * center distance sensor (distC), and previous position and orientation values (x_previous, y_previous, alpha_previous).
	 */
	Robot::Robot() {

		r= 1.2*0.063;   // wheel diamenter 13cm  # 1.2 gain to adjust
		b= 0.25;        // robot diamenter 52 cm

		x = 0;
		y = 0;
		alpha = 0;

		dt = 0.01;

		xo = 0.0;
		yo = 0.0;
		alphao = 0.0;

		x_offset = 0.0;
		y_offset = 0.0;
		alpha_offset = 0.0;

		dx = 0.0;
		dy = 0.0;
		dalpha = 0.0;
		dthR = 0.0;	
		dthL = 0.0;
		distFront = 250.0;
		distBack = 250.0;
		dist_L = 0.0;
		dist_R = 0.0;
		distC = 0.0;
		x_previous = 0.0;
		y_previous = 0.0;
		alpha_previous = 0.0;
	}
	
	/**
	 * @brief Sets the wheel radius of the robot.
	 *
	 * This function sets the wheel radius of the robot based on the provided input.
	 *
	 * @param wheelRaidusInput The input value representing the wheel radius.
	 */
	void Robot::setWheelRadius(double& wheelRaidusInput){
		r = wheelRaidusInput*1.0;
	}

	/**
	 * @brief Sets the radius of the robot.
	 *
	 * This function sets the radius of the robot based on the input value.
	 *
	 * @param robotRadiusInput The input value representing the robot's radius.
	 */
	void Robot::setRobotRadius(double& robotRadiusInput){
		b = robotRadiusInput*1.0;
	}

	/**
	 * @brief Get the wheel radius of the robot.
	 * 
	 * @return The wheel radius.
	 */
	double Robot::getWheelRadius() {
		return r;
	}

	/**
	 * @brief Get the radius of the robot.
	 * 
	 * @return The radius of the robot.
	 */
	double Robot::getRobotRadius() {
		return b;
	}

	/**
	 * @brief Get the value of the robot's dx (change in x) parameter.
	 * 
	 * @return The value of dx.
	 */
	double Robot::getRobotdx(){
		return dx;
	}

	/**
	 * @brief Get the value of dy for the Robot object.
	 * 
	 * @return double The value of dy.
	 */
	double Robot::getRobotdy(){
		return dy;
	}

	/**
	 * @brief Get the value of the robot's dalpha.
	 * 
	 * @return The value of dalpha.
	 */
	double Robot::getRobotdalpha(){
		return dalpha;
	}

	/**
	 * @brief Normalizes an angle within a specified range.
	 * 
	 * This function takes an angle `ang` and a lower bound `low` and normalizes the angle within the range [low, low + 2π).
	 * 
	 * @param ang The angle to be normalized.
	 * @param low The lower bound of the range.
	 * @return The normalized angle within the specified range.
	 */
	double Robot::normalizeAngle(double ang, double low) {
		return ang -2*M_PI*floor((ang-low)/(2*M_PI));
	}

	/**
	 * Applies a low-pass filter to the input value using the given alpha coefficient.
	 * 
	 * @param alpha The coefficient used to control the filter's response speed. Should be between 0 and 1.
	 * @param x The current input value.
	 * @param xpf The previous filtered value.
	 * @return The filtered output value.
	 */
	double Robot::lowPassFilter(double alpha, double x, double xpf) {
		return alpha*xpf + (1-alpha)*x;
	}

	/**
	 * @brief Sets the value of dt for the Robot object.
	 *
	 * This function updates the value of dt, which represents the time step, for the Robot object.
	 *
	 * @param dtValue The new value of dt.
	 */
	void Robot::setRobotdt(double & dtValue) {
		dt = dtValue;
	}

	/**
	 * @brief Sets the position of the robot.
	 * 
	 * This function updates the position of the robot with the provided x, y, and alpha values.
	 * It also calculates the velocity components (dx, dy, dalpha) using a low-pass filter.
	 * 
	 * @param xValue The new x-coordinate of the robot.
	 * @param yValue The new y-coordinate of the robot.
	 * @param alphaValue The new orientation angle (in radians) of the robot.
	 */
	void Robot::setRobotPosition(double& xValue, double& yValue, double& alphaValue) {
		x = xValue;
		y = yValue;
		alpha = alphaValue;

		double dx_raw, dy_raw, dalpha_raw;

		dx_raw=(x-x_previous)/dt;
		dy_raw=(y-y_previous)/dt;
		dalpha_raw = normalizeAngle((alpha-alpha_previous),-M_PI)/dt;

		dx = lowPassFilter(0.7,dx_raw,dx);
		dy = lowPassFilter(0.7,dy_raw,dy);
		dalpha = lowPassFilter(0.7,dalpha_raw,dalpha);

		x_previous = x;
		y_previous = y;
		alpha_previous = alpha;
	}

} // namespace ROBOT