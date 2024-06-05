#include "zlac/robot.hpp"

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
		r = 0.069;   // wheel diamenter 13cm  # 1.2 gain to adjust
		b = 0.262;        // robot diamenter 52 cm

		x = 0;
		y = 0;
		theta = 0;

		dx = 0.0;
		dy = 0.0;
		dtheta = 0.0;
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
	double Robot::normalize_angle(double ang, double low) {
		return ang - 2 * M_PI * floor((ang - low) / (2 * M_PI));
	}

	/**
	 * Applies a low-pass filter to the input value using the given alpha coefficient.
	 * 
	 * @param alpha The coefficient used to control the filter's response speed. Should be between 0 and 1.
	 * @param x The current input value.
	 * @param xpf The previous filtered value.
	 * @return The filtered output value.
	 */
	double Robot::low_pass_filter(double alpha, double x, double xpf) {
		return alpha * xpf + (1 - alpha) * x;
	}
} // namespace ROBOT