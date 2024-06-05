#ifndef ZLAC_H_
#define ZLAC_H_

extern "C" {
	#include <modbus.h>
}
#include <math.h>

#include "rclcpp/rclcpp.hpp"

class Controller {
	public:
		Controller();
		~Controller();
		std::vector<uint16_t> modbus_fail_read_handler(int ADDR, int WORD);
		double rpm_to_radps(double rpm);
		double rpm_to_linear(double rpm);
		void set_mode(int mode);
		int get_mode();
		void set_parameter(int reg_l, int reg_r, int value_l, int value_r);
		void enable_motor();
		void disable_motor();
		int get_l_fault();
		int get_r_fault();
		void clear_alarm();
		void set_accel_time(int l_ms, int r_ms);
		void set_decel_time(int l_ms, int r_ms);
		int16_t int16dec_to_int16hex(int dec);
		void set_rpm(int l_rpm, int r_rpm);
		int get_l_rpm();
		int get_r_rpm();
		double get_l_linear_velocity();
		double get_r_linear_velocity();
		double map(double x, int in_min, int in_max, int out_min, int out_max);
		void set_max_rpm_pos(int l_rpm, int r_rpm);
		void set_ma(int l_ma, int r_ma);
		int get_l_ma();
		int get_r_ma();
		void set_pos_async_control();
		void move_left_wheel();
		void move_right_wheel();
		std::vector<uint16_t> deg_to_32bit(double deg);
		void set_relative_angle(double angle_l, double angle_r);
		double get_l_wheel_travel();
		double get_r_wheel_travel();
		int32_t get_l_ticks();
		int32_t get_r_ticks();
		void set_encoder_line(int l_enc, int r_enc);
		void set_phase_shift(int l_phase, int r_phase);
		void set_continuous_current(int l_cur, int r_cur);
		void set_peak_current(int l_cur, int r_cur);
		void set_motor_pole(int l_pole, int r_pole);
		void set_overload_factor(int l_over, int r_over);
		void set_position_kp(int l_kp, int r_kp);
		void set_position_kf(int l_kf, int r_kf);
		void set_speed_kp(int l_kp, int r_kp);
		void set_speed_ki(int l_ki, int r_ki);
		void set_speed_kf(int l_kf, int r_kf);
		void set_speed_coef(int l_coef, int r_coef);
		void set_feed_coef(int l_coef, int r_coef);
		void set_torque_coef(int l_coef, int r_coef);
		void set_current_kp(int l_kp, int r_kp);
		void set_current_ki(int l_ki, int r_ki);
		double get_main_voltage();

	private:
		modbus_t *client;
		int id;

		// registers
		int CONTROL_REG;
		int OPR_MODE;
		int L_ACL_TIME;
		int R_ACL_TIME;
		int L_DCL_TIME;
		int R_DCL_TIME;

		// motor parameters
		int L_ENC_LINE;
		int R_ENC_LINE;

		int L_HALL_ANG;
		int R_HALL_ANG;

		int L_MOT_POLE;
		int R_MOT_POLE;

		int L_CONT_CUR;
		int R_CONT_CUR;

		int L_PEAK_CUR;
		int R_PEAK_CUR;

		// overload

		int L_OVER_FAC;
		int R_OVER_FAC;

		// velocity control
		int L_CMD_RPM;
		int R_CMD_RPM;
		int L_FB_RPM;
		int R_FB_RPM;

		int L_SPD_KP;
		int L_SPD_KI;
		int L_SPD_KF;
		int R_SPD_KP;
		int R_SPD_KI;
		int R_SPD_KF;

		// output coeff

		int L_SPD_COEF;
		int L_FFW_COEF;
		int L_TRQ_COEF;
		int R_SPD_COEF;
		int R_FFW_COEF;
		int R_TRQ_COEF;

		// torque control

		int L_CMD_MA;
		int R_CMD_MA;
		int L_FB_01A;
		int R_FB_01A;

		int L_CUR_KP;
		int L_CUR_KI;
		int R_CUR_KP;
		int R_CUR_KI;

		// position control
		int POS_CONTROL_TYPE;

		int L_MAX_RPM_POS;
		int R_MAX_RPM_POS;

		int L_CMD_REL_POS_HI;
		int L_CMD_REL_POS_LO;
		int R_CMD_REL_POS_HI;
		int R_CMD_REL_POS_LO;

		int L_FB_POS_HI;
		int L_FB_POS_LO;
		int R_FB_POS_HI;
		int R_FB_POS_LO;

		int L_POS_KP;
		int R_POS_KP;
		int L_POS_KF;
		int R_POS_KF;

		int MAIN_VOLT;

		// troubleshooting
		int L_FAULT;
		int R_FAULT;

		// control cmds
		int EMER_STOP;
		int ALRM_CLR;
		int DOWN_TIME;
		int ENABLE;
		int POS_SYNC;
		int POS_L_START;
		int POS_R_START;

		// operation mode
		int POS_REL_CONTROL;
		int POS_ABS_CONTROL;
		int VEL_CONTROL;
		int TOR_CONTROL;

		int SYNC;
		int ASYNC;

		// fault codes;
		int NO_FAULT;
		int OVER_VOLT;
		int UNDER_VOLT;
		int OVER_CURR;
		int OVER_LOAD;
		int CURR_OUT_TOL;
		int ENCOD_OUT_TOL;
		int MOTOR_BAD;
		int REF_VOLT_ERROR;
		int EEPROM_ERROR;
		int WALL_ERROR;
		int HIGH_TEMP;

		double travel_in_one_rev;
		int cpr;
		double wheel_radius;
};

#endif // ZLAC_H_