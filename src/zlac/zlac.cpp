#include "zlac/zlac.hpp"

Controller::Controller() {

	client = modbus_new_rtu("/dev/ttyUSB0", 115200, 'N', 8, 1);
	if (client == NULL) {
		throw std::runtime_error("Unable to create the libmodbus context");
	} else {
		modbus_set_slave(client, 1);
		if (modbus_connect(client) == -1) {
			throw std::runtime_error("Connection failed: " + std::string(modbus_strerror(errno)));
		}
	}

	id = 1;

	CONTROL_REG = 0x200E;
	OPR_MODE = 0x200D;
	L_ACL_TIME = 0x2080;
	R_ACL_TIME = 0x2081;
	L_DCL_TIME = 0x2082;
	R_DCL_TIME = 0x2083;

	L_ENC_LINE = 0x2030;
	R_ENC_LINE = 0x2060;
	L_HALL_ANG = 0x2031;
	R_HALL_ANG = 0x2061;
	L_MOT_POLE = 0x2045;
	R_MOT_POLE = 0x2075;
	L_CONT_CUR = 0x2033;
	R_CONT_CUR = 0x2063;
	L_PEAK_CUR = 0x2034;
	R_PEAK_CUR = 0x2064;

	L_OVER_FAC = 0x2032;
	R_OVER_FAC = 0x2062;

	L_CMD_RPM = 0x2088;
	R_CMD_RPM = 0x2089;
	L_FB_RPM = 0x20AB;
	R_FB_RPM = 0x20AC;

	L_SPD_KP = 0x203C;
	L_SPD_KI = 0x203D;
	L_SPD_KF = 0x203E;
	R_SPD_KP = 0x206C;
	R_SPD_KI = 0x206D;
	R_SPD_KF = 0x206E;

	L_SPD_COEF = 0x2037;
	L_FFW_COEF = 0x203A;
	L_TRQ_COEF = 0x203B;
	R_SPD_COEF = 0x2067;
	R_FFW_COEF = 0x206A;
	R_TRQ_COEF = 0x206B;

	L_CMD_MA = 0X2090;
	R_CMD_MA = 0X2091;
	L_FB_01A = 0X20AD;
	R_FB_01A = 0X20AE;

	L_CUR_KP = 0x2038;
	L_CUR_KI = 0x2039;
	R_CUR_KP = 0x2068;
	R_CUR_KI = 0x2069;

	POS_CONTROL_TYPE = 0x200F;

	L_MAX_RPM_POS = 0x208E;
	R_MAX_RPM_POS = 0x208F;

	L_CMD_REL_POS_HI = 0x208A;
	L_CMD_REL_POS_LO = 0x208B;
	R_CMD_REL_POS_HI = 0x208C;
	R_CMD_REL_POS_LO = 0x208D;

	L_FB_POS_HI = 0x20A7;
	L_FB_POS_LO = 0x20A8;
	R_FB_POS_HI = 0x20A9;
	R_FB_POS_LO = 0x20AA;

	L_POS_KP = 0x203F;
	R_POS_KP = 0x206F;
	L_POS_KF = 0x2040;
	R_POS_KF = 0x2070;

	MAIN_VOLT = 0x20A1;

	L_FAULT = 0x20A5;
	R_FAULT = 0x20A6;

	EMER_STOP = 0x05;
	ALRM_CLR = 0x06;
	DOWN_TIME = 0x07;
	ENABLE = 0x08;
	POS_SYNC = 0x10;
	POS_L_START = 0x11;
	POS_R_START = 0x12;

	POS_REL_CONTROL = 1;
	POS_ABS_CONTROL = 2;
	VEL_CONTROL = 3;
	TOR_CONTROL = 4;

	ASYNC = 0;
	SYNC = 1;

	NO_FAULT = 0x0000;
	OVER_VOLT = 0x0001;
	UNDER_VOLT = 0x0002;
	OVER_CURR = 0x0004;
	OVER_LOAD = 0x0008;
	CURR_OUT_TOL = 0x0010;
	ENCOD_OUT_TOL = 0x0020;
	MOTOR_BAD = 0x0040;
	REF_VOLT_ERROR = 0x0080;
	EEPROM_ERROR = 0x0100;
	WALL_ERROR = 0x0200;
	HIGH_TEMP = 0x0400;

	travel_in_one_rev = 0.655;
	cpr = 16385;
	wheel_radius = 0.105;
}

Controller::~Controller() {
	modbus_close(client);
	modbus_free(client);
}

std::vector<uint16_t> Controller::modbus_fail_read_handler(int ADDR, int WORD) {
	bool read_success = false;
	std::vector<uint16_t> reg(WORD);
	while (!read_success) {
		modbus_set_slave(client, id);
		int result = modbus_read_registers(client, ADDR, WORD, reg.data());
		if (result == WORD) {
			read_success = true;
		} else {
			RCLCPP_ERROR(rclcpp::get_logger("zlac"), "Failed to read registers: %s", modbus_strerror(errno));
		}
	}
	return reg;
}

double Controller::rpm_to_radps(double rpm) {
	return rpm * 2 * M_PI / 60.0;
}

double Controller::rpm_to_linear(double rpm) {
	return wheel_radius*Controller::rpm_to_radps(rpm);
}

void Controller::set_mode(int mode) {
	if (mode == 1) {
		std::cout << "Set relative position control" << std::endl;
	} else if (mode == 2) {
		std::cout << "Set absolute position control" << std::endl;
	} else if (mode == 3) {
		std::cout << "Set speed RPM control" << std::endl;
	} else if (mode == 4) {
		std::cout << "Set torque control" << std::endl;
	} else {
		std::cout << "Invalid mode, set only 1, 2 or 3" << std::endl;
	}

	modbus_write_register(client, OPR_MODE, mode);
}

int Controller::get_mode() {
	std::vector<uint16_t> reg = modbus_fail_read_handler(OPR_MODE, 1);
	return reg[0];
}

void Controller::set_parameter(int reg_l, int reg_r, int value_l, int value_r) {
	uint16_t left_bytes = Controller::int16dec_to_int16hex(value_l);
	uint16_t right_bytes = Controller::int16dec_to_int16hex(value_r);

	modbus_write_register(client, reg_l, value_l);
	modbus_write_register(client, reg_r, value_r);
}

void Controller::enable_motor() {
	modbus_write_register(client, CONTROL_REG, ENABLE);
}

void Controller::disable_motor() {
	modbus_write_register(client, CONTROL_REG, DOWN_TIME);
}

int Controller::get_l_fault() {
	std::vector<uint16_t> reg = modbus_fail_read_handler(L_FAULT, 1);
	return reg[0];
}

int Controller::get_r_fault() {
	std::vector<uint16_t> reg = modbus_fail_read_handler(R_FAULT, 1);
	return reg[0];
}

void Controller::clear_alarm() {
	modbus_write_register(client, CONTROL_REG, ALRM_CLR);
}

void Controller::set_accel_time(int l_ms, int r_ms) {

	if (l_ms > 32767) {
		l_ms = 32767;
	} else if (l_ms < 0) {
		l_ms = 0;
	}

	if (r_ms > 32767) {
		r_ms = 32767;
	} else if (r_ms < 0) {
		r_ms = 0;
	}

	modbus_write_register(client, L_ACL_TIME, l_ms);
	modbus_write_register(client, R_ACL_TIME, r_ms);
}

void Controller::set_decel_time(int l_ms, int r_ms) {

	if (l_ms > 32767) {
		l_ms = 32767;
	} else if (l_ms < 0) {
		l_ms = 0;
	}

	if (r_ms > 32767) {
		r_ms = 32767;
	} else if (r_ms < 0) {
		r_ms = 0;
	}

	modbus_write_register(client, L_DCL_TIME, l_ms);
	modbus_write_register(client, R_DCL_TIME, r_ms);
}

int16_t Controller::int16dec_to_int16hex(int dec) {
	uint16_t lo_bytes = dec & 0x00FF;
	uint16_t hi_bytes = (dec & 0xFF00) >> 8;

	return (hi_bytes << 8) | lo_bytes;
}

void Controller::set_rpm(int l_rpm, int r_rpm) {

	if (l_rpm > 3000) {
		l_rpm = 3000;
	} else if (l_rpm < -3000) {
		l_rpm = -3000;
	}

	if (r_rpm > 3000) {
		r_rpm = 3000;
	} else if (r_rpm < -3000) {
		r_rpm = -3000;
	}

	int16_t left_bytes = Controller::int16dec_to_int16hex(l_rpm);
	int16_t right_bytes = Controller::int16dec_to_int16hex(r_rpm);

	modbus_write_register(client, L_CMD_RPM, left_bytes);
	modbus_write_register(client, R_CMD_RPM, right_bytes);
}

int Controller::get_l_rpm() {
	std::vector<uint16_t> reg = modbus_fail_read_handler(L_FB_RPM, 1);
	return (signed int) reg[0];
}

int Controller::get_r_rpm() {
	std::vector<uint16_t> reg = modbus_fail_read_handler(R_FB_RPM, 1);
	return (signed int) reg[0];
}

double Controller::get_l_linear_velocity() {
	return Controller::rpm_to_linear(Controller::get_l_rpm());
}

double Controller::get_r_linear_velocity() {
	return Controller::rpm_to_linear(-1 * Controller::get_r_rpm());
}

double Controller::map(double x, int in_min, int in_max, int out_min, int out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Controller::set_max_rpm_pos(int l_rpm, int r_rpm) {

	if (l_rpm > 1000) {
		l_rpm = 1000;
	} else if (l_rpm < 1) {
		l_rpm = 1;
	}

	if (r_rpm > 1000) {
		r_rpm = 1000;
	} else if (r_rpm < 1) {
		r_rpm = 1;
	}

	modbus_write_register(client, L_MAX_RPM_POS, l_rpm);
	modbus_write_register(client, R_MAX_RPM_POS, r_rpm);
}

void Controller::set_ma(int l_ma, int r_ma) {

	if (l_ma > 30000) {
		l_ma = 30000;
	} else if (l_ma < -30000) {
		l_ma = -30000;
	}

	if (r_ma > 30000) {
		r_ma = 30000;
	} else if (r_ma < -30000) {
		r_ma = -30000;
	}

	modbus_write_register(client, L_CMD_MA, l_ma);
	modbus_write_register(client, R_CMD_MA, r_ma);
}

int Controller::get_l_ma() {
	std::vector<uint16_t> reg = modbus_fail_read_handler(L_FB_01A, 1);
	return (uint16_t) reg[0] * 100;
}

int Controller::get_r_ma() {
	std::vector<uint16_t> reg = modbus_fail_read_handler(R_FB_01A, 1);
	return (uint16_t) reg[0] * 100;
}

void Controller::set_pos_async_control() {
	modbus_write_register(client, POS_CONTROL_TYPE, ASYNC);
}

void Controller::move_left_wheel() {
	modbus_write_register(client, CONTROL_REG, POS_L_START);
}

void Controller::move_right_wheel() {
	modbus_write_register(client, CONTROL_REG, POS_R_START);
}

std::vector<uint16_t> Controller::deg_to_32bit(double deg) {
	int dec = static_cast<int>(map(deg, -1440, 1440, -65536, 65536));
	int HI_WORD = (dec & 0xFFFF0000) >> 16;
	int LO_WORD = dec & 0x0000FFFF;

	return {static_cast<uint16_t>(HI_WORD), static_cast<uint16_t>(LO_WORD)};
}

void Controller::set_relative_angle(double ang_l, double ang_r) {
	std::vector<uint16_t> L_array = deg_to_32bit(ang_l);
    std::vector<uint16_t> R_array = deg_to_32bit(ang_r);
    std::vector<uint16_t> all_cmds_array = L_array;
    all_cmds_array.insert(all_cmds_array.end(), R_array.begin(), R_array.end());

    modbus_write_registers(client, 
		L_CMD_REL_POS_HI, 
		all_cmds_array.size(), 
		reinterpret_cast<const uint16_t*>(all_cmds_array.data()));
}

double Controller::get_l_wheel_travel() {
	std::vector<uint16_t> reg = modbus_fail_read_handler(L_FB_POS_HI, 1);
	int HI_WORD = reg[0];
	reg = modbus_fail_read_handler(L_FB_POS_LO, 1);
	int LO_WORD = reg[0];
	int dec = (HI_WORD << 16) | LO_WORD;
	return static_cast<float>(dec) / cpr * travel_in_one_rev;
}

double Controller::get_r_wheel_travel() {
	std::vector<uint16_t> reg = modbus_fail_read_handler(R_FB_POS_HI, 1);
	int HI_WORD = reg[0];
	reg = modbus_fail_read_handler(R_FB_POS_LO, 1);
	int LO_WORD = reg[0];
	int dec = (HI_WORD << 16) | LO_WORD;
	return static_cast<float>(dec) / cpr * travel_in_one_rev;
}

int32_t Controller::get_l_ticks() {
	std::vector<uint16_t> reg = modbus_fail_read_handler(L_FB_POS_HI, 1);
	int HI_WORD = reg[0];
	reg = modbus_fail_read_handler(L_FB_POS_LO, 1);
	int LO_WORD = reg[0];
	int32_t ticks = (HI_WORD & 0xFFFF) << 16 | (LO_WORD & 0xFFFF);
	return ticks;
}

int32_t Controller::get_r_ticks() {
	std::vector<uint16_t> reg = modbus_fail_read_handler(R_FB_POS_HI, 1);
	int HI_WORD = reg[0];
	reg = modbus_fail_read_handler(R_FB_POS_LO, 1);
	int LO_WORD = reg[0];
	int32_t ticks = (HI_WORD & 0xFFFF) << 16 | (LO_WORD & 0xFFFF);
	return ticks;
}

void Controller::set_encoder_line(int l_enc, int r_enc) {
	Controller::set_parameter(L_ENC_LINE, R_ENC_LINE, l_enc, r_enc);
}

void Controller::set_phase_shift(int l_phase, int r_phase) {
	Controller::set_parameter(L_HALL_ANG, R_HALL_ANG, l_phase, r_phase);
}

void Controller::set_continuous_current(int l_cur, int r_cur) {
	Controller::set_parameter(L_CONT_CUR, R_CONT_CUR, l_cur, r_cur);
}

void Controller::set_peak_current(int l_cur, int r_cur) {
	Controller::set_parameter(L_PEAK_CUR, R_PEAK_CUR, l_cur, r_cur);
}

void Controller::set_motor_pole(int l_pole, int r_pole) {
	Controller::set_parameter(L_MOT_POLE, R_MOT_POLE, l_pole, r_pole);
}

void Controller::set_overload_factor(int l_over, int r_over) {
	Controller::set_parameter(L_OVER_FAC, R_OVER_FAC, l_over, r_over);
}

void Controller::set_position_kp(int l_kp, int r_kp) {
	Controller::set_parameter(L_POS_KP, R_POS_KP, l_kp, r_kp);
}

void Controller::set_position_kf(int l_kf, int r_kf) {
	Controller::set_parameter(L_POS_KF, R_POS_KF, l_kf, r_kf);
}

void Controller::set_speed_kp(int l_kp, int r_kp) {
	Controller::set_parameter(L_SPD_KP, R_SPD_KP, l_kp, r_kp);
}

void Controller::set_speed_ki(int l_ki, int r_ki) {
	Controller::set_parameter(L_SPD_KI, R_SPD_KI, l_ki, r_ki);
}

void Controller::set_speed_kf(int l_kf, int r_kf) {
	Controller::set_parameter(L_SPD_KF, R_SPD_KF, l_kf, r_kf);
}

void Controller::set_speed_coef(int l_coef, int r_coef) {
	Controller::set_parameter(L_SPD_COEF, R_SPD_COEF, l_coef, r_coef);
}

void Controller::set_feed_coef(int l_coef, int r_coef) {
	Controller::set_parameter(L_FFW_COEF, R_FFW_COEF, l_coef, r_coef);
}

void Controller::set_torque_coef(int l_coef, int r_coef) {
	Controller::set_parameter(L_TRQ_COEF, R_TRQ_COEF, l_coef, r_coef);
}

void Controller::set_current_kp(int l_kp, int r_kp) {
	Controller::set_parameter(L_CUR_KP, R_CUR_KP, l_kp, r_kp);
}

void Controller::set_current_ki(int l_ki, int r_ki) {
	Controller::set_parameter(L_CUR_KI, R_CUR_KI, l_ki, r_ki);
}

double Controller::get_main_voltage() {
	std::vector<uint16_t> reg = modbus_fail_read_handler(MAIN_VOLT, 1);
	return reg[0]/100;
}
