/*
 * zpdc_bldc.h
 *
 * Created: 6/12/2017 2:31:54 PM
 *  Author: Andres Vasquez
 */ 


#ifndef ZPDC_BLDC_H_
#define ZPDC_BLDC_H_
#ifdef __cplusplus

#define MOTOR_FREEWHEEL		6
#define MOTOR_STOP			7

#define MOTOR_STATUS_RUN	0
#define MOTOR_STATUS_BRAKE	1
#define MOTOR_STATUS_FREE	2

#define MOTOR_DEFINITION_CW		false
#define MOTOR_DEFINITION_CCW	true

#define MOTOR_MAX_DUTY		((uint16_t) 0x0BFF)

#define MOTOR_RAMP_IDLE		0
#define MOTOR_RAMP_UP		1
#define MOTOR_RAMP_DOWN		2

// MOTOR Constants
const constexpr struct MotorOneParameters{
		// Hall A
	uint32_t hall_a_pin = PIN_PA04A_EIC_EXTINT4;
	uint32_t hall_a_mux = PINMUX_PA04A_EIC_EXTINT4;
	uint8_t hall_a_line = 4;

		// Hall B
	uint32_t hall_b_pin = PIN_PA05A_EIC_EXTINT5;
	uint32_t hall_b_mux = PINMUX_PA05A_EIC_EXTINT5;
	uint8_t hall_b_line = 5;

		// Hall C
	uint32_t hall_c_pin = PIN_PA06A_EIC_EXTINT6;
	uint32_t hall_c_mux = PINMUX_PA06A_EIC_EXTINT6;
	uint8_t hall_c_line = 6;

	Tcc *tcc_motor_module = TCC0;
	uint32_t wave_out_p0 = PIN_PA08E_TCC0_WO0;
	uint32_t wave_out_p1 = PIN_PA09E_TCC0_WO1;
	uint32_t wave_out_p2 = PIN_PA10F_TCC0_WO2;
	uint32_t wave_out_p4 = PIN_PA14F_TCC0_WO4;
	uint32_t wave_out_p5 = PIN_PA15F_TCC0_WO5;
	uint32_t wave_out_p6 = PIN_PA16F_TCC0_WO6;

	uint32_t wave_mix_p0 = PINMUX_PA08E_TCC0_WO0;
	uint32_t wave_mix_p1 = PINMUX_PA09E_TCC0_WO1;
	uint32_t wave_mix_p2 = PINMUX_PA10F_TCC0_WO2;
	uint32_t wave_mix_p4 = PINMUX_PA14F_TCC0_WO4;
	uint32_t wave_mix_p5 = PINMUX_PA15F_TCC0_WO5;
	uint32_t wave_mix_p6 = PINMUX_PA16F_TCC0_WO6;

	Tc *speed_measurement_module = TC0;
	Tc *pid_timer_module = TC4;

		// Callback
	void (*ei_callback)(void) = extint_callback_wrapper;
	void (*tsm_callback)(struct tc_module *const module_inst) = speed_measurement_timer_callback;
	void (*pidt_callback)(struct tc_module *const module_inst) = pid_loop_timer_callback;
} motor_one_parameters;

class motor_controller;
class pid_controller;

class motor_ramp : Task {
public:
	void init(motor_controller *instance);

	void setRampStatus(uint8_t rampStatus) { ramp_status = rampStatus; }
	TaskHandle_t getTaskHandle(void) { return handle; }

	void task(void);

	uint16_t ramp_duration;
	uint16_t target_speed;
private:
	int16_t ramp_delta;
	uint8_t ramp_status;
	motor_controller *controller_motor;

	uint32_t getCurrentMotorValue(void);
	void calculate_ramp_delta(void);
};

/************************************************************************/
/* SPEED MEASUREMENT CLASS DEPENDS ON 8MHZ BASE CLOCK                   */
/************************************************************************/
class speed_measurement {
public:
	speed_measurement(void);

	uint16_t measured_speed;
	bool is_measurement_new;
	volatile uint32_t revolution_time_delta;

	void init(const MotorOneParameters *mot_pars, pid_controller *pid);
	void timer_callback(void);
	void get_speed_measurement_callback(void);
	uint32_t get_speed(void);

	bool get_pid_flag(void) {
			// Automatically Clear Flag
		bool result = is_pid_to_execute;
		is_pid_to_execute = false;

		return result;
	}

	void set_pid_flag(bool value) { is_pid_to_execute = value; }
private:
	pid_controller *controller;
	struct tc_module timer_module;
	uint8_t revolution_counter;
	bool is_pid_to_execute;
};

class pid_controller : public Task {
public:
	void init(const MotorOneParameters *mot_pars, 
				speed_measurement *speed_detector, 
				uint8_t *motor_status,
				motor_controller *controller_motor);
	
	void task(void);
	void timer_callback(void);

	void reset(void);

	uint16_t setpoint;

	float kp, ki, kd;
private:
	struct tc_module timer_module;

	speed_measurement *speed_sensor;
	motor_controller *controller;

	int pid_error;
	int pid_integral;
	int pid_derivative, pid_previous_error;
	int pid_output;

	uint8_t *status;
	uint16_t pid_time_delta;

	uint16_t get_granular_time(void) {
		uint8_t result = tc_get_capture_value(&timer_module, TC_COMPARE_CAPTURE_CHANNEL_0) / 4000ul;
		tc_set_count_value(&timer_module, 0);
		return result;
	}
};

/************************************************************************/
/*   BLDC COMMUTATION FIRMWARE V1.0, JUNE 2017                          */
/************************************************************************/
class motor_controller {
public:
	motor_controller(ZpdcSystem *zpdc_system, const MotorOneParameters *mot_pars);

	// C - Accessible
	void hall_callback(void);

	void setMotorDuty(uint16_t duty) {
		motor_duty = duty;
		tcc_set_compare_value(&tcc_instance, TCC_MATCH_CAPTURE_CHANNEL_0, motor_duty);
		tcc_set_compare_value(&tcc_instance, TCC_MATCH_CAPTURE_CHANNEL_1, motor_duty);
		tcc_set_compare_value(&tcc_instance, TCC_MATCH_CAPTURE_CHANNEL_2, motor_duty);
	}

	uint16_t getMotorDuty(void) { return motor_duty; }
	bool isMotorRunning(void) { return is_motor_running; }
	void setMotorRunning(bool motState) { 
		is_motor_running = motState;
		if (motState) ramp.setRampStatus(MOTOR_RAMP_UP);
		else ramp.setRampStatus(MOTOR_RAMP_DOWN);
	}
	TaskHandle_t getRampTaskHandle(void) { return ramp.getTaskHandle(); }
	void force_motor_commutation(void);

	struct tcc_module tcc_instance;
	uint8_t motor_status;
	speed_measurement speed_sensor;
	pid_controller pid_instance;
	motor_ramp ramp;
private:
	static const uint8_t cw_pattern_enable[8];
	static const uint8_t cw_pattern_value[8];
	static const uint8_t ccw_pattern_enable[6];
	static const uint8_t ccw_pattern_value[6];

	uint16_t motor_duty;
	uint8_t a, b, c, hall_status;
	bool motor_direction;
	bool is_motor_running;

	uint8_t getHallSensorStatus(void);
};

#endif // __cplusplus
#endif /* ZPDC_BLDC_H_ */