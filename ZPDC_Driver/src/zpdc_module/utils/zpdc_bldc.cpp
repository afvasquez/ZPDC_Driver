/*
 * zpdc_bldc.cpp
 *
 * Created: 6/12/2017 2:32:11 PM
 *  Author: Andres Vasquez
 *  NOTES: Change pin out according to the board being used
 */ 
 #include <asf.h>

 #define SELECTED_SPEED 4824
 #define SELECTED_RAMP	400

 //const uint8_t motor_controller::cw_pattern_enable[8] = { 0x66, 0x55, 0x55, 0x33, 0x66, 0x33, 0x77, 0x77 };
 //const uint8_t motor_controller::cw_pattern_value[8] =  { 0x20, 0x40, 0x10, 0x10, 0x40, 0x20, 0x70, 0x00 };
 const uint8_t motor_controller::cw_pattern_enable[8] = { 0x66, 0x55, 0x66, 0x33, 0x33, 0x55, 0x77, 0x77 };
 const uint8_t motor_controller::cw_pattern_value[8] =  { 0x40, 0x10, 0x20, 0x20, 0x10, 0x40, 0x70, 0x00 };
 const uint8_t motor_controller::ccw_pattern_enable[6] = { 0x55, 0x33, 0x33, 0x66, 0x55, 0x66 };
 const uint8_t motor_controller::ccw_pattern_value[6] =  { 0x40, 0x10, 0x20, 0x20, 0x10, 0x40 };

 motor_controller::motor_controller(ZpdcSystem *zpdc_system, const MotorOneParameters *mot_pars) {
	struct extint_chan_conf chan_config;
	
		// HALL Sensor A
	extint_chan_get_config_defaults(&chan_config);
	chan_config.gpio_pin = mot_pars->hall_a_pin;
	chan_config.gpio_pin_mux = mot_pars->hall_a_mux;
	chan_config.gpio_pin_pull = EXTINT_PULL_DOWN;
	chan_config.detection_criteria = EXTINT_DETECT_BOTH;
	chan_config.filter_input_signal = true;
	extint_chan_set_config(mot_pars->hall_a_line, &chan_config);

		// HALL Sensor B
	extint_chan_get_config_defaults(&chan_config);
	chan_config.gpio_pin = mot_pars->hall_b_pin;
	chan_config.gpio_pin_mux = mot_pars->hall_b_mux;
	chan_config.gpio_pin_pull = EXTINT_PULL_DOWN;
	chan_config.detection_criteria = EXTINT_DETECT_BOTH;
	chan_config.filter_input_signal = true;
	extint_chan_set_config(mot_pars->hall_b_line, &chan_config);
	
		// HALL Sensor C
	extint_chan_get_config_defaults(&chan_config);
	chan_config.gpio_pin = mot_pars->hall_c_pin;
	chan_config.gpio_pin_mux = mot_pars->hall_c_mux;
	chan_config.gpio_pin_pull = EXTINT_PULL_DOWN;
	chan_config.detection_criteria = EXTINT_DETECT_BOTH;
	chan_config.filter_input_signal = true;
	extint_chan_set_config(mot_pars->hall_c_line, &chan_config);

		// NOTE: 
	// Should there ever be more than 1 motor per zone card, every motor should
	//	have a unique static wrapper for this purpose
	extint_register_callback(mot_pars->ei_callback, mot_pars->hall_a_line, EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(mot_pars->hall_a_line, EXTINT_CALLBACK_TYPE_DETECT);

	extint_register_callback(mot_pars->ei_callback, mot_pars->hall_b_line, EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(mot_pars->hall_b_line, EXTINT_CALLBACK_TYPE_DETECT);

	extint_register_callback(mot_pars->ei_callback, mot_pars->hall_c_line, EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(mot_pars->hall_c_line, EXTINT_CALLBACK_TYPE_DETECT);

		// TCC Module Setup
	struct tcc_config config_tcc;
	tcc_get_config_defaults(&config_tcc, mot_pars->tcc_motor_module);

	config_tcc.pins.enable_wave_out_pin[0] = true;
	config_tcc.pins.enable_wave_out_pin[1] = true;
	config_tcc.pins.enable_wave_out_pin[2] = true;
	config_tcc.pins.enable_wave_out_pin[4] = true;
	config_tcc.pins.enable_wave_out_pin[5] = true;
	config_tcc.pins.enable_wave_out_pin[6] = true;

	config_tcc.pins.wave_out_pin[0] = mot_pars->wave_out_p0;
	config_tcc.pins.wave_out_pin[1] = mot_pars->wave_out_p1;
	config_tcc.pins.wave_out_pin[2] = mot_pars->wave_out_p2;
	config_tcc.pins.wave_out_pin[4] = mot_pars->wave_out_p4;
	config_tcc.pins.wave_out_pin[5] = mot_pars->wave_out_p5;
	config_tcc.pins.wave_out_pin[6] = mot_pars->wave_out_p6;

	config_tcc.pins.wave_out_pin_mux[0] = mot_pars->wave_mix_p0;
	config_tcc.pins.wave_out_pin_mux[1] = mot_pars->wave_mix_p1;
	config_tcc.pins.wave_out_pin_mux[2] = mot_pars->wave_mix_p2;
	config_tcc.pins.wave_out_pin_mux[4] = mot_pars->wave_mix_p4;
	config_tcc.pins.wave_out_pin_mux[5] = mot_pars->wave_mix_p5;
	config_tcc.pins.wave_out_pin_mux[6] = mot_pars->wave_mix_p6;

	config_tcc.wave_ext.invert[4] = true;
	config_tcc.wave_ext.invert[5] = true;
	config_tcc.wave_ext.invert[6] = true;

	config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;
	config_tcc.counter.clock_source = GCLK_GENERATOR_0;
	config_tcc.counter.clock_prescaler = TCC_CLOCK_PRESCALER_DIV1;
	config_tcc.counter.period = 0x0BFF;

	config_tcc.wave.match[0] = 0x00;
	config_tcc.wave.match[1] = 0x00;
	config_tcc.wave.match[2] = 0x00;

	mot_pars->tcc_motor_module->WEXCTRL.reg |= TCC_WEXCTRL_OTMX(2);

	tcc_init(&tcc_instance, mot_pars->tcc_motor_module, &config_tcc);
	tcc_enable_double_buffering(&tcc_instance);
	tcc_enable(&tcc_instance);

	motor_status = MOTOR_STATUS_FREE;
	mot_pars->tcc_motor_module->PATT.reg = TCC_PATT_PGE(cw_pattern_enable[MOTOR_FREEWHEEL]) |
										   TCC_PATT_PGV(cw_pattern_value[MOTOR_FREEWHEEL]);
	mot_pars->tcc_motor_module->PATTBUF.reg = TCC_PATTBUF_PGEB(cw_pattern_enable[MOTOR_FREEWHEEL]) |
											  TCC_PATTBUF_PGVB(cw_pattern_value[MOTOR_FREEWHEEL]);

	a = (uint8_t)mot_pars->hall_a_pin;
	b = (uint8_t)mot_pars->hall_b_pin;
	c = (uint8_t)mot_pars->hall_c_pin;

	motor_duty = 0;

	hall_status = getHallSensorStatus();

	motor_direction = zpdc_system->get_boolean_subpage_bit(4, 0);

	ramp.init(this);
	zpdc_system->read_ramp_values(&(ramp.ramp_duration),&(ramp.target_speed));
	
	pid_instance.init(&motor_one_parameters, &speed_sensor, &motor_status, this);
	zpdc_system->read_pid_values(&(pid_instance.kp), &(pid_instance.ki), &(pid_instance.kd));
	
	speed_sensor.init(mot_pars, &pid_instance);

	is_motor_running = false;
 }
 void motor_controller::hall_callback() {
	force_motor_commutation();
	speed_sensor.get_speed_measurement_callback();
 }

 volatile uint8_t enable_setting;
 volatile uint8_t value_setting;
 void motor_controller::force_motor_commutation(void) {
	hall_status = getHallSensorStatus();

	switch (motor_status) {
		case MOTOR_STATUS_RUN:
		if (hall_status > 0) {
			if (motor_direction) {
				enable_setting = ccw_pattern_enable[hall_status-1];
				value_setting = ccw_pattern_value[hall_status-1];

				tcc_instance.hw->PATTBUF.reg = TCC_PATTBUF_PGEB(enable_setting) |
											   TCC_PATTBUF_PGVB(value_setting);
				
			}
			else {
				tcc_instance.hw->PATTBUF.reg = TCC_PATTBUF_PGEB(cw_pattern_enable[hall_status-1]) |
											   TCC_PATTBUF_PGVB(cw_pattern_value[hall_status-1]);
				enable_setting = cw_pattern_enable[hall_status-1];
				value_setting = cw_pattern_value[hall_status-1];
			}
		} else {
			setMotorDuty(0);
			tcc_instance.hw->PATTBUF.reg = TCC_PATTBUF_PGEB(cw_pattern_enable[MOTOR_FREEWHEEL]) |
									       TCC_PATTBUF_PGVB(cw_pattern_value[MOTOR_FREEWHEEL]);
		}
		break;
		case MOTOR_STATUS_BRAKE:
			setMotorDuty(0);
			tcc_instance.hw->PATTBUF.reg = TCC_PATTBUF_PGEB(cw_pattern_enable[MOTOR_STOP]) |
										   TCC_PATTBUF_PGVB(cw_pattern_value[MOTOR_STOP]);
		break;
		case MOTOR_STATUS_FREE:
		default:
			setMotorDuty(0);
			tcc_instance.hw->PATTBUF.reg = TCC_PATTBUF_PGEB(cw_pattern_enable[MOTOR_FREEWHEEL]) |
										   TCC_PATTBUF_PGVB(cw_pattern_value[MOTOR_FREEWHEEL]);
		break;
	}

	tcc_force_double_buffer_update(&tcc_instance);
 }

 volatile uint8_t last_two_revolutions[13];
 volatile uint8_t revolution_counter = 0;

 uint8_t motor_controller::getHallSensorStatus(void) {
	uint8_t hall_code = (port_pin_get_input_level(c) ? 0x04 : 0x00) | 
						(port_pin_get_input_level(b) ? 0x02 : 0x00) | 
						(port_pin_get_input_level(a) ? 0x01 : 0x00);

	last_two_revolutions[revolution_counter++] = hall_code;
	if (revolution_counter > 12) {
		revolution_counter = 0;
	}

	if ((hall_code < 1) || (hall_code > 6)) hall_code = 0;
	return hall_code;
 }
 
 void motor_ramp::init(motor_controller *instance) {
	ramp_status = MOTOR_RAMP_IDLE;

	controller_motor = instance;

	t_init("RAMP", 2, 1);
 }
 uint32_t motor_ramp::getCurrentMotorValue(void) {
	uint32_t result;
	result = tcc_get_capture_value(&(controller_motor->tcc_instance), TCC_MATCH_CAPTURE_CHANNEL_0);
	if (result == tcc_get_capture_value(&(controller_motor->tcc_instance), TCC_MATCH_CAPTURE_CHANNEL_2) &&
		result == tcc_get_capture_value(&(controller_motor->tcc_instance), TCC_MATCH_CAPTURE_CHANNEL_2))
			return result;
	else return (uint32_t)0x0C00;
	
 }
 void motor_ramp::calculate_ramp_delta(void) {
	float delta;
	if (ramp_status == MOTOR_RAMP_UP) delta = (float)target_speed / (float)ramp_duration;
	else if (ramp_status == MOTOR_RAMP_DOWN) delta = (float)(((float)target_speed * -1) / (float)ramp_duration);
	else {
		ramp_delta = 0;
		return;
	}
	ramp_delta = delta * 10;	// To 10ms intervals
 }
 void motor_ramp::task(void) {
	TickType_t xLastWakeTime;
	controller_motor->motor_status = MOTOR_STATUS_FREE;
	controller_motor->hall_callback();

	bool run_start = false;
	int new_duty;

	for(;;) { 
		vTaskSuspend(NULL);

		if (ramp_status == MOTOR_RAMP_UP) {
			portENTER_CRITICAL();

				// Reset all PID Loop Controls
			controller_motor->pid_instance.reset();

			controller_motor->motor_status = MOTOR_STATUS_RUN;
			controller_motor->force_motor_commutation();
			new_duty = 0;

			run_start = true;

			portEXIT_CRITICAL();
		} else if (ramp_status == MOTOR_RAMP_DOWN) new_duty = target_speed;

		calculate_ramp_delta();

		xLastWakeTime = xTaskGetTickCount();
		while(ramp_status) {
			new_duty = controller_motor->pid_instance.setpoint + ramp_delta;
			if (ramp_status == MOTOR_RAMP_UP) {
				if (new_duty > target_speed) {
					new_duty = target_speed;
					ramp_status = MOTOR_RAMP_IDLE;
				}
				controller_motor->pid_instance.setpoint = (uint16_t)new_duty;
				controller_motor->speed_sensor.set_pid_flag(true);
			} else if (ramp_status == MOTOR_RAMP_DOWN) {
				if (new_duty < 0) {
					controller_motor->pid_instance.setpoint = 0;
					new_duty = 0;
					ramp_status = MOTOR_RAMP_IDLE;
					controller_motor->motor_status = MOTOR_STATUS_FREE;
					controller_motor->hall_callback();
				}
				controller_motor->pid_instance.setpoint = (uint16_t)new_duty;
				controller_motor->speed_sensor.set_pid_flag(true);
			} else {
				ramp_status = MOTOR_RAMP_IDLE;
			}

			if ( run_start ) {
				run_start = false;
				vTaskResume(controller_motor->pid_instance.handle);
			}

			if(ramp_status)
				vTaskDelayUntil( &xLastWakeTime, 5 );	// 10ms
		}
	}
 }

 speed_measurement::speed_measurement(void) {
	measured_speed = 0;
	revolution_counter = 0;
	revolution_time_delta = 0;
	is_measurement_new = false;
	is_pid_to_execute = false;
 }
 void speed_measurement::init(const MotorOneParameters *mot_pars, pid_controller *pid) {
	struct tc_config config_tc;
	tc_get_config_defaults(&config_tc);

	config_tc.clock_source = GCLK_GENERATOR_3;
	config_tc.clock_prescaler = TC_CLOCK_PRESCALER_DIV4;
	config_tc.counter_size = TC_COUNTER_SIZE_32BIT;
	config_tc.counter_32_bit.compare_capture_channel[0] = 800000ul;

	tc_init(&timer_module, mot_pars->speed_measurement_module, &config_tc);
	tc_enable(&timer_module);

	tc_register_callback(&timer_module, mot_pars->tsm_callback, TC_CALLBACK_CC_CHANNEL0);
	tc_enable_callback(&timer_module, TC_CALLBACK_CC_CHANNEL0);

	controller = pid;
 }
 void speed_measurement::timer_callback(void) {
	measured_speed = 0;
	is_measurement_new = true;
	revolution_counter = 0;
	revolution_time_delta = 0;

	//port_pin_toggle_output_level(LED_ERROR);
	//vTaskResume
		// Reset counter
	tc_set_count_value(&timer_module, 0);
 }
 void speed_measurement::get_speed_measurement_callback(void) {
	revolution_counter++;
	if (revolution_counter > 23) {
		revolution_counter = 0;
		revolution_time_delta = tc_get_count_value(&timer_module);
		revolution_time_delta = tc_get_count_value(&timer_module);
		tc_set_count_value(&timer_module, 0);
		is_measurement_new = true;
	}
 }
 uint32_t speed_measurement::get_speed(void) {
	if (is_measurement_new) {
		if (revolution_time_delta > 0) {
			measured_speed = (uint16_t) ( 9600000ul / revolution_time_delta );
		} else measured_speed = 0;
		
		is_measurement_new = false;
		is_pid_to_execute = true;
	}

	return measured_speed;
 }

 
 void pid_controller::init(const MotorOneParameters *mot_pars, 
								speed_measurement *speed_detector, 
								uint8_t *motor_status,
								motor_controller *controller_motor) {
	struct tc_config config_tc;
	tc_get_config_defaults(&config_tc);

	config_tc.clock_source = GCLK_GENERATOR_4;
	config_tc.clock_prescaler = TC_CLOCK_PRESCALER_DIV1;
	config_tc.counter_size = TC_COUNTER_SIZE_16BIT;
	config_tc.counter_16_bit.compare_capture_channel[0] = 39995;

	tc_init(&timer_module, mot_pars->pid_timer_module, &config_tc);
	tc_enable(&timer_module);

	tc_register_callback(&timer_module, mot_pars->pidt_callback, TC_CALLBACK_CC_CHANNEL0);
	tc_enable_callback(&timer_module, TC_CALLBACK_CC_CHANNEL0);

	speed_sensor = speed_detector;
	status = motor_status;
	controller = controller_motor;

	setpoint = 0;

	t_init("PID", 3, 1);
 }
 void pid_controller::task(void) {
	volatile float elapsed_time;
	pid_error = 0;
	pid_integral = 0;
	pid_derivative = 0;
	pid_output = 0;
	pid_previous_error = 0;

	tc_set_count_value(&timer_module, 0);
	pid_time_delta = 0;

	for(;;) {
		vTaskSuspend(NULL);

			// Initial and Final Commutation Step
		if (*status != MOTOR_STATUS_RUN) {
			controller->setMotorDuty((uint16_t)0);
			controller->force_motor_commutation();
			setpoint = 0;
			pid_previous_error = 0;
		} else if (setpoint > 0) {
			// PID Loop
				// Get and Calculate Speed
			speed_sensor->get_speed();

			if (speed_sensor->get_pid_flag()) {
				pid_error = setpoint - speed_sensor->measured_speed;

				if (pid_error != 0) {
					pid_time_delta = (pid_time_delta * 10);
					pid_time_delta += get_granular_time();
					
					if (pid_time_delta > 0) {
						elapsed_time = ((float) pid_time_delta) / ((float) 1000.0);
						pid_integral = pid_error * elapsed_time;
						pid_derivative = (int)((pid_error - pid_previous_error) / elapsed_time);
						pid_output = ((kp*pid_error) + (ki*pid_integral) + (kd*pid_derivative));
					} else pid_output = 0;

					pid_output = controller->getMotorDuty() + pid_output;
					if (pid_output < 0) pid_output = 0;
					if (pid_output > 0)
						if (pid_output > 3071)
							controller->setMotorDuty((uint16_t) 0x0BFF);
						else
							controller->setMotorDuty((uint16_t) pid_output);
					else
						controller->setMotorDuty((uint16_t) 0);
				}

				pid_previous_error = pid_error;
			}
		}
	}
 }
 void pid_controller::timer_callback(void) {
	BaseType_t xYieldRequired = pdFALSE;
	pid_time_delta++;

	if (controller->motor_status == MOTOR_STATUS_RUN)
		xYieldRequired = xTaskResumeFromISR(handle);

	tc_set_count_value(&timer_module, 0);
	portYIELD_FROM_ISR(xYieldRequired);
 }
 void pid_controller::reset(void) {
	 pid_error = 0;
	 pid_time_delta = 0;
	 pid_previous_error = 0;
	 speed_sensor->revolution_time_delta = 0;
	 speed_sensor->is_measurement_new = true;

	 tc_set_count_value(&timer_module, 0);
 }