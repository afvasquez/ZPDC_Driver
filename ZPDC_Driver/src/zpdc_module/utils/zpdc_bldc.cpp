/*
 * zpdc_bldc.cpp
 *
 * Created: 6/12/2017 2:32:11 PM
 *  Author: Andres Vasquez
 *  NOTES: Change pin out according to the board being used
 */ 
 #include <asf.h>

 const uint8_t motor_controller::cw_pattern_enable[8] = { 0x66, 0x55, 0x55, 0x33, 0x66, 0x33, 0x77, 0x77 };
 const uint8_t motor_controller::cw_pattern_value[8] =  { 0x20, 0x40, 0x10, 0x10, 0x40, 0x20, 0x70, 0x00 };
 const uint8_t motor_controller::ccw_pattern_enable[6] = { 0x33, 0x66, 0x33, 0x55, 0x55, 0x66 };
 const uint8_t motor_controller::ccw_pattern_value[6] =  { 0x20, 0x40, 0x10, 0x10, 0x40, 0x20 };

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

	hall_status = getHallSensorStatus();

	motor_direction = zpdc_system->get_boolean_subpage_bit(4, 0);

	ramp.init(this);
 }

 void motor_controller::hall_callback() {
	hall_status = getHallSensorStatus();

	switch (motor_status) {
		case MOTOR_STATUS_RUN:
			if (hall_status > 0) {
				if (motor_direction) 
					tcc_instance.hw->PATTBUF.reg = TCC_PATTBUF_PGEB(ccw_pattern_enable[hall_status-1]) | 
												   TCC_PATTBUF_PGVB(ccw_pattern_value[hall_status-1]);
				else
					tcc_instance.hw->PATTBUF.reg = TCC_PATTBUF_PGEB(cw_pattern_enable[hall_status-1]) | 
												   TCC_PATTBUF_PGVB(cw_pattern_value[hall_status-1]);
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

 uint8_t motor_controller::getHallSensorStatus(void) {
	uint8_t hall_code = (port_pin_get_input_level(c) ? 0x04 : 0x00) | 
						(port_pin_get_input_level(b) ? 0x02 : 0x00) | 
						(port_pin_get_input_level(a) ? 0x01 : 0x00);

	if ((hall_code < 1) || (hall_code > 6)) hall_code = 0;
	return hall_code;
 }
 

 void motor_ramp::init(motor_controller *instance) {
	ramp_duration = 500; // 500ms
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
	if (ramp_status == MOTOR_RAMP_UP) delta = (float)0x07EE / (float)ramp_duration;
	else if (ramp_status == MOTOR_RAMP_DOWN) delta = (float)(((float)0x07EE * -1) / (float)ramp_duration);
	else {
		ramp_delta = 0;
		return;
	}
	ramp_delta = delta * 10;
 }

 void motor_ramp::task(void) {
	 TickType_t xLastWakeTime;
	 int new_duty;
	 bool trigger = true;

	 xLastWakeTime = xTaskGetTickCount() + 1500;
	 for(;;) {
		 port_pin_set_output_level(LED_ERROR, false);
		 port_pin_set_output_level(LED_WARNING, false);
		 
		 vTaskDelay(1500);
		 trigger = !trigger;
		 //vTaskSuspend(NULL);
		 
		 port_pin_toggle_output_level(LED_ACTIVITY);

		 if (!trigger) {
			ramp_status = MOTOR_RAMP_UP;
			controller_motor->motor_status = MOTOR_STATUS_RUN;
			controller_motor->hall_callback();
		 } else {
			ramp_status = MOTOR_RAMP_DOWN;
		 }

		 calculate_ramp_delta();
		 new_duty = 0;

		 while(ramp_status) {
			 new_duty = getCurrentMotorValue() + ramp_delta;
			 if (ramp_status == MOTOR_RAMP_UP) {
				 if (new_duty > 0x07EE) {
					 new_duty = 0x07EE;
					 ramp_status = MOTOR_RAMP_IDLE;
				 }
				 controller_motor->setMotorDuty((uint16_t) new_duty);
				 port_pin_set_output_level(LED_WARNING, true);
			 } else if (ramp_status == MOTOR_RAMP_DOWN) {
				 if (new_duty < 0) {
					 new_duty = 0;
					 ramp_status = MOTOR_RAMP_IDLE;
					 controller_motor->motor_status = MOTOR_STATUS_BRAKE;
					 controller_motor->hall_callback();
				 }
				 controller_motor->setMotorDuty((uint16_t) new_duty);
				 port_pin_set_output_level(LED_ERROR, true);
			 } else ramp_status = MOTOR_RAMP_IDLE;

			 if(ramp_status) { 
				vTaskDelayUntil( &xLastWakeTime, 5 );	// 10ms
				xLastWakeTime = xTaskGetTickCount();
			 } else xLastWakeTime = xTaskGetTickCount() + 1500;
		 }
	 }
 }
