/*
 * zpdc_can.cpp
 *
 * Created: 6/5/2017 10:57:13 AM
 *  Author: Andres Vasquez
 */ 
 #include <asf.h>

 can_service::can_service(CanConfiguration_CAN0 config, ZpdcSystem *system_module) {
		// ISO C++ Compliance
	standard_receive_index = 0; 
	extended_receive_index = 0;
	network_size = 0;

	queue_net_devices = xQueueCreate(10, sizeof(uint32_t));

	system_data = system_module;

		// CAN Module Setup
	struct system_pinmux_config can_pin_config;
	system_pinmux_get_config_defaults(&can_pin_config);
	can_pin_config.mux_position = config.tx_mux;
	system_pinmux_pin_set_config(config.tx_pin, &can_pin_config);
	can_pin_config.mux_position = config.rx_mux;
	system_pinmux_pin_set_config(config.rx_pin, &can_pin_config);

	struct can_config config_can;
	can_get_config_defaults(&config_can);
	config_can.nonmatching_frames_action_standard = config.nonmatching_action;

	can_init(&can0_instance, config.hardware, &config_can);
	can_start(&can0_instance);
	
	system_interrupt_enable(config.interrupt_vector);
	can_enable_interrupt(&can0_instance, CAN_RX_FIFO_0_NEW_MESSAGE);
	can_enable_interrupt(&can0_instance, CAN_TX_EVENT_FIFO_NEW_ENTRY);

	/*can_enable_interrupt(&can0_instance, CAN_TX_EVENT_FIFO_ELEMENT_LOST);
	can_enable_interrupt(&can0_instance, CAN_MESSAGE_RAM_ACCESS_FAILURE);
	can_enable_interrupt(&can0_instance, CAN_TIMEOUT_OCCURRED);
	can_enable_interrupt(&can0_instance, CAN_BIT_ERROR_CORRECTED);
	can_enable_interrupt(&can0_instance, CAN_BIT_ERROR_UNCORRECTED);
	can_enable_interrupt(&can0_instance, CAN_ERROR_LOGGING_OVERFLOW);
	can_enable_interrupt(&can0_instance, CAN_ERROR_PASSIVE);
	can_enable_interrupt(&can0_instance, CAN_PROTOCOL_ERROR_ARBITRATION);
	can_enable_interrupt(&can0_instance, CAN_PROTOCOL_ERROR_DATA);*/

	t_init("CAN", 1, 2);	// Task-Thread Initialization
 }

 void can_service::task(void) {
	uint32_t queue_item = 0;

	//port_pin_set_output_level(LED_ERROR, true);

	for(;;) {
		if (xQueueReceive(system_data->queue_to_can, &queue_item, portMAX_DELAY)) {
			switch (system_data->get_queue_entry_parameter(queue_item, 4)) {
				case CAN_DISCOVERY_REQUEST: 
				break;
				default:
				break;
			}
		}
	}
 }

 void can_service::callback(void) {
	BaseType_t xHigherPriorityWoken = pdFALSE;
	uint32_t status = can_read_interrupt_status(&can0_instance);
	bool is_can_to_send = false;

	if (!(status & CAN_RX_FIFO_0_NEW_MESSAGE) && !(status & CAN_TX_EVENT_FIFO_NEW_ENTRY)) {
		can_clear_interrupt_status(&can0_instance, CAN_RX_FIFO_0_NEW_MESSAGE);
	}

	is_can_to_send = false;
	if (status & CAN_RX_FIFO_0_NEW_MESSAGE) {
		can_clear_interrupt_status(&can0_instance, CAN_RX_FIFO_0_NEW_MESSAGE);

		can_get_rx_fifo_0_element(&can0_instance, &rx_element_fifo_0, standard_receive_index++);
		if (standard_receive_index == CONF_CAN0_RX_BUFFER_NUM) standard_receive_index = 0;

		uint8_t sub_net = CAN_RX_FIFO_ID_SUBNET(rx_element_fifo_0.R0.reg);
		if(sub_net == CAN_SUBNET_NETWORK_REQUEST) {
			switch(rx_element_fifo_0.data[0]) {
				case CAN_ORDER_UPDATE_REQUEST:
					if (rx_element_fifo_0.data[1] == system_data->get_uid_high() && rx_element_fifo_0.data[2] == system_data->get_uid_low())
						{ system_data->set_address(rx_element_fifo_0.data[3]); system_data->set_role(rx_element_fifo_0.data[4]); is_can_to_send = true; }
					tx_message_0[0] = CAN_ORDER_UPDATE_RETURN;
				case CAN_DISCOVERY_REQUEST:
					if (rx_element_fifo_0.data[0] == CAN_DISCOVERY_REQUEST) { tx_message_0[0] = CAN_DISCOVERY_RETURN; is_can_to_send = true; }
					tx_message_0[1] = system_data->get_uid_high();
					tx_message_0[2] = system_data->get_uid_low();
					tx_message_0[3] = system_data->get_address();
					if(is_can_to_send) send(4, system_data->get_role(), CAN_SUBNET_NETWORK_REQUEST, CAN_BUFFER_0);
				break;
				case CAN_REQUEST_LED_TOG:
					if (rx_element_fifo_0.data[1] == system_data->get_uid_high() && rx_element_fifo_0.data[2] == system_data->get_uid_low()) {
						port_pin_toggle_output_level(LED_ACTIVITY);
						tx_message_0[0] = CAN_REQUEST_LED_TOG_RETURN;
						tx_message_0[1] = system_data->get_uid_high();
						tx_message_0[2] = system_data->get_uid_low();
						tx_message_0[3] = (port_pin_get_output_level(LED_ACTIVITY) ? 0x01 : 0x00);
						send(4, system_data->get_role(), CAN_SUBNET_NETWORK_REQUEST, CAN_BUFFER_0);
					}
				break;
				case CAN_MOTOR_GET_PARAMA:
					if ((rx_element_fifo_0.data[1] == system_data->get_uid_high() && rx_element_fifo_0.data[2] == system_data->get_uid_low()) ||
						(rx_element_fifo_0.data[1] + rx_element_fifo_0.data[2] == 0)) {	
						tx_message_0[0] = CAN_MOTOR_GET_PARAMA_RETURN;
						tx_message_0[1] = system_data->get_uid_high();
						tx_message_0[2] = system_data->get_uid_low();
						tx_message_0[3] = (uint8_t)(motor->ramp.ramp_duration >> 8);
						tx_message_0[4] = (uint8_t)(motor->ramp.ramp_duration & 0x00FF);
						tx_message_0[5] = (uint8_t)(motor->ramp.target_speed >> 8);
						tx_message_0[6] = (uint8_t)(motor->ramp.target_speed & 0x00FF);
						tx_message_0[7] = (motor->motor_direction ? 0x01 : 0x00);
						send(8, system_data->get_role(), CAN_SUBNET_NETWORK_REQUEST, CAN_BUFFER_0);
					}
				break;
				case CAN_MOTOR_START:
					if ((rx_element_fifo_0.data[1] == system_data->get_uid_high() && rx_element_fifo_0.data[2] == system_data->get_uid_low()) ||
						(rx_element_fifo_0.data[1] + rx_element_fifo_0.data[2] == 0)) {
						if (!motor->isMotorRunning())
							motor->setMotorRunning(true);
						tx_message_0[0] = CAN_MOTOR_START_RETURN;
					}
				case CAN_MOTOR_STOP:
					if ((rx_element_fifo_0.data[1] == system_data->get_uid_high() && rx_element_fifo_0.data[2] == system_data->get_uid_low()) ||
						(rx_element_fifo_0.data[1] + rx_element_fifo_0.data[2] == 0)) {
						if (rx_element_fifo_0.data[0] == CAN_MOTOR_STOP) {
							if (motor->isMotorRunning())
								motor->setMotorRunning(false);
						}
						xHigherPriorityWoken = xTaskResumeFromISR(motor->getRampTaskHandle());
						if (rx_element_fifo_0.data[0] == CAN_MOTOR_STOP) 
							tx_message_0[0] = CAN_MOTOR_STOP_RETURN;
						tx_message_0[1] = system_data->get_uid_high();
						tx_message_0[2] = system_data->get_uid_low();
						tx_message_0[3] = (motor->isMotorRunning() ? 0x01 : 0x00);
						send(4, system_data->get_role(), CAN_SUBNET_NETWORK_REQUEST, CAN_BUFFER_0);
					}
				break;
				case CAN_MOTOR_PID_TUNE:
					if ((rx_element_fifo_0.data[1] == system_data->get_address()) || (rx_element_fifo_0.data[1] == 0)) {
						tx_message_0[0] = CAN_MOTOR_PID_TUNE_RETURN;
						tx_message_0[1] = system_data->get_address();
						if(!motor->isMotorRunning()) {
							system_data->store_pid_values(&rx_element_fifo_0.data[2], 
														  &rx_element_fifo_0.data[4], 
														  &rx_element_fifo_0.data[6]);
							system_data->read_pid_values(&(motor->pid_instance.kp), &(motor->pid_instance.ki), &(motor->pid_instance.kd));
							tx_message_0[2] = 1;	// OK
						} else tx_message_0[2] = 0;	// FAIL
						send(3, system_data->get_role(), CAN_SUBNET_NETWORK_REQUEST, CAN_BUFFER_0);
					}
				break;
				case CAN_MOTOR_PARAM_A:
					if ((rx_element_fifo_0.data[1] == system_data->get_address()) || (rx_element_fifo_0.data[1] == 0)) {
						tx_message_0[0] = CAN_MOTOR_PARAM_A_RETURN;
						tx_message_0[1] = system_data->get_address();
						if(!motor->isMotorRunning()) {
							system_data->store_ramp_values(&rx_element_fifo_0.data[2],
														   &rx_element_fifo_0.data[4]);
							system_data->read_ramp_values(&(motor->ramp.ramp_duration),&(motor->ramp.target_speed));
							system_data->set_boolean_subpage_bit(4, 0, (rx_element_fifo_0.data[7] > 0 ? true : false));
							motor->motor_direction = system_data->get_boolean_subpage_bit(4, 0);
							tx_message_0[2] = 1;	// OK
						} else tx_message_0[2] = 0;	// FAIL
						send(3, system_data->get_role(), CAN_SUBNET_NETWORK_REQUEST, CAN_BUFFER_0);
					}
				break;
				case CAN_MOTOR_PARAM_B:	// TODO: This is an expansion slot for 
					if ((rx_element_fifo_0.data[1] == system_data->get_address()) || (rx_element_fifo_0.data[1] == 0)) {
						tx_message_0[0] = CAN_MOTOR_PARAM_B_RETURN;
						tx_message_0[1] = system_data->get_address();
						if(!motor->isMotorRunning()) {
							tx_message_0[2] = 1;	// OK
						} else tx_message_0[2] = 0;	// FAIL
						send(3, system_data->get_role(), CAN_SUBNET_NETWORK_REQUEST, CAN_BUFFER_0);
					}
				break;
			}	
		}
	}

	if (status & CAN_TX_EVENT_FIFO_NEW_ENTRY) {
		can_clear_interrupt_status(&can0_instance, CAN_TX_EVENT_FIFO_NEW_ENTRY);
	}

	if (status & CAN_TX_EVENT_FIFO_FULL) {
		can_clear_interrupt_status(&can0_instance, CAN_TX_EVENT_FIFO_FULL);
		can_tx_event_fifo_acknowledge(&can0_instance, CAN_TX_EVENT_FIFO_FULL);
	}

	portYIELD_FROM_ISR(xHigherPriorityWoken);
 }

 void can_service::send(uint8_t length, uint8_t device, uint8_t sub_net, uint8_t buffer) {
	struct can_tx_element tx_element;
	can_get_tx_buffer_element_defaults(&tx_element);

	tx_element.T0.reg |= CAN_TX_ELEMENT_T0_STANDARD_ID((device << 9) | (sub_net << 7) | ((uint8_t) ((system_data->get_uid() >> 1) & 0x7F)));
	tx_element.T1.bit.DLC = (uint32_t)length;
	if (buffer == CAN_BUFFER_0) for (uint8_t i=0; i<length; i++) tx_element.data[i] = tx_message_0[i];
	else for (uint8_t i=0; i<length; i++) tx_element.data[i] = tx_message_1[i];
	can_set_tx_buffer_element(&can0_instance, &tx_element, (uint32_t)buffer);
	can_tx_transfer_request(&can0_instance, (uint32_t)(1 << buffer));
	//if (can_tx_transfer_request(&can0_instance, (uint32_t)(1 << buffer)) == STATUS_OK) port_pin_set_output_level(PIN_PA07, true);
 }
