/*
 * zpdc_system.h
 *
 * Created: 6/5/2017 10:56:56 AM
 *  Author: Andres Vasquez
 */ 


#ifndef ZPDC_SYSTEM_H_
#define ZPDC_SYSTEM_H_
#ifdef __cplusplus

#define LED_ACTIVITY	PIN_PA17
#define LED_WARNING		PIN_PA18
#define LED_ERROR		PIN_PA19

void extint_callback_wrapper(void);
void speed_measurement_timer_callback(struct tc_module *const module_inst);
void pid_loop_timer_callback(struct tc_module *const module_inst);

/* System Data */
class ZpdcSystem {
	public:
	const uint32_t* const uniqueness_A = reinterpret_cast<uint32_t *>(0x0080A00C);
	const uint32_t* const uniqueness_B = reinterpret_cast<uint32_t *>(0x0080A040);
	const uint32_t* const uniqueness_C = reinterpret_cast<uint32_t *>(0x0080A044);
	const uint32_t* const uniqueness_D = reinterpret_cast<uint32_t *>(0x0080A048);
	
	ZpdcSystem() {
		/* ---- Initialize the UID ---- */
		uid = 0;
		net_address = 0xFF;
		enum status_code result_code = eeprom_emulator_init();

		if ( result_code == STATUS_ERR_NO_MEMORY ) while (true) {  }
		else if ( result_code == STATUS_ERR_BAD_FORMAT ) {
			while ( result_code == STATUS_ERR_BAD_FORMAT ) {
				eeprom_emulator_erase_memory();
				result_code = eeprom_emulator_init();
				if ( result_code != STATUS_ERR_BAD_FORMAT ) uid_setup();
			}
		} else if ( result_code == STATUS_OK ) {
			while ((uid = read_uid()) == 0xFFFF ) uid_setup();
			while (read_boolean_subpage(4) & 0x80) setup_boolean_subpage(4);
			net_address = read_address();
			role = read_role();
		} else while (true) {  }	// LOCK if EEPROM error

		queue_to_can = xQueueCreate(1, sizeof(uint32_t) );

		struct port_config pin_output;
		port_get_config_defaults(&pin_output);
		pin_output.direction = PORT_PIN_DIR_OUTPUT;
		port_pin_set_config(LED_WARNING, &pin_output);
		port_pin_set_config(LED_ERROR, &pin_output);
		port_pin_set_config(LED_ACTIVITY, &pin_output);
	}

		// ETHERNET - CAN Queue
		QueueHandle_t queue_to_can;

		inline uint8_t get_queue_entry_parameter(uint32_t parm, uint8_t parm_num) {
			return (uint8_t)((parm >> ((parm_num-1) * 8)) & 0xFF);
		}

		uint32_t get_queue_parameter_value(uint8_t cmd, uint8_t p3, uint8_t p2, uint8_t p1) {
			uint32_t hold = (uint32_t) (cmd << 24);	// Push the command

			if (p3 != 0) hold |= (uint32_t) (p3 << 16);
			if (p2 != 0) hold |= (uint32_t) (p2 << 8);
			if (p1 != 0) hold |= (uint32_t) (p3);

			return hold;
		}

		uint16_t get_uid(void) { return uid; }
		uint8_t get_uid_high(void) { return (uint8_t)(uid >> 8); }
		uint8_t get_uid_low(void) { return (uint8_t)(uid & 0xFF); }

		uint8_t get_address(void) { return net_address; }
		uint8_t get_role(void) { return role; }
		void set_address(uint8_t order) {
			uint8_t page_data[EEPROM_PAGE_SIZE];
			net_address = order;
			eeprom_emulator_read_page(0, page_data);
			page_data[2] = order;
			eeprom_emulator_write_page(0,page_data);
			eeprom_emulator_commit_page_buffer();	// END UID EEPROM STORAGE
		}
		void set_role(uint8_t p_role) {
			uint8_t page_data[EEPROM_PAGE_SIZE];
			role = p_role;
			eeprom_emulator_read_page(0, page_data);
			page_data[3] = p_role;
			eeprom_emulator_write_page(0,page_data);
			eeprom_emulator_commit_page_buffer();	// END UID EEPROM STORAGE
		}
		void set_boolean_subpage_bit(uint8_t subpage, uint8_t position, bool value) {
			uint8_t page_data[EEPROM_PAGE_SIZE];
			eeprom_emulator_read_page(0, page_data);
			page_data[subpage] = (value ? (page_data[subpage] | (1 << position)) : (page_data[subpage] & ~(1 << position)) );
			eeprom_emulator_write_page(0,page_data);
			eeprom_emulator_commit_page_buffer();	// END UID EEPROM STORAGE
		}
		bool get_boolean_subpage_bit(uint8_t subpage, uint8_t position) {
			uint8_t page_data[EEPROM_PAGE_SIZE];
			eeprom_emulator_read_page(0, page_data);
			if (page_data[subpage] & (1 << position)) return true;
			else return false;
		}
private:
	uint16_t uid;
	uint8_t net_address;
	uint8_t role;

		// Remap a pseudo-unique ID by hashing a 128-bit value to 16-bit
		void uid_setup(void) {
			volatile uint32_t id_int = 0;
			uid = 0;
			id_int += (*uniqueness_A) * 3;
			id_int += (*uniqueness_B) * 3;
			//uid = (uint16_t)((id_int % 0x0000FFFF));	// END UID HASH
			//id_int = 0;
			id_int += (*uniqueness_C) * 3;
			id_int += (*uniqueness_D) * 3;
			uid |= (uint16_t)((id_int % 0x0000FFFF));	// END UID HASH

			uint8_t page_data[EEPROM_PAGE_SIZE];
			eeprom_emulator_read_page(0, page_data);
			page_data[0] = (uint8_t)(uid & 0x00FF);
			page_data[1] = (uint8_t)(uid >> 8);
			page_data[3] = 0x03;
			eeprom_emulator_write_page(0,page_data);
			eeprom_emulator_commit_page_buffer();	// END UID EEPROM STORAGE
		}
		uint16_t read_uid(void) {
			uint8_t page_data[EEPROM_PAGE_SIZE];
			eeprom_emulator_read_page(0, page_data);
			return ((uint16_t)page_data[1] << 8) | (uint16_t)page_data[0];
		}
		uint8_t read_address(void) {
			uint8_t page_data[EEPROM_PAGE_SIZE];
			eeprom_emulator_read_page(0, page_data);
			return (uint8_t)page_data[2];
		}
		uint8_t read_role(void) {
			uint8_t page_data[EEPROM_PAGE_SIZE];
			eeprom_emulator_read_page(0, page_data);
			return (uint8_t)page_data[3];
		}
		uint8_t read_boolean_subpage(uint8_t subpage) {
			uint8_t page_data[EEPROM_PAGE_SIZE];
			eeprom_emulator_read_page(0, page_data);
			return (uint8_t)page_data[subpage];
		}
		void setup_boolean_subpage(uint8_t subpage) {
			uint8_t page_data[EEPROM_PAGE_SIZE];
			eeprom_emulator_read_page(0, page_data);
			page_data[subpage] = 0;
			eeprom_emulator_write_page(0,page_data);
			eeprom_emulator_commit_page_buffer();	// END UID EEPROM STORAGE
		}
	};

	#endif // __cplusplus
	#endif /* ZPDC_SYSTEM_H_ */