/*
 * zpdc_can.h
 *
 * Created: 6/5/2017 10:55:43 AM
 *  Author: Andres Vasquez
 */ 

#ifndef ZPDC_CAN_H_
#define ZPDC_CAN_H_
#ifdef __cplusplus

/************************************************************************/
/*  TYPE DEFINITIONS FOR NETWORK DEVICE TABLE                           */
/************************************************************************/
#define CAN_DISCOVERY_REQUEST		(uint8_t)('A')
#define CAN_ORDER_UPDATE_REQUEST	(uint8_t)('B')
#define CAN_REQUEST_LED_TOG			(uint8_t)('C')
// Return Values
#define CAN_DISCOVERY_RETURN		(uint8_t)('a')
#define CAN_ORDER_UPDATE_RETURN		(uint8_t)('b')
#define CAN_REQUEST_LED_TOG_RETURN	(uint8_t)('c')
	/********************************************************************/
#define CAN_SUBNET_NETWORK_REQUEST	((uint8_t) 0)
#define CAN_SUBNET_PARAMETER_SETUP	((uint8_t) 2)
#define CAN_SUBNET_ZPDC_OPERATION	((uint8_t) 3)

#define CAN_DEVICE_GATEWAY			((uint8_t) 0)
#define CAN_DEVICE_HYBRID			((uint8_t) 2)
#define CAN_DEVICE_DRIVE_CARD		((uint8_t) 3)

#define CAN_BUFFER_0				((uint8_t) 0)
#define CAN_BUFFER_1				((uint8_t) 1)

#define CAN_RX_FIFO_ID_SUBNET(value)	(uint8_t)((0x03ul & ((value) >> (CAN_TX_ELEMENT_T0_STANDARD_ID_Pos + 7))))
#define CAN_RX_FIFO_ID_DEVICE(value)	(uint8_t)((0x03ul & ((value) >> (CAN_TX_ELEMENT_T0_STANDARD_ID_Pos + 9))))

/************************************************************************/
/*  TYPE DEFINITIONS FOR NETWORK DEVICE TABLE                           */
/************************************************************************/
typedef struct {
	uint16_t uid;
	uint8_t net_address;
} NetworkDevices;

/************************************************************************/
/*  HARDWARE MODULE CONSTANT STRUCTURES (ONLY 1 CAN MODULE IN C21)      */
/************************************************************************/
static const struct CanConfiguration_CAN0 {
	const static constexpr char *NAME = "CAN0\n";

	Can *hardware = CAN0;

	/* CAN Module Setup Parameters */
	enum can_nonmatching_frames_action nonmatching_action = CAN_NONMATCHING_FRAMES_FIFO_0;
	const enum system_interrupt_vector interrupt_vector = SYSTEM_INTERRUPT_MODULE_CAN0;
	const uint8_t tx_pin = PIN_PA24G_CAN0_TX;
	const uint8_t tx_mux = MUX_PA24G_CAN0_TX;
	const uint8_t rx_pin = PIN_PA25G_CAN0_RX;
	const uint8_t rx_mux = MUX_PA25G_CAN0_RX;

} zpdc_can0_configuration;
static can_module can0_instance;

/************************************************************************/
/* CAN MODULE OBJECT ABSTRACTION                                        */
/************************************************************************/

class can_service : public Task {
public:
	const constexpr static char* NAME = "CAN_BUS\0";

	can_service(CanConfiguration_CAN0 config, ZpdcSystem *system_module);

	void send(uint8_t length, uint8_t device, uint8_t sub_net, uint8_t buffer);

	void task(void);

	void callback(void);
private:
	NetworkDevices network_devices[100];
	uint8_t network_size;
	QueueHandle_t queue_net_devices;
	
	struct can_rx_element_fifo_0 rx_element_fifo_0;
	struct can_rx_element_fifo_0 rx_element_fifo_1;
	struct can_rx_element_fifo_0 rx_element_buffer;

	// Data Variables
		// TX
	uint8_t tx_message_0[CONF_CAN_ELEMENT_DATA_SIZE];
	uint8_t tx_message_1[CONF_CAN_ELEMENT_DATA_SIZE];
		// RX
	volatile uint8_t standard_receive_index;
	volatile uint8_t extended_receive_index;

	// Data Interfaces
	
	// System Data
	ZpdcSystem *system_data;

	
	inline void CancelTransmission(uint32_t buffer) { can_tx_cancel_request(&can0_instance, (uint32_t)(1 << buffer)); }
};



#endif
#endif /* ZPDC_CAN_H_ */
