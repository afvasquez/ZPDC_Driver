/*
 * ZPDC_Driver.cpp
 *
 * Created: 5/11/2017 9:37:54 AM
 * Author : Andres Vasquez
 */ 
#include <asf.h>

can_service *Can_Object;
motor_controller *Motor_Object;

int main(void)
{
    /* Initialize the SAM system */
    system_init();
	ZpdcSystem system_data;

	can_service can_obj(zpdc_can0_configuration, &system_data);
	Can_Object = &can_obj;

	motor_controller bldc_motor(&system_data, &motor_one_parameters);
	Motor_Object = &bldc_motor;

    /* Replace with your application code */
    vTaskStartScheduler();
}

void CAN0_Handler(void) {
	Can_Object->callback();
}

void extint_callback_wrapper(void) {
	Motor_Object->hall_callback();
}
void speed_measurement_timer_callback(struct tc_module *const module_inst) {
	Motor_Object->speed_sensor.timer_callback();
}
void pid_loop_timer_callback(struct tc_module *const module_inst) {
	Motor_Object->pid_instance.timer_callback();
}
