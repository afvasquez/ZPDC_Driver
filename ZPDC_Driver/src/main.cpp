/*
 * ZPDC_Driver.cpp
 *
 * Created: 5/11/2017 9:37:54 AM
 * Author : Andres Vasquez
 */ 
#include <asf.h>

can_service *Object;

int main(void)
{
    /* Initialize the SAM system */
    system_init();

	ZpdcSystem system_data;

	can_service can_obj(zpdc_can0_configuration, &system_data);
	Object = &can_obj;

    /* Replace with your application code */
    vTaskStartScheduler();
}

void CAN0_Handler(void) {
	Object->callback();
}
