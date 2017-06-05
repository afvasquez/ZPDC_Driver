/*
 * zpdc_oop_task.h
 *
 * Created: 6/5/2017 10:56:15 AM
 *  Author: Andres Vasquez
 */ 


#ifndef ZPDC_OOP_TASK_H_
#define ZPDC_OOP_TASK_H_
#ifdef __cplusplus

class Task {
	public:
	BaseType_t t_init(const char* name, uint8_t priority, uint8_t stack_multiplier) {
		return xTaskCreate(
		&taskfun,
		name,
		configMINIMAL_STACK_SIZE * stack_multiplier,
		this,
		tskIDLE_PRIORITY + priority,
		&handle);
	}

	virtual void task(void) =0;
	static void taskfun(void *parm) {
		((Task*)parm)->task();
		vTaskDelete(NULL);
	}

	TaskHandle_t handle;
};

#endif // __cplusplus
#endif /* ZPDC_OOP_TASK_H_ */