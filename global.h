#ifndef _GLOBAL_H
#define _GLOBAL_H
#include <stdint.h>
#include <stddef.h>
#include "main.h"
#include "CAN_receive.h"
#include "timers.h"
//chassis_behaviiour.c
#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "arm_math.h"

#include "gimbal_behaviour.h"

#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
#include "detect_task.h"

#include "chassis_task.h"
#include "chassis_behaviour.h"

#include "cmsis_os.h"
//chassis_task.c
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"

//gimbal_behaviour.c
#include "gimbal_behaviour.h"
#include "arm_math.h"
#include "bsp_buzzer.h"
#include "detect_task.h"

#include "user_lib.h"
//gimbal_task.c
#include "gimbal_task.h"

#include "main.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "shoot.h"
#include "pid.h"
//remote_control.c
#include "remote_control.h"

#include "main.h"

#include "bsp_usart.h"
#include "string.h"

#include "detect_task.h"
//BMI088.driver.c
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"
//BMI088Middleware.c
#include "BMI088Middleware.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_delay.h"
//ist8310driver.c
#include "ist8310driver.h"
#include "ist8310driver_middleware.h"
//ist8310driver_middleware.c
#include "ist8310driver_middleware.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_delay.h"
#include "bsp_i2c.h"


#endif /*_GLOBAL_H */