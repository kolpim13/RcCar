#ifndef HW_CONFIG_H_
#define HW_CONFIG_H_

#include <stdint.h>

#include <driver/gpio.h>
#include <driver/ledc.h>

/* GENERAL */

#define DISABLE     0
#define ENABLE      1

#define GPIO_NO     0
/*=============================================================*/

/* MOTOR DRIVER TB6612
    1. Main motor
        * Channel A is used for main motor.
        * Channel B is not used at the moment.
        * For both channels, timer 0 is used.
*/

#define TB6612_AIN1_GPIO                GPIO_NUM_4    
#define TB6612_AIN2_GPIO                GPIO_NUM_5
#define TB6612_PWMA_GPIO                GPIO_NUM_6
#define TB6612_AIN1_GPIO_MASK           (1UL << TB6612_AIN1_GPIO)    
#define TB6612_AIN2_GPIO_MASK           (1UL << TB6612_AIN2_GPIO)  
#define TB6612_PWMA_GPIO_MASK           (1UL << TB6612_PWMA_GPIO)  
#define TB6612_PWMA_LEDC_CHANNEL        LEDC_CHANNEL_0

#define TB6612_BIN1_GPIO                GPIO_NO
#define TB6612_BIN2_GPIO                GPIO_NO
#define TB6612_PWMB_GPIO                GPIO_NO
#define TB6612_PWMB_LEDC_CHANNEL        LEDC_CHANNEL_MAX

#define TB6612_LEDC_TIMER               LEDC_TIMER_0
#define TB6612_LEDC_PWM_FREQ_HZ         20000
/*=============================================================*/

/* SERVO MOTORS
    1. Servo for turning front axle

*/

#define SERVO_MIN_PULSE_WIDTH_US        1000UL
#define SERVO_MAX_PULSE_WIDTH_US        2000UL
#define SERVO_MIN_ANGLE_DEG             0UL
#define SERVO_MAX_ANGLE_DEG             180UL

#define SERVO_PWM_GPIO                  GPIO_NUM_7
#define SERVO_PWM_GPIO_MASK             (1UL << SERVO_PWM_GPIO)
#define SERVO_PWM_LEDC_CHANNEL          LEDC_CHANNEL_2
#define SERVO_LEDC_TIMER                LEDC_TIMER_0
#define SERVO_LEDC_PWM_FREQ_HZ          50             
/*=============================================================*/

#endif //HW_CONFIG_H_
