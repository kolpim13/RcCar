#include "car.h"

#include "hw_config.h"
#include "servo.h"
/*=============================================================*/

/* STATIC RESOURCES
    1. Servo Motor - For stearing purpose. 
*/
static ServoMotor_t stearing_servo = {
    .pwm_freq_hz = SERVO_LEDC_PWM_FREQ_HZ,
    .ledc_channel = SERVO_PWM_LEDC_CHANNEL,
    .min_pulse_width_us = SERVO_MIN_PULSE_WIDTH_US,
    .max_pulse_width_us = SERVO_MAX_PULSE_WIDTH_US,
    .min_angle_deg = SERVO_MIN_ANGLE_DEG,
    .max_angle_deg = SERVO_MAX_ANGLE_DEG,
};
/*=============================================================*/

static bool control_input_have_changed = false;
static uint16_t stearing_angle = 0;
/*=============================================================*/

void platform_init(void)
{
    /* Init control mechanism:
        Stearing servo --> ... 
    */
    servo_init(&stearing_servo, SERVO_LEDC_TIMER, SERVO_PWM_GPIO);
    platform_stearing_set(90);
}

void platform_stearing_set(uint16_t an)
{
    servo_set_angle(&stearing_servo, an);
    stearing_angle = an;
}

uint16_t platform_stearing_get(void)
{
    return stearing_angle;
}
/*=============================================================*/