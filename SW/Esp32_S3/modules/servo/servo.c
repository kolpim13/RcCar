#include "servo.h"
/*=============================================================*/

void servo_init(ServoMotor_t* servo, 
                ledc_timer_t timer, uint32_t gpio)
{
    /* Configure PWM
       Timer --> LEDC channel. */
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = timer,
        .freq_hz = servo->pwm_freq_hz,
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ledc_channel = {
        .gpio_num = gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = servo->ledc_channel,
        .timer_sel = timer,
        .duty = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
    };
    ledc_channel_config(&ledc_channel);

    /* Computate some values for speed-up */
    servo->_pulse_width_range_us = servo->max_pulse_width_us - servo->min_pulse_width_us;
    servo->_angle_range_degree = servo->max_angle_deg - servo->min_angle_deg;
    servo->_pwm_period_us = 1000000 / servo->pwm_freq_hz;
}

void servo_set_angle(ServoMotor_t* servo, uint32_t angle)
{
    // Perform some checks
    if (angle < servo->min_angle_deg)
        angle = servo->min_angle_deg;

    if (angle > servo->max_angle_deg)
        angle = servo->max_angle_deg;

    // Calculate pulse width --> duty
    uint32_t pulse_width = servo->min_pulse_width_us + (servo->_pulse_width_range_us * angle) / servo->_angle_range_degree;
    uint32_t duty = (pulse_width * 8192) / servo->_pwm_period_us;
    
    // Set value
    ledc_set_duty(LEDC_LOW_SPEED_MODE, servo->ledc_channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, servo->ledc_channel);
}

void servo_set_pulse_width_us(ServoMotor_t* servo, uint32_t pulse_width_us)
{
    // Checks
    if (pulse_width_us < servo->min_pulse_width_us)
        pulse_width_us = servo->min_pulse_width_us;

    if (pulse_width_us > servo->max_pulse_width_us)
        pulse_width_us = servo->max_pulse_width_us;

    // Calculate duty --> set value
    uint32_t duty = (pulse_width_us * 8192) / servo->_pwm_period_us;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, servo->ledc_channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, servo->ledc_channel);
}

/*=============================================================*/
