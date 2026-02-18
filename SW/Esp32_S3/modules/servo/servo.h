#ifndef SERVO_H_
#define SERVO_H_

#include "hw_config.h"

typedef struct
{
    uint32_t pwm_freq_hz;
    ledc_channel_t ledc_channel;

    uint32_t min_pulse_width_us; 
    uint32_t max_pulse_width_us;
    uint32_t min_angle_deg;
    uint32_t max_angle_deg;

    uint32_t _pulse_width_range_us;
    uint32_t _angle_range_degree;
    uint32_t _pwm_period_us;
}ServoMotor_t;

void servo_init(ServoMotor_t* servo,
                ledc_timer_t timer, uint32_t gpio_mask);

void servo_set_angle(ServoMotor_t* servo, uint32_t angle);
void servo_set_pulse_width_us(ServoMotor_t* servo, uint32_t pulse_width_us);

#endif //SERVO_H_
