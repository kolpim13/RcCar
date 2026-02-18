#include "TB6612FNG.h"
#include "hw_config.h"

void TB6612FNG_Init(TB6612FNG_t* bridge)
{
    /* Confiure discrete pins */
    gpio_config_t pins_conf = {
        .pin_bit_mask = bridge->AIN1 | bridge->AIN2,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&pins_conf);

    /* Configure PWM
       Timer --> LEDC channel. */
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = TB6612_LEDC_TIMER,
        .freq_hz = TB6612_LEDC_PWM_FREQ_HZ,
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ledc_channel = {
        .gpio_num = bridge->PWMA,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = TB6612_PWMA_LEDC_CHANNEL,
        .timer_sel = TB6612_LEDC_TIMER,
        .duty = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
    };
    ledc_channel_config(&ledc_channel);
}
