#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"

#include "esp_system.h"
#include "nvs_flash.h"

#include "ble.h"

#include "hw_config.h"
#include "TB6612FNG.h"
#include "servo.h"
/*=============================================================*/

/* STATIC RESOURCES */
static TB6612FNG_t motor_driver = {
    .AIN1 = TB6612_AIN1_GPIO_MASK,
    .AIN2 = TB6612_AIN2_GPIO_MASK,
    .PWMA = TB6612_PWMA_GPIO_MASK,
    .BIN1 = GPIO_NO,
    .BIN2 = GPIO_NO,
    .PWMB = GPIO_NO,
};

static ServoMotor_t servo = {
    .pwm_freq_hz = SERVO_LEDC_PWM_FREQ_HZ,
    .ledc_channel = SERVO_PWM_LEDC_CHANNEL,
    .min_pulse_width_us = SERVO_MIN_PULSE_WIDTH_US,
    .max_pulse_width_us = SERVO_MAX_PULSE_WIDTH_US,
    .min_angle_deg = SERVO_MIN_ANGLE_DEG,
    .max_angle_deg = SERVO_MAX_ANGLE_DEG,
};
/*=============================================================*/

/* FreeRTOS
    Tasks:
        1. [Core 1, 4096, High Priority] Main - General behaviour logic.  
        Reaction on user`s input and sensors information.
        Triggers updates that should be done in another tasks.
        2. [Core 1, 4096, Low Priority] Cyclic_10ms - ...
*/

#define CORE_0  0
#define CORE_1  1

static TaskHandle_t Task_Handle_Main;           //Assigned to Core 1
static TaskHandle_t Task_Handle_Cyclic_10ms;    //Assigned to Core 1

static void Task_Main(void *params);
static void Task_Cyclic_10ms(void *params);
/*=============================================================*/

void app_main(void)
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // BLE Stack
    ble_initialize();

    // Motor driver
    //TB6612FNG_Init(&motor_driver);

    // Servo motor
    servo_init(&servo, SERVO_LEDC_TIMER, SERVO_PWM_GPIO);

    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(
        Task_Main,
        "Main 10ms",
        4096,
        NULL,
        5,
        &Task_Handle_Cyclic_10ms,
        CORE_1
    );

    xTaskCreatePinnedToCore(
        Task_Cyclic_10ms,
        "Cyclic 10ms",
        4096,
        NULL,
        0,
        &Task_Handle_Cyclic_10ms,
        CORE_1
    );
}
/*=============================================================*/

/* FreeRTOS TASKS IMPLEMENTATION
*/

static void Task_Main(void *params)
{
    (void)params;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
        xTaskDelayUntil(&xLastWakeTime, (TickType_t)10);
    }
}

static void Task_Cyclic_10ms(void *params)
{
    (void)params;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
        xTaskDelayUntil(&xLastWakeTime, (TickType_t)10);
    }
}
/*=============================================================*/
