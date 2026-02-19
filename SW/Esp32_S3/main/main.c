#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"

#include "esp_system.h"
#include "nvs_flash.h"

#include "ble.h"
#include "car.h"
/*=============================================================*/

/* STATIC RESOURCES */
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

    // Initialize whole platform
    platform_init();

    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(
        Task_Main,
        "Main 10ms",
        4096,
        NULL,
        5,
        &Task_Handle_Main,
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
