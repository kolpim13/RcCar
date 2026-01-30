#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "sdkconfig.h"
/*=============================================================*/

// For debugging purpose
static const char *BLE_TAG = "BLE";
/*=============================================================*/

/* APPLICATION PROFILE 
    * SERVICE 1 - CONTROL (CTRL)
        * CHARACTERISTIC 1 - SPEED [] [INT16] 
*/

#define DEVICE_NAME         "RioCar"
#define DEVICE_PROFILE_NUM  1           // Number of profiles available for this device - 1. 
#define DEVICE_APP_ID       0           // 

#define UUID_LEN            16

// Randomly generated UUID: eb029cf2-7afe-4617-94b2-1ce44867b0fc
#define SVC_CTRL_BASE\      
    0xEB, 0x02, 0x9C, 0xF2,\
    0x7A, 0xFE,\
    0x46, 0x17,\
    0x94, 0xB2,\
    0x1C, 0xE4, 0x48, 0x67, 0xB0
static const uint8_t SVC_CTRL_UUID[UUID_LEN]                = { SVC_CTRL_BASE, 0x00 };
static const uint8_t SVC_CTRL_CHAR_POWER_UUID[UUID_LEN]     = { SVC_CTRL_BASE, 0x01 };

// User Description Descriptor UUID
static const esp_bt_uuid_t user_desc_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = { .uuid16 = ESP_GATT_UUID_CHAR_DESCRIPTION }  // 0x2901
};
static const char SVC_CTRL_CHAR_POWER_DESC[]  = "Power";

// Attribute handlers
static uint16_t svc_ctrl_handle                     = 0;
static uint16_t svc_ctrl_char_power_handle          = 0;
static uint16_t svc_ctrl_char_power_cccd_handle     = 0;    // Descriptor (Not used for now)

// Connection related
static esp_gatt_if_t g_gatts_if = ESP_GATT_IF_NONE;
static uint16_t g_conn_id = 0;
static bool g_connected = false;

// Test values to read/write - to be moved to another files (where they belong)
static uint16_t power_value = 101; 
/*=============================================================*/

/* GAP
    * Basic settings
    * Advertising settings and payload 
*/

#define ADV_INT_PARAMS_MIN  0x20  // 0x20 * 0.625 ms [unit] = 12.5 ms.
#define ADV_INT_PARAMS_MAX  0x40  // 0x40 * 0.625 ms [unit] = 25 ms.
#define ADV_INT_DATA_MIN    0x10  // 0x10 * 1.25 ms [unit] = 12.5 ms.
#define ADV_INT_DATA_MAX    0x20  // 0x20 * 1.25 ms [unit] = 25 ms.

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = ADV_INT_PARAMS_MIN, 
    .adv_int_max        = ADV_INT_PARAMS_MAX, 
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_adv_data_t adv_data_svc_ctrl = {
    .set_scan_rsp        = false,
    .include_name        = true,   // include "RioCar" in advertising packet
    .include_txpower     = true,
    .min_interval        = ADV_INT_DATA_MIN,
    .max_interval        = ADV_INT_DATA_MAX,
    .appearance          = 0x00,   // no standard appearance
    .manufacturer_len    = 0,
    .p_manufacturer_data = NULL,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(SVC_CTRL_UUID),
    .p_service_uuid      = (uint8_t*)SVC_CTRL_UUID,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, 
                              esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            /* Advertising data configured; start advertising now. */
            ESP_LOGI(BLE_TAG, "GAP: Adv data set -> start advertising");
            esp_ble_gap_start_advertising(&adv_params);
            break;

        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            ESP_LOGI(BLE_TAG, "GAP: Advertising start %s",
                    (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) ? "OK" : "FAIL");
            break;

        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            ESP_LOGI(BLE_TAG, "GAP: Advertising stop %s",
                    (param->adv_stop_cmpl.status == ESP_BT_STATUS_SUCCESS) ? "OK" : "FAIL");
            break;

        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            /* This event reports negotiated connection parameters (interval/latency/timeout). */
            ESP_LOGI(BLE_TAG,
                    "GAP: Conn params updated: status=%d, min_int=%d, max_int=%d, latency=%d, timeout=%d",
                    param->update_conn_params.status,
                    param->update_conn_params.min_int,
                    param->update_conn_params.max_int,
                    param->update_conn_params.latency,
                    param->update_conn_params.timeout);
            break;

        default:
            /* For a minimal device-role implementation, we ignore other GAP events. */
            break;
    }
}
/*=============================================================*/

/* GATT SERVER: Appication profile

*/

static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT: 
        {
            /* This event happens once: your app profile is registered. */
            ESP_LOGI(BLE_TAG, "GATTS: REG_EVT (RioCar profile registered)");
            g_gatts_if = gatts_if;

            /* Set the GAP-visible device name (what scanners see). */
            esp_ble_gap_set_device_name(DEVICE_NAME);

            /* Configure advertising data - PUT all advirtase services here */
            esp_ble_gap_config_adv_data(&adv_data_svc_ctrl);

            /* Create primary service. */
            esp_gatt_srvc_id_t srvc_id = {
                .is_primary = true,
                .id = {
                    .inst_id = 0,
                    .uuid = { .len = ESP_UUID_LEN_128 }
                }
            };
            memcpy(srvc_id.id.uuid.uuid.uuid128, SVC_CTRL_UUID, 16);

            /* The last parameter is "number of handles" reserved for this service in the attribute table.
            * We need enough for:
            * - service declaration (1)
            * - Power char declaration + value (2)
            * - CCCD descriptor for Power (1)
            * => total 4 is safe; we’ll reserve 8.
            */
            esp_ble_gatts_create_service(gatts_if, &srvc_id, 8);
            break;
        }

        case ESP_GATTS_CREATE_EVT: 
        {
            /* Once service is created --> add characteristics to it. */
            svc_ctrl_handle = param->create.service_handle;
            ESP_LOGI(BLE_TAG, "GATTS: Service created handle=0x%04x", svc_ctrl_handle);

            esp_ble_gatts_start_service(svc_ctrl_handle);

            /* 1) Add "power" characteristic (Write) */
            esp_bt_uuid_t power_uuid = { .len = ESP_UUID_LEN_128 };
            memcpy(power_uuid.uuid.uuid128, SVC_CTRL_CHAR_POWER_UUID, 16);

            esp_gatt_char_prop_t power_props =
                ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;

            /* Characteristic initial value can be empty; we'll react to writes in WRITE_EVT. */
            esp_attr_value_t power_val = {
                .attr_max_len = 512,
                .attr_len     = 0,
                .attr_value   = NULL
            };

            esp_ble_gatts_add_char(
                svc_ctrl_handle,
                &power_uuid,
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                power_props,
                &power_val,
                NULL
            );

            break;
        }

        case ESP_GATTS_ADD_CHAR_EVT: 
        {
            /* Each call to esp_ble_gatts_add_char results in ADD_CHAR_EVT. 
               If description should be added to a characteristic --> add it here. */

            const char *char_name = "Unknown";
            const esp_bt_uuid_t *uuid = &param->add_char.char_uuid;

            if (memcmp(uuid->uuid.uuid128, SVC_CTRL_CHAR_POWER_UUID, 16) == 0) 
            {
                // Power does not require any notification
                char_name = SVC_CTRL_CHAR_POWER_DESC;
                svc_ctrl_char_power_handle = param->add_char.attr_handle;
                ESP_LOGI(BLE_TAG, "GATTS: Control char added handle=0x%04x", svc_ctrl_char_power_handle);
            }

            // Set descriptor name
            esp_attr_value_t desc_val = {
                .attr_max_len = sizeof(char_name),
                .attr_len     = sizeof(char_name) - 1,
                .attr_value   = (uint8_t*)char_name,
            };

            esp_ble_gatts_add_char_descr(
                svc_ctrl_handle,
                &user_desc_uuid,
                ESP_GATT_PERM_READ,
                &desc_val,
                NULL
            );
            
            /* Notifications require a CCCD descriptor on the characteristic.
                * The client writes 0x0001 into CCCD to enable notifications.
            */
            break;
        }

        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        {
            /* Each call to esp_ble_gatts_add_char_descr results in this event [ESP_GATTS_ADD_CHAR_DESCR_EVT]. 
               If description should be added to a characteristic --> add it here. */

            const esp_bt_uuid_t *uuid = &param->add_char.char_uuid;

            if (memcmp(uuid->uuid.uuid128, SVC_CTRL_CHAR_POWER_UUID, 16) == 0) 
            {
                 // Power does not require any notification [For now - never fall here]

                /* This is where we learn the CCCD handle. */
                svc_ctrl_char_power_cccd_handle = param->add_char_descr.attr_handle;
                ESP_LOGI(BLE_TAG, "GATTS: CCCD added handle=0x%04x", svc_ctrl_char_power_cccd_handle);
            } 

            break;
        }

        case ESP_GATTS_CONNECT_EVT:
        {
            /* A central (PC/phone) connected. */
            g_connected = true;
            g_conn_id = param->connect.conn_id;
            ESP_LOGI(BLE_TAG, "GATTS: Connected (conn_id=%u)", g_conn_id);
            break;
        }

        case ESP_GATTS_DISCONNECT_EVT:
        {
            /* The central disconnected; restart advertising (device role). */
            ESP_LOGI(BLE_TAG, "GATTS: Disconnected, restarting advertising");
            g_connected = false;
            esp_ble_gap_start_advertising(&adv_params);
            break;
        }

        case ESP_GATTS_READ_EVT: 
        {
            /* Occures on event - Client read some characteristic */
            ESP_LOGI(BLE_TAG, "GATTS: READ_EVT handle=0x%04x", param->read.handle);

            if (param->read.handle == svc_ctrl_char_power_handle) {
                /* Char "Power" was read */
                ESP_LOGI(BLE_TAG, "Power value: %d", power_value);

                esp_gatt_rsp_t rsp;
                memset(&rsp, 0, sizeof(rsp)); 

                rsp.attr_value.handle = param->read.handle;
                rsp.attr_value.len = 2;
                rsp.attr_value.value[0] = (uint8_t)(power_value & 0xFF);
                rsp.attr_value.value[1] = (uint8_t)((power_value >> 8) & 0xFF);

                esp_ble_gatts_send_response(
                    gatts_if,
                    param->read.conn_id,
                    param->read.trans_id,
                    ESP_GATT_OK,
                    &rsp
                );
            } 
            else 
            {
                /* Anything else - default response */
                esp_ble_gatts_send_response(
                    gatts_if,
                    param->read.conn_id,
                    param->read.trans_id,
                    ESP_GATT_READ_NOT_PERMIT,
                    NULL
                );
            }
            break;
        }

        case ESP_GATTS_WRITE_EVT: {
            /* Client writes either:
                * Control characteristic (steering/throttle/etc.)
                * CCCD descriptor (enable/disable notifications)
            */
            if (param->write.is_prep) {
                /* Prepared writes are used for long writes; we keep it minimal and ignore. */
                ESP_LOGW(BLE_TAG, "GATTS: Prepared write not supported in this minimal example");
                break;
            }

            ESP_LOGI(BLE_TAG, "GATTS: WRITE_EVT handle=0x%04x, len=%d", param->write.handle, param->write.len);

            if (param->write.handle == svc_ctrl_char_power_handle) {
                /* Char "Power" was written */
                power_value = *((uint16_t *)&param->write.value[0]);
                ESP_LOGI(BLE_TAG, "Power value: %d", power_value);

                /* If you use Write With Response, you should respond OK.
                * If you use Write Without Response (Write NR), there is no response.
                * ESP-IDF sets param->write.need_rsp for write-with-response cases.
                */
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(
                        gatts_if,
                        param->write.conn_id,
                        param->write.trans_id,
                        ESP_GATT_OK,
                        NULL
                    );
                }

            } 
            else {
                /* Unknown handle written. */
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(
                        gatts_if,
                        param->write.conn_id,
                        param->write.trans_id,
                        ESP_GATT_WRITE_NOT_PERMIT,
                        NULL
                    );
                }
            }
            break;
        }

        default:
            break;
    }
}
/*=============================================================*/

void ble_initialize(void)
{
    esp_err_t ret;

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s initialize controller failed", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s enable controller failed", __func__);
        return;
    }

    esp_bluedroid_config_t cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&cfg);
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s init bluetooth failed", __func__);
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s enable bluetooth failed", __func__);
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(BLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(BLE_TAG, "gap register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(DEVICE_APP_ID);
    if (ret){
        ESP_LOGE(BLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(512);
    if (local_mtu_ret){
        ESP_LOGE(BLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    return;
}
/*=============================================================*/
