// toy_connector.c

#include "toy_connector.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"
#include "freertos/event_groups.h"
#include "string.h"
#include <stdlib.h>
#include <stdbool.h> // Include for 'bool' type

static const char *TAG = "toy_connector";

#define AMORELIE_JOY_SERVICE_UUID 0xffe0
#define RX_UUID                   0xffe2
#define TX_UUID                   0xffe3

// Event Group bits
#define CONNECTED_BIT                  BIT0
#define SERVICE_DISCOVERED_BIT         BIT1
#define CHARACTERISTICS_DISCOVERED_BIT BIT2
#define REGISTERED_BIT                 BIT3 // NEW

// Maximum number of GATT clients
#define GATTC_APP_ID              1

// Timeout for pairing process
#define BLE_CONNECT_TIMEOUT       20   // 20 seconds
#define BLE_CONNECT_TIMEOUT_MS    (BLE_CONNECT_TIMEOUT * 1000)

#define MAX_CHAR_COUNT            10

static const struct {
    uint16_t id;
    const char *name;
} allowed_devices[] = {
    {0x4D02, "Amorelie Joy Move"},    
    {0x4D05, "Amorelie Joy Cha-Cha"},
    {0x4D06, "Amorelie Joy Boogie"}, 
    {0x4D01, "Amorelie Joy Shimmer"},
    {0x4D03, "Amorelie Joy Grow"},   
    {0x4D04, "Amorelie Joy Shuffle"},
    {0x4D07, "Amorelie Joy Salsa"}
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type          = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval      = 0x50,
    .scan_window        = 0x30,
    .scan_duplicate     = BLE_SCAN_DUPLICATE_DISABLE
};

// Forward declarations
static void gattc_event_handler(esp_gattc_cb_event_t event, 
                                esp_gatt_if_t gattc_if, 
                                esp_ble_gattc_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, 
                              esp_ble_gap_cb_param_t *param);

struct ToyConnector {
    esp_gattc_cb_t  gattc_cb;
    esp_gatt_if_t   gattc_if;
    uint16_t        conn_id;
    uint16_t        service_handle;
    uint16_t        rx_char_handle;
    uint16_t        tx_char_handle;
    bool            connected;
    bool            service_found;
    bool            characteristics_found;
    EventGroupHandle_t event_group;
};

static ToyConnector *global_toy_connector = NULL;

//------------------------------------------------------
// Check if device is one of the "allowed_devices"
static bool device_matches(const uint8_t *adv_data, uint8_t adv_len)
{
    uint8_t name_len = 0;
    uint8_t *name_data = esp_ble_resolve_adv_data(
        (uint8_t *)adv_data,
        ESP_BLE_AD_TYPE_NAME_CMPL,
        &name_len
    );
    if (!name_data || name_len < 1) {
        return false;
    }

    char tmp[32];
    if (name_len >= sizeof(tmp)) {
        name_len = sizeof(tmp) - 1;
    }
    memcpy(tmp, name_data, name_len);
    tmp[name_len] = '\0';

    uint16_t did = (uint16_t)strtol(tmp, NULL, 16);
    for (size_t i = 0; i < sizeof(allowed_devices) / sizeof(allowed_devices[0]); i++) {
        if (did == allowed_devices[i].id) {
            ESP_LOGI(TAG, "Found known device: %s (ID: 0x%04X)",
                allowed_devices[i].name, did);
            return true;
        }
    }
    return false;
}

// ------------------------------------------------------
// GAP callback
// ------------------------------------------------------
static void gap_event_handler(esp_gap_ble_cb_event_t event, 
                              esp_ble_gap_cb_param_t *param)
{
    if (global_toy_connector == NULL) {
        ESP_LOGE(TAG, "[gap_event_handler] No ToyConnector instance");
        return;
    }

    ToyConnector *toy = global_toy_connector;

    switch (event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Scan parameters set. Not scanning yet.");
            break;

        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Failed to start scanning: %d", param->scan_start_cmpl.status);
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_ble_gap_start_scanning(BLE_CONNECT_TIMEOUT);
            } else {
                ESP_LOGI(TAG, "Scanning started successfully");
            }
            break;

        case ESP_GAP_BLE_SCAN_RESULT_EVT:
        {
            if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
                const uint8_t *adv_data = param->scan_rst.ble_adv;
                uint8_t adv_len = param->scan_rst.adv_data_len;

                if (device_matches(adv_data, adv_len)) {
                    ESP_LOGI(TAG, "Device matches, stopping scan and connecting...");
                    esp_ble_gap_stop_scanning();
                    esp_ble_gattc_open(toy->gattc_if,
                                       param->scan_rst.bda,
                                       param->scan_rst.ble_addr_type,
                                       true);
                }
            }
            break;
        }

        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Failed to stop scanning: %d", param->scan_stop_cmpl.status);
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_ble_gap_stop_scanning();
            } else {
                ESP_LOGI(TAG, "Scanning stopped successfully");
            }
            break;

        default:
            ESP_LOGD(TAG, "Unhandled GAP event: %d", event);
            break;
    }
}

// ------------------------------------------------------
// GATTC event callback
// ------------------------------------------------------
static void gattc_event_handler(esp_gattc_cb_event_t event, 
                                esp_gatt_if_t gattc_if, 
                                esp_ble_gattc_cb_param_t *param)
{
    if (global_toy_connector == NULL) {
        ESP_LOGE(TAG, "[gattc_event_handler] No ToyConnector instance");
        return;
    }

    ToyConnector *toy = global_toy_connector;

    switch (event) {
        case ESP_GATTC_REG_EVT:
            ESP_LOGI(TAG, "GATTC_REG_EVT: app_id=%d, gattc_if=%d",
                     param->reg.app_id, gattc_if);
            toy->gattc_if = gattc_if;
            // Set REGISTERED_BIT
            xEventGroupSetBits(toy->event_group, REGISTERED_BIT);
            break;

        case ESP_GATTC_CONNECT_EVT:
            ESP_LOGI(TAG, "GATTC_CONNECT_EVT: conn_id=%d, remote_bda=", param->connect.conn_id);
            esp_log_buffer_hex(TAG, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            break;

        case ESP_GATTC_OPEN_EVT:
            ESP_LOGI(TAG, "GATTC_OPEN_EVT: status=%d", param->open.status);
            if (param->open.status == ESP_GATT_OK) {
                toy->conn_id = param->open.conn_id;
                toy->connected = true;
                xEventGroupSetBits(toy->event_group, CONNECTED_BIT);
                ESP_LOGI(TAG, "Connection established with device");
                esp_ble_gattc_search_service(gattc_if, param->open.conn_id, NULL);
            } else {
                ESP_LOGE(TAG, "Failed to open GATT connection, status=%d", param->open.status);
                esp_ble_gap_start_scanning(BLE_CONNECT_TIMEOUT);
            }
            break;

        case ESP_GATTC_SEARCH_RES_EVT:
        {
            if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 &&
                param->search_res.srvc_id.uuid.uuid.uuid16 == AMORELIE_JOY_SERVICE_UUID)
            {
                toy->service_handle = param->search_res.start_handle;
                toy->service_found  = true;
                ESP_LOGI(TAG, "Found Amorelie service: handle=%d", toy->service_handle);
                xEventGroupSetBits(toy->event_group, SERVICE_DISCOVERED_BIT);
            }
            break;
        }

        case ESP_GATTC_SEARCH_CMPL_EVT:
            ESP_LOGI(TAG, "GATTC_SEARCH_CMPL_EVT: status=%d", param->search_cmpl.status);
            if (toy->service_found) {
                esp_gattc_char_elem_t char_result[MAX_CHAR_COUNT];
                uint16_t char_count = MAX_CHAR_COUNT;
                uint16_t offset = 0;

                esp_gatt_status_t status = esp_ble_gattc_get_all_char(
                    toy->gattc_if,
                    toy->conn_id,
                    toy->service_handle,
                    0xFFFF,
                    char_result,
                    &char_count,
                    offset
                );
                if (status == ESP_GATT_OK) {
                    for (int i = 0; i < char_count; i++) {
                        esp_bt_uuid_t char_uuid = char_result[i].uuid;
                        if (char_uuid.uuid.uuid16 == RX_UUID) {
                            toy->rx_char_handle = char_result[i].char_handle;
                            ESP_LOGI(TAG, "Found RX char: handle=%d", toy->rx_char_handle);
                        }
                        if (char_uuid.uuid.uuid16 == TX_UUID) {
                            toy->tx_char_handle = char_result[i].char_handle;
                            ESP_LOGI(TAG, "Found TX char: handle=%d", toy->tx_char_handle);
                        }
                    }
                    if (toy->rx_char_handle != 0 && toy->tx_char_handle != 0) {
                        toy->characteristics_found = true;
                        xEventGroupSetBits(toy->event_group, CHARACTERISTICS_DISCOVERED_BIT);
                        ESP_LOGI(TAG, "Required characteristics discovered");

                        // === ADDED/CHANGED HERE ===
                        // Send the initialization command [0x03] once we have TX handle
                        uint8_t init_cmd = 0x03;
                        esp_err_t ret_init = esp_ble_gattc_write_char(
                            toy->gattc_if,
                            toy->conn_id,
                            toy->tx_char_handle, // Important: we write to TX
                            sizeof(init_cmd),
                            &init_cmd,
                            ESP_GATT_WRITE_TYPE_RSP,
                            ESP_GATT_AUTH_REQ_NONE
                        );
                        if (ret_init == ESP_OK) {
                            ESP_LOGI(TAG, "Init command [0x03] sent to device");
                        } else {
                            ESP_LOGE(TAG, "Failed to send init command: %s", esp_err_to_name(ret_init));
                        }
                        // === END CHANGES ===
                    } else {
                        ESP_LOGE(TAG, "Required characteristics not found");
                        esp_ble_gap_start_scanning(BLE_CONNECT_TIMEOUT);
                    }
                } else {
                    ESP_LOGE(TAG, "Failed to initiate char discovery: %d", status);
                    esp_ble_gap_start_scanning(BLE_CONNECT_TIMEOUT);
                }
            } else {
                ESP_LOGE(TAG, "Target service not found, restarting scanning");
                esp_ble_gap_start_scanning(BLE_CONNECT_TIMEOUT);
            }
            break;

        case ESP_GATTC_WRITE_CHAR_EVT:
            ESP_LOGI(TAG, "GATTC_WRITE_CHAR_EVT: status=%d", param->write.status);
            if (param->write.status != ESP_GATT_OK) {
                ESP_LOGE(TAG, "Failed to write characteristic");
            }
            break;

        case ESP_GATTC_CLOSE_EVT:
            ESP_LOGI(TAG, "GATTC_CLOSE_EVT: reason=0x%x", param->close.reason);
            toy->connected = false;
            toy->service_found = false;
            toy->characteristics_found = false;
            esp_ble_gap_start_scanning(BLE_CONNECT_TIMEOUT);
            break;

        default:
            ESP_LOGD(TAG, "Unhandled GATTC event: %d", event);
            break;
    }
}

// ------------------------------------------------------
// Public / ToyConnector interface
// ------------------------------------------------------
ToyConnector* toy_connector_init(void) {
    if (global_toy_connector != NULL) {
        ESP_LOGW(TAG, "ToyConnector already initialized");
        return global_toy_connector;
    }

    ToyConnector *toy = calloc(1, sizeof(ToyConnector));
    if (toy == NULL) {
        ESP_LOGE(TAG, "Failed to allocate ToyConnector");
        return NULL;
    }

    global_toy_connector = toy;

    toy->event_group = xEventGroupCreate();
    if (toy->event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        free(toy);
        return NULL;
    }

    esp_err_t ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Bluetooth mem release (classic) failed: %s", esp_err_to_name(ret));
        vEventGroupDelete(toy->event_group);
        free(toy);
        return NULL;
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
        vEventGroupDelete(toy->event_group);
        free(toy);
        return NULL;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
        esp_bt_controller_deinit();
        vEventGroupDelete(toy->event_group);
        free(toy);
        return NULL;
    }

    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        vEventGroupDelete(toy->event_group);
        free(toy);
        return NULL;
    }
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        esp_bluedroid_deinit();
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        vEventGroupDelete(toy->event_group);
        free(toy);
        return NULL;
    }

    esp_ble_gap_register_callback(gap_event_handler);

    ret = esp_ble_gattc_register_callback(gattc_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATTC register callback failed: %s", esp_err_to_name(ret));
        esp_ble_gattc_register_callback(NULL);
        goto cleanup;
    }

    ret = esp_ble_gattc_app_register(GATTC_APP_ID);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATTC app register failed: %s", esp_err_to_name(ret));
        esp_ble_gattc_app_unregister(GATTC_APP_ID);
        esp_ble_gattc_register_callback(NULL);
        goto cleanup;
    }

    // Set BLE Scan Parameters (but DON'T auto-start scanning here)
    ret = esp_ble_gap_set_scan_params(&ble_scan_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set scan params: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    ESP_LOGI(TAG, "ToyConnector initialized successfully");
    return toy;

cleanup:
    ESP_LOGE(TAG, "Encountered an error. Starting cleanup");
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    vEventGroupDelete(toy->event_group);
    free(toy);
    return NULL;
}

static bool toy_connector_start_pairing(ToyConnector* toy) {
    if (toy == NULL) {
        ESP_LOGE(TAG, "ToyConnector is NULL");
        return false;
    }

    // Wait until we have a valid gattc_if (registered)
    EventBits_t reg_bits = xEventGroupWaitBits(toy->event_group,
                                               REGISTERED_BIT,
                                               false,
                                               true,
                                               pdMS_TO_TICKS(BLE_CONNECT_TIMEOUT_MS));
    if (!(reg_bits & REGISTERED_BIT)) {
        ESP_LOGE(TAG, "GATTC registration timed out or failed");
        return false;
    }

    if (toy->connected) {
        ESP_LOGI(TAG, "Already connected to a device");
        return true;
    }

    // Clear old bits
    xEventGroupClearBits(toy->event_group, CONNECTED_BIT | SERVICE_DISCOVERED_BIT | CHARACTERISTICS_DISCOVERED_BIT);

    // Start scanning
    esp_err_t ret = esp_ble_gap_start_scanning(BLE_CONNECT_TIMEOUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start scanning: %s", esp_err_to_name(ret));
        return false;
    }
    ESP_LOGI(TAG, "Started scanning for %d seconds", BLE_CONNECT_TIMEOUT);

    // Wait for connected + discovered
    EventBits_t bits = xEventGroupWaitBits(toy->event_group,
                                           CONNECTED_BIT | SERVICE_DISCOVERED_BIT | CHARACTERISTICS_DISCOVERED_BIT,
                                           pdFALSE, pdTRUE, 
                                           pdMS_TO_TICKS(BLE_CONNECT_TIMEOUT_MS));
    if ((bits & (CONNECTED_BIT | SERVICE_DISCOVERED_BIT | CHARACTERISTICS_DISCOVERED_BIT)) ==
        (CONNECTED_BIT | SERVICE_DISCOVERED_BIT | CHARACTERISTICS_DISCOVERED_BIT))
    {
        ESP_LOGI(TAG, "BLE connected and characteristics discovered");
        return true;
    } else {
        ESP_LOGE(TAG, "BLE connection or discovery timed out");
        return false;
    }
}

bool toy_connector_start_pairing_process(void) {
    if (global_toy_connector == NULL) {
        ESP_LOGE(TAG, "ToyConnector not initialized");
        return false;
    }
    bool result = toy_connector_start_pairing(global_toy_connector);
    if (result) {
        ESP_LOGI(TAG, "Pairing process completed successfully");
    } else {
        ESP_LOGE(TAG, "Pairing process failed or timed out");
    }
    return result;
}

// ------------------------------------------------------
// CHANGED toy_connector_vibrate()
// ------------------------------------------------------
void toy_connector_vibrate(ToyConnector* toy, uint8_t strength) {
    if (toy == NULL || !toy->characteristics_found) {
        ESP_LOGE(TAG, "ToyConnector not ready or characteristics not found");
        return;
    }
    uint8_t intensity = (100.0f / 255.0f) * strength;

    // === ADDED/CHANGED HERE ===
    // According to Buttplug.io, for "Amorelie Joy" we send 3 bytes: [0x01, 0x01, speed]
    // to the *TX* characteristic, after sending an init of [0x03] once.
    // 'speed' is 0-100 in the official protocol, so we just pass 'intensity' directly
    // (or you could clamp 0-100 if needed).
    uint8_t command[3];
    command[0] = 0x01;      // static header
    command[1] = 0x01;      // "steady" pattern
    command[2] = intensity; // speed: 0-100

    esp_err_t ret = esp_ble_gattc_write_char(
        toy->gattc_if,
        toy->conn_id,
        toy->tx_char_handle,    // IMPORTANT: use the TX handle
        sizeof(command),
        command,
        ESP_GATT_WRITE_TYPE_RSP,
        ESP_GATT_AUTH_REQ_NONE
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write vibration intensity: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Vibration command set: [0x01, 0x01, %d]", intensity);
    }
    // === END CHANGES ===
}

void toy_connector_deinit(ToyConnector* toy) {
    if (toy == NULL) {
        return;
    }
    if (toy->connected) {
        esp_ble_gattc_close(toy->gattc_if, toy->conn_id);
    }
    esp_ble_gattc_app_unregister(GATTC_APP_ID);
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    vEventGroupDelete(toy->event_group);
    free(toy);
    global_toy_connector = NULL;
    ESP_LOGI(TAG, "ToyConnector deinitialized");
}
