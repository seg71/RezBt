#include <stdint.h>
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include "driver/gpio.h"
#include "iot_button.h"
#include "led_strip.h"
#include "nvs_flash.h"
#include "toy_connector.h"
#include <ctype.h>

static const char *TAG = "rezbt";

/************* TinyUSB descriptors ****************/
#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)
#define VENDOR_ID 0x0B49
#define PRODUCT_ID 0x064F

const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_GENERIC_INOUT(8)
};

/************* overrideable calls ****************/

void setVibrate(uint8_t speed);
void setLED(uint8_t value);

/************* helper ****************/
void processCommand(const uint8_t *data, uint16_t length)
{
    if (length < 4)
    {
        ESP_LOGE(TAG, "Received data length %d is less than expected 4 bytes", length);
        return;
    }

    // Extract wValue and wIndex (little endian)
    uint16_t value = (data[1] << 8) | data[0];
    uint16_t index = (data[3] << 8) | data[2];

    // Extract command from low byte of wIndex
    uint8_t command = index & 0x0F; // Lower nibble

    ESP_LOGI(TAG, "Received Command: %u, Vibration Level: %u", command, (uint8_t)(value >> 8));

    // Set LED if command is within valid range
    if (command <= 7)
    {
        setLED(command);
    }
    else
    {
        ESP_LOGW(TAG, "Command %u is out of expected range (0-7)", command);
    }

    // Set Vibration
    uint8_t vibration_level = value;
    setVibrate(vibration_level);
}


/************* TinyUSB descriptors ****************/

const char *hid_string_descriptor[5] = {
    (char[]){0x09, 0x04},
    "ASCII CORPORATION",
    "ASCII Vib",
    "123456",
    "Drmn4ea Tech",
};

// Descriptor
// note: using usb 2.0 instead of 1.1 of the rez device because only this works with esp32s3
static const tusb_desc_device_t hid_device_descriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200, // USB 2.0
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 0x40, // 64 bit packages
    .idVendor = VENDOR_ID,
    .idProduct = PRODUCT_ID,
    .bcdDevice = 0x0100,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1
};

/********* TinyUSB callbacks ***************/

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    ESP_LOGD(TAG, "tud_vendor_control_xfer_cb >> bmRequestType: 0x%02X, bRequest: 0x%02X, wValue: 0x%04X, wIndex: 0x%04X, wLength: %u", 
             request->bmRequestType, request->bRequest, request->wValue, request->wIndex, request->wLength);

    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR && 
        stage == CONTROL_STAGE_SETUP && 
        (request->bmRequestType == 0x41 || request->bmRequestType == 0x40) && 
        request->bRequest == 0x00)
    {
        uint16_t value = request->wValue;
        uint16_t index = request->wIndex;

        uint8_t data[4] = {
            (uint8_t)(value & 0xFF),         // wValue low byte
            (uint8_t)((value & 0xFF00) >> 8), // wValue high byte
            (uint8_t)(index & 0xFF),         // wIndex low byte
            (uint8_t)((index & 0xFF00) >> 8)  // wIndex high byte
        };

        ESP_LOGD(TAG, "Processing Command: wValue=0x%04X, wIndex=0x%04X", value, index);

        // Call processCommand() to handle the received data
        processCommand(data, 4);

        // Respond to the control transfer (no data phase)
        tud_control_xfer(rhport, request, NULL, 0);
        return true;
    }

    return false;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
    ESP_LOGD(TAG, "tud_hid_set_report_cb >> instance: %u, report_id: %u, report_type: %u, bufsize: %u", 
             instance, report_id, report_type, bufsize);
    if (bufsize >= 8)
    {
        processCommand(buffer, bufsize);
    }
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    memset(buffer, 0, reqlen);
    return reqlen;
}

static const uint8_t hid_configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, 0x80, 98),
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 8, 10), // IN (0x81)
    TUD_HID_DESCRIPTOR(1, 4, false, sizeof(hid_report_descriptor), 0x00, 8, 10)  // OUT (0x02)
};

// Descriptor report
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    return hid_report_descriptor;
}

/************* Custom code for led strip and override implementations ****************/

#define APP_BUTTON (GPIO_NUM_0)
#define LED_STRIP_GPIO_PIN GPIO_NUM_21
#define LED_STRIP_LED_COUNT 1
#define LED_STRIP_RMT_RES_HZ (10 * 1000 * 1000)

typedef enum
{
    LED_ON,
    LED_BLINK_SLOW,
    LED_BLINK_FAST,
    LED_GAME,
    LED_ONETIME_EFFECT_GAME_DIRECT,
    LED_ONETIME_EFFECT_GAME_FADE,
    LED_OFF
} LMode;

typedef enum
{
    GAME_DIRECT,
    GAME_FADE
} GMode;

// LED Strip Configuration
TaskHandle_t ledTaskHandle = NULL;
led_strip_handle_t strip;
ToyConnector *toy = NULL;
GMode gameMode = GAME_DIRECT;
LMode ledMode = LED_OFF;

uint8_t gameVibrationSpeed = 0;
uint8_t gameLedValue = 0;
bool sendVibrationBLE = false;

void updateLED()
{
    int8_t blinkCounter = 0;
    while (1)
    {
        switch (ledMode)
        {
        case LED_ON:
            led_strip_set_pixel(strip, 0, 0, 0, 150);
            break;
        case LED_BLINK_FAST:
            blinkCounter -= 5;
            // fall through
        case LED_BLINK_SLOW:
            --blinkCounter;
            if (blinkCounter <= 0)
            {
                led_strip_set_pixel(strip, 0, 0, 0, 150);
                blinkCounter = 50;
            }
            else
                led_strip_clear(strip);
            break;
        case LED_GAME:
            {
                uint8_t red = gameVibrationSpeed * 0.3;
                uint8_t green = gameLedValue * 30;
                led_strip_clear(strip);
                led_strip_set_pixel(strip, 0, red, green, 0);
            }
            break;
        case LED_ONETIME_EFFECT_GAME_DIRECT:
            {
                for (uint8_t i = 0; i < 3; ++i)
                {
                    led_strip_clear(strip);
                    led_strip_refresh(strip);
                    vTaskDelay(pdMS_TO_TICKS(30));
                    led_strip_set_pixel(strip, 0, 255, 0, 0);
                    led_strip_refresh(strip);
                    vTaskDelay(pdMS_TO_TICKS(30));
                }
                ledMode = LED_GAME;
            }
            break;
        case LED_ONETIME_EFFECT_GAME_FADE:
            {
                for (uint8_t r = 255; r > 0; r -=5)
                {
                    led_strip_set_pixel(strip, 0, r, 0, 0);
                    led_strip_refresh(strip);
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                ledMode = LED_GAME;
            }
            break;
        default:
            led_strip_clear(strip);
            break;
        }
        led_strip_refresh(strip);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


void setVibrate(uint8_t speed)
{
    // ignore zero values in fade mode because it decreases on its own
    if (gameVibrationSpeed > 0 && speed == 0 && gameMode == GAME_FADE)
        return;
    ledMode = LED_GAME;
    gameVibrationSpeed = speed;
    if (sendVibrationBLE)
        toy_connector_vibrate(toy, speed); // Update vibration intensity
    ESP_LOGI(TAG, "Vibration speed set to: %d", speed);
}

void setLED(uint8_t value)
{
    ledMode = LED_GAME;
    gameLedValue = value & 7;
    ESP_LOGI(TAG, "LED state set to: %d", gameLedValue);
}

void setup_led_strip()
{
    // LED strip general initialization, according to your LED board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_PIN,                        // The GPIO connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_COUNT,                             // The number of LEDs in the strip
        .led_model = LED_MODEL_WS2812,                               // LED strip model
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_RGB, // The color order of the strip: GRB
        .flags = {
            .invert_out = false, // don't invert the output signal
        }
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock sources can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .mem_block_symbols = 64,               // the memory size of each RMT channel, in words (4 bytes)
        .flags = {
            .with_dma = false, // DMA feature is available on chips like ESP32-S3/P4
        }
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &strip));
    led_strip_clear(strip);
}

/**
 * @brief Start the BLE pairing process.
 *
 * This function initiates the BLE pairing by starting the scanning process
 * and waits for the pairing to complete or timeout.
 *
 * @return true if pairing was successful, false otherwise.
 */
bool pairBLE()
{
    // Begin the BLE pairing process
    sendVibrationBLE = false;
    ledMode = LED_BLINK_FAST;

    bool result = toy_connector_start_pairing_process();
    if (result)
    {
        ESP_LOGI(TAG, "ToyConnector paired successfully");
        // Set vibration intensity
        toy_connector_vibrate(toy, 255);
        vTaskDelay(pdMS_TO_TICKS(100));
        toy_connector_vibrate(toy, 0);
        ledMode = LED_ON;
        sendVibrationBLE = true;
    }
    else
    {
        ledMode = LED_BLINK_SLOW;
        ESP_LOGE(TAG, "Failed to pair with ToyConnector");
    }
    return result;
}

/**
 * @brief Button callback for single click to change game mode.
 *
 * @param arg Button handle.
 * @param data Button event data.
 */
static void button_event_mode(void *arg, void *data)
{
    iot_button_print_event((button_handle_t)arg);
    if (gameMode == GAME_DIRECT)
    {
        ESP_LOGI(TAG, "Changing game mode to FADE");
        gameMode = GAME_FADE;
        ledMode = LED_ONETIME_EFFECT_GAME_FADE;
    }
    else
    {
        ESP_LOGI(TAG, "Changing game mode to DIRECT");
        gameMode = GAME_DIRECT;
        ledMode = LED_ONETIME_EFFECT_GAME_DIRECT;
    }
}

/**
 * @brief Button callback for long press hold to initiate pairing.
 *
 * @param arg Button handle.
 * @param data Button event data.
 */
static void button_event_pair(void *arg, void *data)
{
    iot_button_print_event((button_handle_t)arg);
    pairBLE();
}

void app_main()
{
    ESP_LOGI(TAG, "Initializing...");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized successfully");

    // Initialize button handling
    button_config_t btn_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .gpio_num = APP_BUTTON,
            .active_level = 0,
#if CONFIG_GPIO_BUTTON_SUPPORT_POWER_SAVE
            .enable_power_save = true,
#endif
        },
    };
    button_handle_t btn = iot_button_create(&btn_cfg);
    assert(btn);
    esp_err_t err = iot_button_register_cb(btn, BUTTON_SINGLE_CLICK, button_event_mode, NULL);
    err |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_HOLD, button_event_pair, NULL);
    ESP_ERROR_CHECK(err);

    // Initialize TinyUSB
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = &hid_device_descriptor,
        .string_descriptor = hid_string_descriptor,
        .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
        .external_phy = false,
        .configuration_descriptor = hid_configuration_descriptor,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    // Initialize LED Strip
    setup_led_strip();
    led_strip_set_pixel(strip, 0, 0, 0, 20);
    led_strip_refresh(strip);
    xTaskCreate(updateLED, "LED Task", 4096, NULL, 10, &ledTaskHandle);

    ledMode = LED_BLINK_SLOW;

    // Initialize ToyConnector once
    toy = toy_connector_init();
    if (toy == NULL)
    {
        ESP_LOGE(TAG, "Failed to initialize ToyConnector");
        return;
    }

    // Start initial pairing
    pairBLE();

    while (1)
    {
        if (gameMode == GAME_FADE && gameVibrationSpeed > 0)
        {
            --gameVibrationSpeed;
            if (gameVibrationSpeed > 0)
                --gameVibrationSpeed;
            setVibrate(gameVibrationSpeed);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
