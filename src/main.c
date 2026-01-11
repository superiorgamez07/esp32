#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <math.h>

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/* ===================== CONFIG ===================== */

#define TAG "FallBelt"

/* I2C */
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_SDA GPIO_NUM_21
#define I2C_SCL GPIO_NUM_20
#define I2C_FREQ_HZ 100000

/* MMA7660FC */
#define MMA7660_ADDR 0x4C
#define MMA_XOUT 0x00
#define MMA_YOUT 0x01
#define MMA_ZOUT 0x02
#define MMA_MODE 0x07

#define FALL_THRESHOLD_G 2.2f

/* GPIO */
#define BUTTON_FALL GPIO_NUM_4
#define BUTTON_USER GPIO_NUM_5
#define BUZZER GPIO_NUM_6
#define LED GPIO_NUM_7

/* Buzzer */
#define TONE_FALL 2000
#define TONE_SHORT 3000
#define TONE_LONG 4000

#define BEEP_DURATION_MS 300
#define BEEP_INTERVAL_MS 2000
#define MAX_BEEPS 10
#define LONG_PRESS_MS 2000

/* ===================== BLE ===================== */

static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t char_handle = 0;

// Forward declarations
static int ble_gap_event(struct ble_gap_event *event, void *arg);
static int ble_alert_chr_access(uint16_t conn_handle_,
                               uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt,
                               void *arg);
void ble_app_on_sync(void);

// UUIDs (reversed for ESP32-C6 endianness)
static const ble_uuid128_t svc_uuid =
    BLE_UUID128_INIT(0x21,0x43,0x65,0x87,0x09,0xba,0xdc,0xfe,
                     0xef,0xcd,0xab,0x90,0x78,0x56,0x34,0x12);

static const ble_uuid128_t chr_uuid =
    BLE_UUID128_INIT(0x99,0x88,0x77,0x66,0x55,0x44,0x33,0x22,
                     0x11,0x00,0xff,0xee,0xdd,0xcc,0xbb,0xaa);

// GATT service definition
static const struct ble_gatt_chr_def gatt_characteristics[] = {
    {
        .uuid = &chr_uuid.u,
        .access_cb = ble_alert_chr_access,
        .flags = BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &char_handle,
    },
    { 0 }
};

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = gatt_characteristics,
    },
    { 0 }
};

/* ===================== BUZZER ===================== */

void beep(int freq_hz, int duration_ms)
{
    ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, freq_hz);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

/* ===================== BLE ===================== */

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "BLE connected, handle=%d", conn_handle);
        } else {
            ESP_LOGI(TAG, "BLE connect failed, status=%d", event->connect.status);
            ble_app_on_sync();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE disconnected, reason=%d", event->disconnect.reason);
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ble_app_on_sync();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "BLE advertising complete");
        ble_app_on_sync();
        return 0;

    default:
        return 0;
    }
}

static int ble_alert_chr_access(uint16_t conn_handle_,
                               uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt,
                               void *arg)
{
    return 0;
}

void send_ble_alert(uint8_t val)
{
    ESP_LOGI(TAG, "send_ble_alert called with val=%d, conn_handle=%d, char_handle=%d", 
             val, conn_handle, char_handle);
             
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGW(TAG, "Cannot send alert, not connected");
        return;
    }
    
    if (char_handle == 0) {
        ESP_LOGE(TAG, "Cannot send alert, char_handle is 0!");
        return;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(&val, sizeof(val));
    if (om == NULL) {
        ESP_LOGE(TAG, "Failed to allocate mbuf");
        return;
    }

    int rc = ble_gatts_notify_custom(conn_handle, char_handle, om);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to send notification, rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "✓ Sent BLE alert: %d", val);
    }
}

void ble_app_on_sync(void)
{
    int rc;
    
    rc = ble_svc_gap_device_name_set("FallBelt");
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set device name, rc=%d", rc);
    }

    struct ble_hs_adv_fields fields = {0};
    const char *device_name = "FallBelt";
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = &svc_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    
    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set adv fields, rc=%d", rc);
    }

    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = 0x20;
    adv_params.itvl_max = 0x40;

    rc = ble_gap_adv_start(
        BLE_OWN_ADDR_PUBLIC,
        NULL,
        BLE_HS_FOREVER,
        &adv_params,
        ble_gap_event,
        NULL
    );
    
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start advertising, rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "✓ BLE advertising started");
    }
}

void bleprph_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_init(void)
{
    ESP_LOGI(TAG, "Initializing BLE...");
    
    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init nimble %d", ret);
        return;
    }

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to count GATT config, rc=%d", rc);
    }

    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to add GATT services, rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "GATT services added, char_handle=%d", char_handle);
    }

    ble_hs_cfg.sync_cb = ble_app_on_sync;
    ble_hs_cfg.sm_bonding = 0;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 0;

    nimble_port_freertos_init(bleprph_host_task);
    
    ESP_LOGI(TAG, "✓ BLE initialized");
}

/* ===================== I2C ===================== */

void i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void mma_write(uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MMA7660_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
}

uint8_t mma_read(uint8_t reg)
{
    uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MMA7660_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MMA7660_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return data;
}

void mma_init(void)
{
    mma_write(MMA_MODE, 0x01);
    ESP_LOGI(TAG, "MMA7660 initialized");
}

/* ===================== FALL DETECTION ===================== */

static TickType_t last_fall = 0;

float mma_to_g(uint8_t raw)
{
    int8_t v = raw & 0x3F;
    if (v & 0x20) v |= 0xC0;
    return v * 0.047f;
}

bool detect_fall(void)
{
    if (xTaskGetTickCount() - last_fall < pdMS_TO_TICKS(3000))
        return false;

    float x = mma_to_g(mma_read(MMA_XOUT));
    float y = mma_to_g(mma_read(MMA_YOUT));
    float z = mma_to_g(mma_read(MMA_ZOUT));

    float mag = sqrtf(x*x + y*y + z*z);

    if (mag > FALL_THRESHOLD_G) {
        last_fall = xTaskGetTickCount();
        return true;
    }
    return false;
}

/* ===================== MAIN ===================== */

void app_main(void)
{
    ESP_LOGI(TAG, "🚨 FallBelt starting...");
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");
    
    i2c_init();
    mma_init();

    gpio_config_t btn = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL<<BUTTON_FALL) | (1ULL<<BUTTON_USER),
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&btn);

    gpio_config_t led = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL<<LED)
    };
    gpio_config(&led);

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = TONE_FALL,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch = {
        .gpio_num = BUZZER,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ch);

    ble_init();
    
    ESP_LOGI(TAG, "✓ FallBelt ready!\n");

    while (1) {
        if (detect_fall()) {
            gpio_set_level(LED, 1);

            int beep_count = 0;
            int emergency = 1;

            while (beep_count < MAX_BEEPS) {
                beep(TONE_FALL, BEEP_DURATION_MS);

                if (!gpio_get_level(BUTTON_USER)) {
                    int press = 0;
                    while (!gpio_get_level(BUTTON_USER) && press < 10000) {
                        vTaskDelay(pdMS_TO_TICKS(50));
                        press += 50;
                    }

                    if (press < LONG_PRESS_MS) {
                        beep(TONE_SHORT, 300);
                        send_ble_alert(0);
                    } else {
                        beep(TONE_LONG, 300);
                        vTaskDelay(pdMS_TO_TICKS(100));
                        beep(TONE_LONG, 300);
                        send_ble_alert(1);
                    }
                    emergency = 0;
                    break;
                }

                vTaskDelay(pdMS_TO_TICKS(BEEP_INTERVAL_MS));
                beep_count++;
            }

            if (emergency) {
                for (int i = 0; i < 5; i++) {
                    beep(TONE_LONG, 300);
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                send_ble_alert(2);
            }

            gpio_set_level(LED, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}