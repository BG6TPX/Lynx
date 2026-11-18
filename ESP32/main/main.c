/*
 * Lynx 气象站 - 正式版
 *
 * 默认流程（深睡眠模式）:
 *   上电/唤醒 -> 读 SHT40 -> OLED 刷新 -> 连 WiFi -> 连 MQTT
 *   -> 发布 HA MQTT Discovery 配置(一次性, retained) + 温湿度状态 -> 深睡眠 15 分钟 -> 循环
 *
 * HA 侧: 安装 MQTT 集成并指向同一个 broker, 设备 "Lynx Weather" 会自动出现。
 * 联调: menuconfig -> Lynx Weather Station -> 勾选 "Disable deep sleep"。
 */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "esp_sleep.h"
#include "esp_mac.h"
#include "driver/i2c_master.h"
#include "sht40.h"
#include "ssd1306.h"

#define I2C_SDA_GPIO   4
#define I2C_SCL_GPIO   5

static const char *TAG = "main";

static uint8_t s_fb[SSD1306_FB_SIZE];

static i2c_master_dev_handle_t s_sht40_dev;
static i2c_master_dev_handle_t s_oled_dev;
static esp_mqtt_client_handle_t s_mqtt;

static char s_dev_id[16];        /* wx1a2b3c, 取自 MAC, 保证主题唯一 */
static char s_client_id[24];     /* lynx-wx1a2b3c */
static char s_topic_base[24];    /* lynx/wx1a2b3c */
static char s_device_json[128];  /* HA Discovery 的 device 字段 */

static volatile bool s_wifi_got_ip;
static volatile bool s_mqtt_up;
static volatile int  s_last_pub_msg_id;
static bool s_net_inited;

static char s_ip_str[16] = "--";   /* 设备在本地(上海)网络拿到的 IP */
static int8_t s_rssi = 0;          /* WiFi 信号强度 dBm */

/* ---------------- WiFi ---------------- */

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_got_ip = false;
        esp_wifi_connect();     /* 掉线自动重连 */
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        s_wifi_got_ip = true;
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        snprintf(s_ip_str, sizeof(s_ip_str), IPSTR, IP2STR(&ev->ip_info.ip));
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            s_rssi = ap.rssi;
        }
        ESP_LOGI(TAG, "WiFi got ip %s, rssi %d dBm", s_ip_str, (int)s_rssi);
    }
}

static esp_err_t wifi_init_and_wait(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid = CONFIG_LYNX_WIFI_SSID,
            .password = CONFIG_LYNX_WIFI_PASSWORD,
            /* 不设 authmode 阈值: WPA2/WPA3 混合或 WPA3 路由器都能连 */
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));  /* 避免每次唤醒都写 NVS/Flash */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* 最多等 15 秒拿 IP, 拿不到就算失败 */
    for (int i = 0; i < 30 && !s_wifi_got_ip; i++) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    return s_wifi_got_ip ? ESP_OK : ESP_ERR_TIMEOUT;
}

/* ---------------- MQTT ---------------- */

static void mqtt_event_handler(void *h, esp_event_base_t base, int32_t id, void *data)
{
    esp_mqtt_event_handle_t ev = (esp_mqtt_event_handle_t)data;
    switch (id) {
    case MQTT_EVENT_CONNECTED:    s_mqtt_up = true; break;
    case MQTT_EVENT_DISCONNECTED: s_mqtt_up = false; break;
    case MQTT_EVENT_PUBLISHED:    s_last_pub_msg_id = ev->msg_id; break;
    default: break;
    }
}

static esp_err_t mqtt_start_and_wait(void)
{
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = CONFIG_LYNX_MQTT_BROKER_URI,
        .credentials = {
            .client_id = s_client_id,
            .username = CONFIG_LYNX_MQTT_USERNAME,
            .authentication.password = CONFIG_LYNX_MQTT_PASSWORD,
        },
        .session.keepalive = 30,
    };
    if (CONFIG_LYNX_MQTT_USERNAME[0] == '\0') {
        cfg.credentials.username = NULL;
    }
    if (CONFIG_LYNX_MQTT_PASSWORD[0] == '\0') {
        cfg.credentials.authentication.password = NULL;
    }

    s_mqtt = esp_mqtt_client_init(&cfg);
    ESP_RETURN_ON_FALSE(s_mqtt != NULL, ESP_ERR_NO_MEM, TAG, "mqtt init");
    /* v6 推荐的事件注册方式, 代替配置结构体里的 event_handle */
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(s_mqtt, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    esp_mqtt_client_start(s_mqtt);

    /* 最多等 10 秒连上 broker */
    for (int i = 0; i < 20 && !s_mqtt_up; i++) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    return s_mqtt_up ? ESP_OK : ESP_ERR_TIMEOUT;
}

/* QoS1 发布并等 broker 回 ACK。深睡前必须确认消息真的发出去了 */
static esp_err_t publish_and_wait(const char *topic, const char *data)
{
    s_last_pub_msg_id = -1;
    int msg_id = esp_mqtt_client_publish(s_mqtt, topic, data, 0, 1, 1);
    if (msg_id <= 0) {
        return ESP_FAIL;
    }
    for (int i = 0; i < 100 && s_last_pub_msg_id != msg_id; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return (s_last_pub_msg_id == msg_id) ? ESP_OK : ESP_ERR_TIMEOUT;
}

/* HA MQTT Discovery: 注册设备 + 4 个实体(温湿度测量 + IP/信号诊断), retained */
static esp_err_t publish_discovery(void)
{
    static const struct {
        const char *obj;       /* HA 实体对象 id, 也是主题片段 */
        const char *name;      /* HA 里显示的名字 */
        const char *template;  /* 从状态 JSON 里取值的模板 */
        const char *extra;     /* device_class/unit/entity_category 等附加字段 */
    } sensors[] = {
        { "temperature", "温度",
          "{{ value_json.temperature }}",
          "\"device_class\":\"temperature\",\"state_class\":\"measurement\","
          "\"unit_of_measurement\":\"°C\"" },
        { "humidity", "湿度",
          "{{ value_json.humidity }}",
          "\"device_class\":\"humidity\",\"state_class\":\"measurement\","
          "\"unit_of_measurement\":\"%\"" },
        { "ip", "IP地址",
          "{{ value_json.ip }}",
          "\"entity_category\":\"diagnostic\",\"icon\":\"mdi:ip-network\"" },
        { "rssi", "信号强度",
          "{{ value_json.rssi }}",
          "\"device_class\":\"signal_strength\",\"state_class\":\"measurement\","
          "\"unit_of_measurement\":\"dBm\",\"entity_category\":\"diagnostic\"" },
    };

    char topic[80], payload[512];
    for (size_t i = 0; i < sizeof(sensors) / sizeof(sensors[0]); i++) {
        snprintf(topic, sizeof(topic), "homeassistant/sensor/%s/%s/config",
                 s_dev_id, sensors[i].obj);
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"%s\",\"state_topic\":\"%s/state\","
                 "\"value_template\":\"%s\",%s,"
                 "\"unique_id\":\"%s_%s\",\"device\":%s}",
                 sensors[i].name, s_topic_base, sensors[i].template,
                 sensors[i].extra, s_dev_id, sensors[i].obj, s_device_json);
        esp_err_t err = publish_and_wait(topic, payload);
        ESP_RETURN_ON_ERROR(err, TAG, "pub %s discovery", sensors[i].obj);
    }
    return ESP_OK;
}

static const char *state_topic(void)
{
    static char topic[24];   /* lynx/wx1a2b3c/state */
    snprintf(topic, sizeof(topic), "%s/state", s_topic_base);
    return topic;
}

/* 确保 WiFi/MQTT 在线, 然后发布一次数据 */
static esp_err_t network_report(float t, float rh)
{
    if (!s_net_inited) {
        esp_err_t err = wifi_init_and_wait();
        s_net_inited = true;
        ESP_RETURN_ON_ERROR(err, TAG, "wifi connect timeout");
    }
    if (s_mqtt == NULL) {
        esp_err_t err = mqtt_start_and_wait();
        ESP_RETURN_ON_ERROR(err, TAG, "mqtt connect timeout");
    }

    static bool s_discovery_done = false;
    if (!s_discovery_done) {
        esp_err_t err = publish_discovery();
        ESP_RETURN_ON_ERROR(err, TAG, "discovery");
        s_discovery_done = true;
    }

    char state[96];
    snprintf(state, sizeof(state),
             "{\"temperature\":%.2f,\"humidity\":%.2f,\"ip\":\"%s\",\"rssi\":%d}",
             t, rh, s_ip_str, (int)s_rssi);
    esp_err_t err = publish_and_wait(state_topic(), state);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "report OK: %s", state);
    }
    return err;
}

/* ---------------- OLED ---------------- */

static void update_oled(bool sensor_ok, float t, float rh, const char *net_status)
{
    char l1[24], l2[24];
    if (sensor_ok) {
        snprintf(l1, sizeof(l1), "Temp: %6.2f C", t);
        snprintf(l2, sizeof(l2), "Humi: %5.1f %%", rh);
    } else {
        snprintf(l1, sizeof(l1), "Temp: ERR");
        snprintf(l2, sizeof(l2), "Humi: ERR");
    }
    ssd1306_clear(s_fb);
    ssd1306_draw_text(s_fb, 0, 1, "Lynx Weather");
    ssd1306_draw_text(s_fb, 0, 3, l1);
    ssd1306_draw_text(s_fb, 0, 5, l2);
    ssd1306_draw_text(s_fb, 0, 6, s_ip_str);
    ssd1306_draw_text(s_fb, 0, 7, net_status);
    if (ssd1306_flush(s_oled_dev, s_fb) != ESP_OK) {
        ESP_LOGE(TAG, "OLED flush failed");
    }
}

static bool read_sensor(float *t, float *rh)
{
    esp_err_t err = sht40_read(s_sht40_dev, t, rh);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SHT40 read failed: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "SHT40: %.2f C, %.2f %%RH", *t, *rh);
    return true;
}

#if CONFIG_LYNX_DISABLE_DEEP_SLEEP
/* 台架联调时才跑总线扫描, 多一层诊断信息 */
static void i2c_scan(i2c_master_bus_handle_t bus)
{
    ESP_LOGI(TAG, "--- I2C scan, SDA=GPIO%d SCL=GPIO%d ---", I2C_SDA_GPIO, I2C_SCL_GPIO);
    for (uint16_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_master_probe(bus, addr, 50) == ESP_OK) {
            ESP_LOGI(TAG, "  found device: 0x%02X", (unsigned)addr);
        }
    }
}
#endif

void app_main(void)
{
    ESP_LOGI(TAG, "Lynx weather station, report every %d min", CONFIG_LYNX_REPORT_INTERVAL_MIN);

    /* 设备唯一 ID 来自 MAC */
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_STA));
    snprintf(s_dev_id, sizeof(s_dev_id), "wx%02x%02x%02x", mac[3], mac[4], mac[5]);
    snprintf(s_client_id, sizeof(s_client_id), "lynx-%s", s_dev_id);
    snprintf(s_topic_base, sizeof(s_topic_base), "lynx/%s", s_dev_id);
    snprintf(s_device_json, sizeof(s_device_json),
             "{\"identifiers\":[\"%s\"],\"name\":\"Lynx Weather\","
             "\"manufacturer\":\"Lynx\",\"model\":\"ESP32-C3\"}", s_dev_id);

    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

#if CONFIG_LYNX_DISABLE_DEEP_SLEEP
    i2c_scan(bus);
#endif

    esp_err_t err = sht40_init(bus, &s_sht40_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SHT40 init failed: %s (check wiring/power)", esp_err_to_name(err));
    }
    err = ssd1306_init(bus, &s_oled_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OLED init failed: %s (check wiring/address)", esp_err_to_name(err));
    }

    float t, rh;
    bool ok = read_sensor(&t, &rh);
    update_oled(ok, t, rh, "");

#if CONFIG_LYNX_DISABLE_DEEP_SLEEP
    /* 台架模式: 常驻在线, 每个周期读一次、发一次, 方便看日志 */
    ESP_LOGW(TAG, "bench mode: deep sleep disabled");
    const int report_ms = CONFIG_LYNX_REPORT_INTERVAL_MIN * 60 * 1000;
    while (1) {
        esp_err_t rep = ESP_FAIL;
        if (ok) {
            rep = network_report(t, rh);
        }
        update_oled(ok, t, rh, rep == ESP_OK ? "MQTT report OK" : "MQTT report FAIL");
        vTaskDelay(pdMS_TO_TICKS(report_ms));
        ok = read_sensor(&t, &rh);
        update_oled(ok, t, rh, "");
    }
#else
    /* 正式模式: 发完就睡, 失败也照睡, 下个周期再试 */
    if (ok) {
        network_report(t, rh);
    }
    if (s_mqtt != NULL) {
        esp_mqtt_client_disconnect(s_mqtt);   /* 优雅断开, 别让 broker 等一个僵尸连接 */
    }
    vTaskDelay(pdMS_TO_TICKS(300));

    ESP_LOGI(TAG, "deep sleep for %d min", CONFIG_LYNX_REPORT_INTERVAL_MIN);
    vTaskDelay(pdMS_TO_TICKS(300));           /* 让日志发完; 深睡后 USB 会断开, 属正常 */
    esp_sleep_enable_timer_wakeup((uint64_t)CONFIG_LYNX_REPORT_INTERVAL_MIN * 60ULL * 1000000ULL);
    esp_deep_sleep_start();                   /* 不返回 */
#endif
}
