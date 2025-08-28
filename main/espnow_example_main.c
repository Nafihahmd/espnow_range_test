#include <string.h>
#include <stdint.h>
#include <limits.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "driver/gpio.h"
#include "esp_timer.h"

static const char *TAG = "espnow_range_v5";

#define WIFI_CHANNEL         1
#define LED_GPIO             15    // XIAO ESP32-C6 user LED; change if needed
#define PING_INTERVAL_MS     1000
#define REPLY_TIMEOUT_MS     2000
#define FAST_BLINK_THRESH_MS 10000

#define MSG_TYPE_PING  0x01
#define MSG_TYPE_PONG  0x02

static const uint8_t broadcast_mac[6] = {0xff,0xff,0xff,0xff,0xff,0xff};

typedef struct {
    uint8_t type;
    uint32_t seq;
    uint32_t ts_ms;
} __attribute__((packed)) espnow_pkt_t;

static volatile uint32_t last_reply_seq = 0;
static volatile int64_t  last_reply_time_ms = 0;
static volatile int8_t   last_reply_rssi = 0;
static volatile uint32_t tx_seq = 0;

static inline int64_t now_ms(void){
    return esp_timer_get_time() / 1000;
}

/* If the build doesn't define ESPNOW_WIFI_IF, default to STA interface */
#ifndef ESPNOW_WIFI_IF
#define ESPNOW_WIFI_IF ESP_IF_WIFI_STA
#endif

/* --- ESP-NOW callbacks (v5-style send callback) --- */
static void on_send_cb(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    const uint8_t *mac = NULL;
    if (tx_info) {
        /* In IDF v5+, wifi_tx_info_t contains des_addr and src_addr arrays */
        mac = tx_info->des_addr; /* destination address for the transmitted frame */
    }

    if (mac) {
        ESP_LOGD(TAG, "send_cb -> %02x:%02x:%02x:%02x:%02x:%02x status=%d",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], (int)status);
    } else {
        ESP_LOGD(TAG, "send_cb (no mac) status=%d", (int)status);
    }
}

static void ensure_peer_and_send_unicast(const uint8_t *peer_mac, const void *data, size_t len)
{
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, peer_mac, 6);
    peer.channel = WIFI_CHANNEL;
    peer.ifidx = ESPNOW_WIFI_IF;
    peer.encrypt = false;
    esp_err_t ret = esp_now_add_peer(&peer);
    if (ret != ESP_OK && ret != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGW(TAG, "esp_now_add_peer failed: %s", esp_err_to_name(ret));
    }
    ret = esp_now_send(peer_mac, (const uint8_t *)data, len);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "esp_now_send unicast err %s", esp_err_to_name(ret));
    }
}

static void on_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (!info || !data || len < (int)sizeof(espnow_pkt_t)) return;

    const espnow_pkt_t *pkt = (const espnow_pkt_t *)data;

    if (pkt->type == MSG_TYPE_PING) {
        espnow_pkt_t rsp = {
            .type = MSG_TYPE_PONG,
            .seq  = pkt->seq,
            .ts_ms = (uint32_t)now_ms()
        };
        ensure_peer_and_send_unicast(info->src_addr, &rsp, sizeof(rsp));
        ESP_LOGI(TAG, "PING from %02x:%02x:%02x:%02x:%02x:%02x seq=%u rssi=%d -> replied",
                 info->src_addr[0],info->src_addr[1],info->src_addr[2],
                 info->src_addr[3],info->src_addr[4],info->src_addr[5],
                 pkt->seq, info->rx_ctrl->rssi);
        return;
    }

    if (pkt->type == MSG_TYPE_PONG) {
        last_reply_seq = pkt->seq;
        last_reply_time_ms = now_ms();
        last_reply_rssi = info->rx_ctrl->rssi;
        ESP_LOGI(TAG, "PONG from %02x:%02x:%02x:%02x:%02x:%02x seq=%u rssi=%d",
                 info->src_addr[0],info->src_addr[1],info->src_addr[2],
                 info->src_addr[3],info->src_addr[4],info->src_addr[5],
                 pkt->seq, info->rx_ctrl->rssi);
        return;
    }
}

/* --- Tasks --- */
static void sender_task(void *arg)
{
    (void)arg;
    espnow_pkt_t pkt;
    while (1) {
        pkt.type = MSG_TYPE_PING;
        pkt.seq = ++tx_seq;
        pkt.ts_ms = (uint32_t)now_ms();

        esp_err_t ret = esp_now_send(broadcast_mac, (uint8_t *)&pkt, sizeof(pkt));
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "esp_now_send broadcast err %s", esp_err_to_name(ret));
        } else {
            ESP_LOGD(TAG, "PING broadcast seq=%u", pkt.seq);
        }
        vTaskDelay(pdMS_TO_TICKS(PING_INTERVAL_MS));
    }
}

static void led_task(void *arg)
{
    (void)arg;
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    for (;;) {
        int64_t age = now_ms() - last_reply_time_ms;
        if (last_reply_time_ms == 0) age = INT64_MAX;
        if (age <= REPLY_TIMEOUT_MS) {
            gpio_set_level(LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
        } else if (age <= FAST_BLINK_THRESH_MS) {
            gpio_set_level(LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level(LED_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
        } else {
            gpio_set_level(LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(150));
            gpio_set_level(LED_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(150));
        }
    }
}

/* --- Initialization --- */
static void wifi_init_as_station_with_channel(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* force channel for ESP-NOW */
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_LOGI(TAG, "WiFi started in STA mode, channel=%d", WIFI_CHANNEL);

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    /* Enable Long Range protocol flags for the chosen interface. */
    ESP_LOGI(TAG, "CONFIG_ESPNOW_ENABLE_LONG_RANGE enabled -> turning on WIFI_PROTOCOL_LR");
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF,
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR) );
#endif
}

#define WIFI_ENABLE      3   // GPIO3 (RF ANTENNA SWITCH EN)
#define WIFI_ANT_CONFIG  14  // GPIO14
void app_main(void)
{
#ifdef CONFIG_EXTERNAL_ANTENNA
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << WIFI_ENABLE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Configure WIFI_ANT_CONFIG (GPIO14) as output
    io_conf.pin_bit_mask = (1ULL << WIFI_ANT_CONFIG);
    gpio_config(&io_conf);

    // Set WIFI_ENABLE = LOW  (Activate RF switch control)
    gpio_set_level(WIFI_ENABLE, 0);

    // Delay 100 ms
    vTaskDelay(pdMS_TO_TICKS(100));

    // Set WIFI_ANT_CONFIG = HIGH (Use external antenna)
    gpio_set_level(WIFI_ANT_CONFIG, 1);
#endif
     esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_as_station_with_channel();

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_recv_cb));
    /* v5-style send-cb */
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_send_cb));

    xTaskCreate(led_task, "led_task", 2048, NULL, 5, NULL);
    xTaskCreate(sender_task, "sender_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "ESP-NOW range tester (v5.5) started");
}
