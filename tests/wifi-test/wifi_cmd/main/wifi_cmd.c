/*
 * wifi_cmd — ESP-IDF WiFi command receiver
 *
 * Commands (sent as plain text over TCP):
 *   "print"   → logs "print cmd received" to monitor
 *   "led_on"  → turns LED on
 *   "led_off" → turns LED off
 *   "msg"     → sends "msg response sent" back to client
 *
 * Build-time config (set via menuconfig or sdkconfig.defaults):
 *   CONFIG_WIFI_SSID
 *   CONFIG_WIFI_PASSWORD
 *   CONFIG_TCP_PORT          (default 3333)
 *   CONFIG_LED_GPIO          (default 2 — onboard LED on most DevKits)
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

/* ── User config ──────────────────────────────────────────────────────────── */
#ifndef CONFIG_WIFI_SSID
#define CONFIG_WIFI_SSID      "YOUR_SSID"
#endif
#ifndef CONFIG_WIFI_PASSWORD
#define CONFIG_WIFI_PASSWORD  "YOUR_PASSWORD"
#endif
#ifndef CONFIG_TCP_PORT
#define CONFIG_TCP_PORT       3333
#endif
#ifndef CONFIG_LED_GPIO
#define CONFIG_LED_GPIO       2        /* GPIO2 = onboard LED on most DevKits */
#endif

#define WIFI_CONNECTED_BIT    BIT0
#define WIFI_FAIL_BIT         BIT1
#define MAX_RETRY             5
#define RX_BUF_SIZE           128

static const char *TAG = "wifi_cmd";
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_count = 0;

/* ── WiFi event handler ───────────────────────────────────────────────────── */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_count < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_count++;
            ESP_LOGW(TAG, "Retrying WiFi connection (%d/%d)…",
                     s_retry_count, MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "WiFi connection failed after %d retries", MAX_RETRY);
        }

    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "✓ WiFi connected — IP: " IPSTR,
                 IP2STR(&event->ip_info.ip));
        s_retry_count = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* ── WiFi init (STA mode) ─────────────────────────────────────────────────── */
static bool wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t h_wifi, h_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &h_wifi));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &h_ip));

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid     = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to SSID: %s …", CONFIG_WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE,
        pdMS_TO_TICKS(15000));  /* 15 s timeout */

    if (bits & WIFI_CONNECTED_BIT) {
        return true;
    }
    ESP_LOGE(TAG, "Failed to connect to WiFi");
    return false;
}

/* ── LED helpers ──────────────────────────────────────────────────────────── */
static void led_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << CONFIG_LED_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(CONFIG_LED_GPIO, 0);  /* start off */
}

static void led_set(bool on)
{
    gpio_set_level(CONFIG_LED_GPIO, on ? 1 : 0);
    ESP_LOGI(TAG, "LED %s", on ? "ON" : "OFF");
}

/* ── Command processor ────────────────────────────────────────────────────── */
static void process_command(int client_sock, const char *cmd)
{
    /* Trim trailing whitespace / newlines */
    char buf[RX_BUF_SIZE];
    strncpy(buf, cmd, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    int len = strlen(buf);
    while (len > 0 && (buf[len-1] == '\n' || buf[len-1] == '\r'
                        || buf[len-1] == ' ')) {
        buf[--len] = '\0';
    }

    ESP_LOGI(TAG, "Command received: \"%s\"", buf);

    if (strcmp(buf, "print") == 0) {
        /* ── Command 1: print ── */
        ESP_LOGI(TAG, "print cmd received");
        const char *ack = "ACK: print cmd received\n";
        send(client_sock, ack, strlen(ack), 0);

    } else if (strcmp(buf, "led_on") == 0) {
        /* ── Command 2a: LED on ── */
        led_set(true);
        const char *ack = "ACK: LED on\n";
        send(client_sock, ack, strlen(ack), 0);

    } else if (strcmp(buf, "led_off") == 0) {
        /* ── Command 2b: LED off ── */
        led_set(false);
        const char *ack = "ACK: LED off\n";
        send(client_sock, ack, strlen(ack), 0);

    } else if (strcmp(buf, "msg") == 0) {
        /* ── Command 3: send message ── */
        const char *response = "msg response sent\n";
        send(client_sock, response, strlen(response), 0);
        ESP_LOGI(TAG, "msg response sent to client");

    } else {
        const char *err = "ERR: unknown command\n";
        send(client_sock, err, strlen(err), 0);
        ESP_LOGW(TAG, "Unknown command: \"%s\"", buf);
    }
}

/* ── TCP server task ──────────────────────────────────────────────────────── */
static void tcp_server_task(void *pvParam)
{
    char rx_buf[RX_BUF_SIZE];

    struct sockaddr_in server_addr = {
        .sin_family      = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port        = htons(CONFIG_TCP_PORT),
    };

    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "socket() failed: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(listen_sock, (struct sockaddr *)&server_addr,
             sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "bind() failed: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    if (listen(listen_sock, 1) < 0) {
        ESP_LOGE(TAG, "listen() failed: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "TCP server listening on port %d", CONFIG_TCP_PORT);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);

        int client_sock = accept(listen_sock,
                                 (struct sockaddr *)&client_addr, &addr_len);
        if (client_sock < 0) {
            ESP_LOGE(TAG, "accept() failed: errno %d", errno);
            continue;
        }

        ESP_LOGI(TAG, "Client connected: %s",
                 inet_ntoa(client_addr.sin_addr));

        /* Receive loop — one command per recv(), newline-terminated */
        while (1) {
            int bytes = recv(client_sock, rx_buf, sizeof(rx_buf) - 1, 0);
            if (bytes <= 0) {
                ESP_LOGI(TAG, "Client disconnected (recv=%d)", bytes);
                break;
            }
            rx_buf[bytes] = '\0';
            process_command(client_sock, rx_buf);
        }

        close(client_sock);
    }
    /* unreachable */
    close(listen_sock);
    vTaskDelete(NULL);
}

/* ── app_main ─────────────────────────────────────────────────────────────── */
void app_main(void)
{
    /* NVS — required by WiFi driver */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    led_init();

    /* Blink LED twice to signal boot */
    for (int i = 0; i < 2; i++) {
        led_set(true);
        vTaskDelay(pdMS_TO_TICKS(150));
        led_set(false);
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    if (!wifi_init_sta()) {
        /* Fast-blink forever to signal WiFi failure */
        ESP_LOGE(TAG, "Halting — WiFi unavailable");
        while (1) {
            led_set(true);  vTaskDelay(pdMS_TO_TICKS(100));
            led_set(false); vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    /* Slow-blink 3× to signal successful WiFi connection */
    for (int i = 0; i < 3; i++) {
        led_set(true);  vTaskDelay(pdMS_TO_TICKS(300));
        led_set(false); vTaskDelay(pdMS_TO_TICKS(300));
    }

    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
}