/*
 * WiFi Robot Controller — ESP-IDF
 * TB6612FNG dual motor driver
 * Motor A = Left, Motor B = Right
 *
 * HTTP endpoints:
 *   POST /forward?speed=80
 *   POST /backward?speed=80
 *   POST /left?speed=80          (spin left in place)
 *   POST /right?speed=80         (spin right in place)
 *   POST /turn?dir=left&inner=40&outer=80   (wide arc turn)
 *   POST /stop
 *   GET  /status
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"

/* ── WiFi credentials ─────────────────────────────────── */
#define WIFI_SSID      "YOUR_SSID"
#define WIFI_PASSWORD  "YOUR_PASSWORD"

/* ── TB6612FNG pin assignments ────────────────────────── */
/* Motor A (Left)  */
#define MOTOR_A_IN1   GPIO_NUM_25
#define MOTOR_A_IN2   GPIO_NUM_26
#define MOTOR_A_PWM   GPIO_NUM_27   /* PWMA */

/* Motor B (Right) */
#define MOTOR_B_IN1   GPIO_NUM_32
#define MOTOR_B_IN2   GPIO_NUM_33
#define MOTOR_B_PWM   GPIO_NUM_14   /* PWMB */

/* Driver standby — pull HIGH to enable */
#define MOTOR_STBY    GPIO_NUM_12

/* ── LEDC (PWM) configuration ─────────────────────────── */
#define LEDC_TIMER       LEDC_TIMER_0
#define LEDC_MODE        LEDC_LOW_SPEED_MODE
#define LEDC_FREQ_HZ     20000
#define LEDC_RESOLUTION  LEDC_TIMER_8_BIT   /* 0-255 */

#define LEDC_CHANNEL_A   LEDC_CHANNEL_0
#define LEDC_CHANNEL_B   LEDC_CHANNEL_1

/* ── Defaults ─────────────────────────────────────────── */
#define DEFAULT_SPEED    80   /* 0-100 % */

static const char *TAG = "robot";

/* ─────────────────────────────────────────────────────── */
/*  Motor driver                                           */
/* ─────────────────────────────────────────────────────── */

typedef enum { FORWARD, BACKWARD, BRAKE, COAST } motor_dir_t;

static void pwm_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = LEDC_RESOLUTION,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch_a = {
        .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_A,
        .timer_sel  = LEDC_TIMER, .gpio_num = MOTOR_A_PWM,
        .duty = 0, .hpoint = 0,
    };
    ledc_channel_config_t ch_b = {
        .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_B,
        .timer_sel  = LEDC_TIMER, .gpio_num = MOTOR_B_PWM,
        .duty = 0, .hpoint = 0,
    };
    ledc_channel_config(&ch_a);
    ledc_channel_config(&ch_b);
}

static void gpio_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << MOTOR_A_IN1) | (1ULL << MOTOR_A_IN2) |
                        (1ULL << MOTOR_B_IN1) | (1ULL << MOTOR_B_IN2) |
                        (1ULL << MOTOR_STBY),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    /* Enable driver (take out of standby) */
    gpio_set_level(MOTOR_STBY, 1);
}

/* speed: 0-100 (percent) */
static void set_motor_a(motor_dir_t dir, int speed)
{
    uint32_t duty = (uint32_t)((speed / 100.0f) * 255);

    switch (dir) {
        case FORWARD:
            gpio_set_level(MOTOR_A_IN1, 1);
            gpio_set_level(MOTOR_A_IN2, 0);
            break;
        case BACKWARD:
            gpio_set_level(MOTOR_A_IN1, 0);
            gpio_set_level(MOTOR_A_IN2, 1);
            break;
        case BRAKE:
            gpio_set_level(MOTOR_A_IN1, 1);
            gpio_set_level(MOTOR_A_IN2, 1);
            duty = 0;
            break;
        case COAST:
        default:
            gpio_set_level(MOTOR_A_IN1, 0);
            gpio_set_level(MOTOR_A_IN2, 0);
            duty = 0;
            break;
    }
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);
}

static void set_motor_b(motor_dir_t dir, int speed)
{
    uint32_t duty = (uint32_t)((speed / 100.0f) * 255);

    switch (dir) {
        case FORWARD:
            gpio_set_level(MOTOR_B_IN1, 1);
            gpio_set_level(MOTOR_B_IN2, 0);
            break;
        case BACKWARD:
            gpio_set_level(MOTOR_B_IN1, 0);
            gpio_set_level(MOTOR_B_IN2, 1);
            break;
        case BRAKE:
            gpio_set_level(MOTOR_B_IN1, 1);
            gpio_set_level(MOTOR_B_IN2, 1);
            duty = 0;
            break;
        case COAST:
        default:
            gpio_set_level(MOTOR_B_IN1, 0);
            gpio_set_level(MOTOR_B_IN2, 0);
            duty = 0;
            break;
    }
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
}

/* ─────────────────────────────────────────────────────── */
/*  Movement commands                                      */
/* ─────────────────────────────────────────────────────── */

static char g_state[32] = "stopped";

static void robot_forward(int speed)
{
    set_motor_a(FORWARD, speed);
    set_motor_b(FORWARD, speed);
    snprintf(g_state, sizeof(g_state), "forward@%d%%", speed);
    ESP_LOGI(TAG, "Forward speed=%d", speed);
}

static void robot_backward(int speed)
{
    set_motor_a(BACKWARD, speed);
    set_motor_b(BACKWARD, speed);
    snprintf(g_state, sizeof(g_state), "backward@%d%%", speed);
    ESP_LOGI(TAG, "Backward speed=%d", speed);
}

/* Spin left: left wheel back, right wheel forward */
static void robot_spin_left(int speed)
{
    set_motor_a(BACKWARD, speed);
    set_motor_b(FORWARD,  speed);
    snprintf(g_state, sizeof(g_state), "spin_left@%d%%", speed);
    ESP_LOGI(TAG, "Spin left speed=%d", speed);
}

/* Spin right: left wheel forward, right wheel back */
static void robot_spin_right(int speed)
{
    set_motor_a(FORWARD,  speed);
    set_motor_b(BACKWARD, speed);
    snprintf(g_state, sizeof(g_state), "spin_right@%d%%", speed);
    ESP_LOGI(TAG, "Spin right speed=%d", speed);
}

/*
 * Wide arc turn (forward):
 *   dir        = "left" | "right"
 *   inner_spd  = speed of the slower (inside) wheel  0-100
 *   outer_spd  = speed of the faster (outside) wheel 0-100
 */
static void robot_arc(const char *dir, int inner_spd, int outer_spd)
{
    if (strcmp(dir, "left") == 0) {
        /* Turning left: left wheel slower, right wheel faster */
        set_motor_a(FORWARD, inner_spd);
        set_motor_b(FORWARD, outer_spd);
    } else {
        /* Turning right: right wheel slower, left wheel faster */
        set_motor_a(FORWARD, outer_spd);
        set_motor_b(FORWARD, inner_spd);
    }
    snprintf(g_state, sizeof(g_state), "arc_%s i%d o%d", dir, inner_spd, outer_spd);
    ESP_LOGI(TAG, "Arc turn dir=%s inner=%d outer=%d", dir, inner_spd, outer_spd);
}

static void robot_stop(void)
{
    set_motor_a(BRAKE, 0);
    set_motor_b(BRAKE, 0);
    snprintf(g_state, sizeof(g_state), "stopped");
    ESP_LOGI(TAG, "Stop");
}

/* ─────────────────────────────────────────────────────── */
/*  HTTP helpers                                           */
/* ─────────────────────────────────────────────────────── */

/* Extract integer query param; returns default_val if absent */
static int query_int(httpd_req_t *req, const char *key, int default_val)
{
    char buf[16];
    if (httpd_query_key_value(req->uri + strcspn(req->uri, "?"),
                              key, buf, sizeof(buf)) == ESP_OK) {
        int v = atoi(buf);
        if (v < 0)   v = 0;
        if (v > 100) v = 100;
        return v;
    }
    return default_val;
}

/* Extract string query param; fills out (max len); returns true on success */
static bool query_str(httpd_req_t *req, const char *key,
                      char *out, size_t out_len)
{
    return httpd_query_key_value(req->uri + strcspn(req->uri, "?"),
                                 key, out, out_len) == ESP_OK;
}

static esp_err_t send_json(httpd_req_t *req, const char *json)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, json);
}

/* ─────────────────────────────────────────────────────── */
/*  HTTP handlers                                          */
/* ─────────────────────────────────────────────────────── */

static esp_err_t handler_forward(httpd_req_t *req)
{
    robot_forward(query_int(req, "speed", DEFAULT_SPEED));
    return send_json(req, "{\"ok\":true,\"cmd\":\"forward\"}");
}

static esp_err_t handler_backward(httpd_req_t *req)
{
    robot_backward(query_int(req, "speed", DEFAULT_SPEED));
    return send_json(req, "{\"ok\":true,\"cmd\":\"backward\"}");
}

static esp_err_t handler_left(httpd_req_t *req)
{
    robot_spin_left(query_int(req, "speed", DEFAULT_SPEED));
    return send_json(req, "{\"ok\":true,\"cmd\":\"spin_left\"}");
}

static esp_err_t handler_right(httpd_req_t *req)
{
    robot_spin_right(query_int(req, "speed", DEFAULT_SPEED));
    return send_json(req, "{\"ok\":true,\"cmd\":\"spin_right\"}");
}

static esp_err_t handler_turn(httpd_req_t *req)
{
    char dir[8] = "left";
    query_str(req, "dir", dir, sizeof(dir));
    int inner = query_int(req, "inner", 40);
    int outer = query_int(req, "outer", DEFAULT_SPEED);
    robot_arc(dir, inner, outer);
    char resp[96];
    snprintf(resp, sizeof(resp),
             "{\"ok\":true,\"cmd\":\"arc\",\"dir\":\"%s\",\"inner\":%d,\"outer\":%d}",
             dir, inner, outer);
    return send_json(req, resp);
}

static esp_err_t handler_stop(httpd_req_t *req)
{
    robot_stop();
    return send_json(req, "{\"ok\":true,\"cmd\":\"stop\"}");
}

static esp_err_t handler_status(httpd_req_t *req)
{
    char resp[128];
    snprintf(resp, sizeof(resp), "{\"ok\":true,\"state\":\"%s\"}", g_state);
    return send_json(req, resp);
}

/* CORS pre-flight */
static esp_err_t handler_options(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET,POST,OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* ─────────────────────────────────────────────────────── */
/*  HTTP server startup                                    */
/* ─────────────────────────────────────────────────────── */

#define ROUTE(uri_str, m, fn) \
    { .uri = uri_str, .method = m, .handler = fn, .user_ctx = NULL }

static void start_http_server(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 16;

    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &cfg));

    const httpd_uri_t routes[] = {
        ROUTE("/forward",  HTTP_POST,    handler_forward),
        ROUTE("/backward", HTTP_POST,    handler_backward),
        ROUTE("/left",     HTTP_POST,    handler_left),
        ROUTE("/right",    HTTP_POST,    handler_right),
        ROUTE("/turn",     HTTP_POST,    handler_turn),
        ROUTE("/stop",     HTTP_POST,    handler_stop),
        ROUTE("/status",   HTTP_GET,     handler_status),
        /* wildcard OPTIONS for CORS */
        ROUTE("/*",        HTTP_OPTIONS, handler_options),
    };

    for (int i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        httpd_register_uri_handler(server, &routes[i]);
    }

    ESP_LOGI(TAG, "HTTP server started");
}

/* ─────────────────────────────────────────────────────── */
/*  WiFi                                                   */
/* ─────────────────────────────────────────────────────── */

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected — retrying…");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "===========================================");
        ESP_LOGI(TAG, "Robot IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "===========================================");
        start_http_server();
    }
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               wifi_event_handler, NULL));

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to SSID: %s", WIFI_SSID);
}

/* ─────────────────────────────────────────────────────── */
/*  Entry point                                            */
/* ─────────────────────────────────────────────────────── */

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    gpio_init();
    pwm_init();
    robot_stop();   /* safe initial state */

    wifi_init();
    /* HTTP server starts in the IP_EVENT callback once we have an address */
}