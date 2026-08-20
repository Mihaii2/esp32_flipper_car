#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>

#define TAG "DIGITAL_TWIN_FLIPPER"

#define TARGET_SSID     "laptop_flipper"
#define TARGET_PASS     "mafiosu123"
#define AGENT_IP        "10.42.0.1"
#define AGENT_PORT      "8888"

// Hardware Pin Definitions
#define PIN_IN1         19   // Motor A Left Inverted FWD
#define PIN_IN2         18   // Motor A Left Inverted REV
#define PIN_IN3         22   // Motor B Right FWD
#define PIN_IN4         23   // Motor B Right REV
#define PIN_TILT        4    // SW-520D Tilt Sensor

// PWM Configuration: 1 kHz for high low-end torque
#define PWM_TIMER       LEDC_TIMER_0
#define PWM_MODE        LEDC_LOW_SPEED_MODE
#define PWM_DUTY_RES    LEDC_TIMER_8_BIT
#define PWM_FREQ_HZ     1000

#define CH_A_FWD        LEDC_CHANNEL_0
#define CH_A_REV        LEDC_CHANNEL_1
#define CH_B_FWD        LEDC_CHANNEL_2
#define CH_B_REV        LEDC_CHANNEL_3

#define WINDOW_SIZE     10 

// Sim velocities
static const float base_speed = 0.5f;
static const float spin_omega = 2.5f;
static const float turn_omega = 1.25f;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ ESP_LOGE(TAG, "Failed status line %d: %d.",__LINE__,(int)temp_rc); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ ESP_LOGW(TAG, "Soft fail line %d: %d.",__LINE__,(int)temp_rc);}}

static rcl_publisher_t cmd_vel_pub;
static geometry_msgs__msg__Twist twist_msg;

static char current_command[32] = "STOP";
static volatile float current_gear_multiplier = 0.50f; // Starts at Gear 1 (50% power)
static volatile bool is_upside_down = false;

static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

extern uint8_t temprature_sens_read(void);
static float read_esp32_internal_temp(void) {
    return (float)(temprature_sens_read() - 32) / 1.8f;
}

static void force_motors_hard_stop(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_IN1) | (1ULL << PIN_IN2) | (1ULL << PIN_IN3) | (1ULL << PIN_IN4),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    gpio_set_level(PIN_IN1, 0);
    gpio_set_level(PIN_IN2, 0);
    gpio_set_level(PIN_IN3, 0);
    gpio_set_level(PIN_IN4, 0);
}

static void init_hardware(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_TILT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = PWM_MODE,
        .duty_resolution  = PWM_DUTY_RES,
        .timer_num        = PWM_TIMER,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ch_cfg[4] = {
        { .channel = CH_A_FWD, .gpio_num = PIN_IN1, .speed_mode = PWM_MODE, .timer_sel = PWM_TIMER, .duty = 0 },
        { .channel = CH_A_REV, .gpio_num = PIN_IN2, .speed_mode = PWM_MODE, .timer_sel = PWM_TIMER, .duty = 0 },
        { .channel = CH_B_FWD, .gpio_num = PIN_IN3, .speed_mode = PWM_MODE, .timer_sel = PWM_TIMER, .duty = 0 },
        { .channel = CH_B_REV, .gpio_num = PIN_IN4, .speed_mode = PWM_MODE, .timer_sel = PWM_TIMER, .duty = 0 }
    };

    for (int i = 0; i < 4; i++) {
        ledc_channel_config(&ch_cfg[i]);
    }
}

static void set_motor_speeds(int left_speed, int right_speed) {
    if (left_speed >= 0) {
        ledc_set_duty(PWM_MODE, CH_A_FWD, left_speed);
        ledc_set_duty(PWM_MODE, CH_A_REV, 0);
    } else {
        ledc_set_duty(PWM_MODE, CH_A_FWD, 0);
        ledc_set_duty(PWM_MODE, CH_A_REV, -left_speed);
    }

    if (right_speed >= 0) {
        ledc_set_duty(PWM_MODE, CH_B_FWD, right_speed);
        ledc_set_duty(PWM_MODE, CH_B_REV, 0);
    } else {
        ledc_set_duty(PWM_MODE, CH_B_FWD, 0);
        ledc_set_duty(PWM_MODE, CH_B_REV, -right_speed);
    }

    ledc_update_duty(PWM_MODE, CH_A_FWD);
    ledc_update_duty(PWM_MODE, CH_A_REV);
    ledc_update_duty(PWM_MODE, CH_B_FWD);
    ledc_update_duty(PWM_MODE, CH_B_REV);
}

void motor_control_task(void *pvParameters) {
    uint32_t temp_ticks = 0;
    uint8_t history[WINDOW_SIZE] = {0};
    int head = 0;
    bool stable_flipped = false;

    while (1) {
        history[head] = (gpio_get_level(PIN_TILT) == 1) ? 1 : 0;
        head = (head + 1) % WINDOW_SIZE;

        int votes = 0;
        for (int i = 0; i < WINDOW_SIZE; i++) votes += history[i];

        if (!stable_flipped && votes >= 8) {
            stable_flipped = true;
            ESP_LOGW(TAG, "ORIENTATION -> FLIPPED (UPSIDE DOWN)");
        } else if (stable_flipped && votes <= 2) {
            stable_flipped = false;
            ESP_LOGW(TAG, "ORIENTATION -> NORMAL (UPRIGHT)");
        }
        is_upside_down = stable_flipped;

        if (++temp_ticks >= 250) {
            temp_ticks = 0;
            ESP_LOGI(TAG, "Temp: %.1f °C | Gear: %.0f%% | Flipped: %s", 
                     read_esp32_internal_temp(), current_gear_multiplier * 100.0f, is_upside_down ? "YES" : "NO");
        }

        int full_duty = (int)(255 * current_gear_multiplier);
        int turn_duty = (int)(180 * current_gear_multiplier);

        int target_left = 0;
        int target_right = 0;

        if (strcmp(current_command, "FORWARD") == 0) { target_left = full_duty; target_right = full_duty; }
        else if (strcmp(current_command, "REVERSE") == 0) { target_left = -full_duty; target_right = -full_duty; }
        else if (strcmp(current_command, "SPIN_LEFT") == 0) { target_left = -full_duty; target_right = full_duty; }
        else if (strcmp(current_command, "SPIN_RIGHT") == 0) { target_left = full_duty; target_right = -full_duty; }
        else if (strcmp(current_command, "FWD_LEFT") == 0) { target_left = turn_duty; target_right = full_duty; }
        else if (strcmp(current_command, "FWD_RIGHT") == 0) { target_left = full_duty; target_right = turn_duty; }
        else if (strcmp(current_command, "REV_LEFT") == 0) { target_left = -turn_duty; target_right = -full_duty; }
        else if (strcmp(current_command, "REV_RIGHT") == 0) { target_left = -full_duty; target_right = -turn_duty; }

        if (is_upside_down) {
            target_left = -target_left;
            target_right = -target_right;
        }

        set_motor_speeds(target_left, target_right);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void) {
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = TARGET_SSID,
            .password = TARGET_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = { .capable = true, .required = false },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(TAG, "Connecting to Laptop Hotspot: %s...", TARGET_SSID);
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "Wi-Fi Connected successfully!");
}

void cmd_callback(const void * msgin) {
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    if (msg != NULL && msg->data.data != NULL) {
        const char *cmd = msg->data.data;
        // Remapped to overcome static friction
        if (strcmp(cmd, "GEAR_1") == 0) current_gear_multiplier = 0.50f;      // 50%
        else if (strcmp(cmd, "GEAR_2") == 0) current_gear_multiplier = 0.68f; // 68%
        else if (strcmp(cmd, "GEAR_3") == 0) current_gear_multiplier = 0.84f; // 84%
        else if (strcmp(cmd, "GEAR_4") == 0) current_gear_multiplier = 1.00f; // 100%
        else {
            strncpy(current_command, cmd, sizeof(current_command) - 1);
            current_command[sizeof(current_command) - 1] = '\0';
        }
    }
}

void sim_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {
        float flip = is_upside_down ? -1.0f : 1.0f;
        float scale = current_gear_multiplier;

        memset(&twist_msg, 0, sizeof(geometry_msgs__msg__Twist));

        if (strcmp(current_command, "FORWARD") == 0) twist_msg.linear.x = (base_speed * scale) * flip;
        else if (strcmp(current_command, "REVERSE") == 0) twist_msg.linear.x = -(base_speed * scale) * flip;
        else if (strcmp(current_command, "SPIN_LEFT") == 0) twist_msg.angular.z = (spin_omega * scale) * flip;
        else if (strcmp(current_command, "SPIN_RIGHT") == 0) twist_msg.angular.z = -(spin_omega * scale) * flip;
        else if (strcmp(current_command, "FWD_LEFT") == 0) {
            twist_msg.linear.x = (base_speed * 0.75f * scale) * flip;
            twist_msg.angular.z = (turn_omega * scale) * flip;
        } else if (strcmp(current_command, "FWD_RIGHT") == 0) {
            twist_msg.linear.x = (base_speed * 0.75f * scale) * flip;
            twist_msg.angular.z = -(turn_omega * scale) * flip;
        } else if (strcmp(current_command, "REV_LEFT") == 0) {
            twist_msg.linear.x = -(base_speed * 0.75f * scale) * flip;
            twist_msg.angular.z = -(turn_omega * scale) * flip;
        } else if (strcmp(current_command, "REV_RIGHT") == 0) {
            twist_msg.linear.x = -(base_speed * 0.75f * scale) * flip;
            twist_msg.angular.z = (turn_omega * scale) * flip;
        }

        RCSOFTCHECK(rcl_publish(&cmd_vel_pub, &twist_msg, NULL));
    }
}

void micro_ros_task(void * arg) {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

    RCCHECK(rcl_init_options_init(&init_options, allocator));

    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(AGENT_IP, AGENT_PORT, rmw_options));

    while (rmw_uros_ping_agent_options(100, 2, rmw_options) != RMW_RET_OK) {
        ESP_LOGW(TAG, "Waiting for agent at %s:%s...", AGENT_IP, AGENT_PORT);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "Connected to Agent! Creating nodes...");
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "esp32_digital_twin", "", &support));

    RCCHECK(rclc_publisher_init_best_effort(
        &cmd_vel_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));

    rcl_subscription_t cmd_sub;
    std_msgs__msg__String cmd_msg;
    char cmd_buffer[32];

    RCCHECK(rclc_subscription_init_best_effort(
        &cmd_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/flipper/command"));

    cmd_msg.data.data = cmd_buffer;
    cmd_msg.data.capacity = sizeof(cmd_buffer);
    cmd_msg.data.size = 0;

    rcl_timer_t sim_timer;
    RCCHECK(rclc_timer_init_default2(&sim_timer, &support, RCL_MS_TO_NS(50), sim_timer_callback, true));

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmd_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &sim_timer));

    ESP_LOGI(TAG, "micro-ROS executor running smoothly!");

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    force_motors_hard_stop();

    nvs_flash_init();
    init_hardware();
    wifi_init_sta();

    xTaskCreatePinnedToCore(motor_control_task, "motor_task", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(micro_ros_task, "uros_task", 4096 * 4, NULL, 5, NULL, 0);
}