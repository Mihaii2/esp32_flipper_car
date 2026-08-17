#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/string.h>

#define TAG "HIL_FLIPPER_FIRMWARE"

// Network Configuration (Make sure to match your network credentials)
#define WIFI_SSID       "My_Redmi"
#define WIFI_PASS       "formula1"
#define AGENT_IP        "10.87.8.200" // Your Laptop's LAN IP
#define AGENT_PORT      "8888"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ ESP_LOGW(TAG, "Soft fail on line %d: %d.",__LINE__,(int)temp_rc);}}

// Micro-ROS entities
static rcl_publisher_t cmd_vel_pub;
static rcl_subscription_t imu_sub;
static rcl_subscription_t cmd_sub;
static rcl_timer_t control_timer;

static geometry_msgs__msg__Twist twist_msg;
static sensor_msgs__msg__Imu imu_msg;
static std_msgs__msg__String cmd_msg;
static char cmd_buffer[32];

// Controller State
static bool is_upside_down = false;
static char current_command[32] = "STOP";
static const float base_speed = 0.5f;
static const float spin_omega = 2.5f;
static const float turn_omega = 1.25f;

static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
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

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    

    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI(TAG, "Wi-Fi Connected successfully.");
}

void cmd_callback(const void * msgin) {
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    if (msg != NULL && msg->data.data != NULL) {
        strncpy(current_command, msg->data.data, sizeof(current_command) - 1);
        current_command[sizeof(current_command) - 1] = '\0';
    }
}

void imu_callback(const void * msgin) {
    const sensor_msgs__msg__Imu * msg = (const sensor_msgs__msg__Imu *)msgin;
    double qx = msg->orientation.x;
    double qy = msg->orientation.y;
    double qz = msg->orientation.z;
    double qw = msg->orientation.w;

    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    double roll = atan2(sinr_cosp, cosr_cosp);

    is_upside_down = (fabs(roll) > (M_PI / 2.0));
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {
        float flip_drive = is_upside_down ? -1.0f : 1.0f;
        float flip_steer = is_upside_down ? -1.0f : 1.0f;

        memset(&twist_msg, 0, sizeof(geometry_msgs__msg__Twist));

        if (strcmp(current_command, "FORWARD") == 0) {
            twist_msg.linear.x = base_speed * flip_drive;
        } else if (strcmp(current_command, "REVERSE") == 0) {
            twist_msg.linear.x = -base_speed * flip_drive;
        } else if (strcmp(current_command, "SPIN_LEFT") == 0) {
            twist_msg.angular.z = spin_omega * flip_steer;
        } else if (strcmp(current_command, "SPIN_RIGHT") == 0) {
            twist_msg.angular.z = -spin_omega * flip_steer;
        } else if (strcmp(current_command, "FWD_LEFT") == 0) {
            twist_msg.linear.x = (base_speed * 0.75f) * flip_drive;
            twist_msg.angular.z = turn_omega * flip_steer;
        } else if (strcmp(current_command, "FWD_RIGHT") == 0) {
            twist_msg.linear.x = (base_speed * 0.75f) * flip_drive;
            twist_msg.angular.z = -turn_omega * flip_steer;
        } else if (strcmp(current_command, "REV_LEFT") == 0) {
            twist_msg.linear.x = -(base_speed * 0.75f) * flip_drive;
            twist_msg.angular.z = -turn_omega * flip_steer;
        } else if (strcmp(current_command, "REV_RIGHT") == 0) {
            twist_msg.linear.x = -(base_speed * 0.75f) * flip_drive;
            twist_msg.angular.z = turn_omega * flip_steer;
        }

        RCSOFTCHECK(rcl_publish(&cmd_vel_pub, &twist_msg, NULL));
    }
}

void micro_ros_task(void * arg) {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

    RCCHECK(rcl_init_options_init(&init_options, allocator));

    // Configure micro-ROS network transport options directly
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(AGENT_IP, AGENT_PORT, rmw_options));

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "esp32_hil_controller", "", &support));

    // Publisher
    RCCHECK(rclc_publisher_init_default(
        &cmd_vel_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));

    // Subscriptions
    RCCHECK(rclc_subscription_init_default(
        &imu_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu"));

    RCCHECK(rclc_subscription_init_default(
        &cmd_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/flipper/command"));

    cmd_msg.data.data = cmd_buffer;
    cmd_msg.data.capacity = sizeof(cmd_buffer);
    cmd_msg.data.size = 0;

    // Timer (50ms = 20Hz)
    RCCHECK(rclc_timer_init_default2(
        &control_timer,
        &support,
        RCL_MS_TO_NS(50),
        timer_callback,
        true));

    // Executor (2 subscriptions + 1 timer = 3 handles)
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &imu_sub, &imu_msg, &imu_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmd_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();
    xTaskCreate(micro_ros_task, "uros_task", 4096 * 4, NULL, 5, NULL);
}