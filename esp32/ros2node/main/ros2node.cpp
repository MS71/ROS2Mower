//#define LOG_LOCAL_LEVEL ESP_LOG_INFO

static const char* TAG = "R2N";

#include "sdkconfig.h"
#include <exception>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "../components/http_server/my_http_server.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp32/rom/uart.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_spiffs.h"
#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "esp_wifi.h"
#include "i2chandler.h"
#include "nvs_flash.h"
#include "sdmmc_cmd.h"

#include "ros2node.h"

#include <stdio.h>
#include <unistd.h>

extern "C" 
{
#include <geometry_msgs/msg/point32.h>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/transform.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rclc_lifecycle/rclc_lifecycle.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joy.h>
#include <std_msgs/msg/header.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int64.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/string.h>

#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_uros/options.h>
}

#define CONFIG_MICRO_ROS_AGENT_IP "192.168.1.85\0"
#define CONFIG_MICRO_ROS_AGENT_PORT "8888\0"

#define ROS2_PUB(_type_, _name_)  \
    rcl_publisher_t pub_##_name_; \
    _type_ msg_##_name_;

#define ROS2_SUB(_type_, _name_)     \
    rcl_subscription_t sub_##_name_; \
    _type_ msg_##_name_;

static struct
{
    uint8_t active;
    TaskHandle_t taskhandle;

    struct
    {
        ROS2_PUB(std_msgs__msg__Int8, wifi_rssi);
        //ROS2_PUB(std_msgs__msg__Float32, ubat);
        rcl_publisher_t pub_ubat;
#if 0        
        ROS2_PUB(std_msgs__msg__Float32, iout);
        ROS2_PUB(std_msgs__msg__Float32, isolar);
        ROS2_PUB(std_msgs__msg__Float32, icharge);
        ROS2_PUB(std_msgs__msg__Int64, motor_l_enc);
        ROS2_PUB(std_msgs__msg__Int16, motor_l_pid_sv);
        ROS2_PUB(std_msgs__msg__Int16, motor_l_pid_ov);
        ROS2_PUB(std_msgs__msg__Int16, motor_l_pid_pv);
        ROS2_PUB(std_msgs__msg__Int64, motor_r_enc);
        ROS2_PUB(std_msgs__msg__Int16, motor_r_pid_sv);
        ROS2_PUB(std_msgs__msg__Int16, motor_r_pid_ov);
        ROS2_PUB(std_msgs__msg__Int16, motor_r_pid_pv);
#endif
        //ROS2_PUB(geometry_msgs__msg__TransformStamped, odom_tf);
        //ROS2_PUB(sensor_msgs__msg__Imu, imu);
#ifdef I2CROS2SENSORDATA_USE_NAV_MSG_ODOMETRY    
        rcl_publisher_t pub_odom_tf;
#endif
#ifdef I2CROS2SENSORDATA_USE_GEOMETRY_MSG_POSE_2D    
        rcl_publisher_t pub_pose_2d;
#endif
        rcl_publisher_t pub_imu;
#ifdef CONFIG_ROS2NODE_HW_ROS2ZUMO
        rcl_publisher_t pub_range[I2CROS2SENSORDATA_NUM_RANGE];
#endif
    } pub;
    struct
    {
        ROS2_SUB(geometry_msgs__msg__Twist, cmd_vel);
        ROS2_SUB(sensor_msgs__msg__Joy, joy);
    } sub;
} r2n_md = {};

#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if((temp_rc != RCL_RET_OK))                                                      \
        {                                                                                \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            vTaskDelete(NULL);                                                           \
        }                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if((temp_rc != RCL_RET_OK))                                                        \
        {                                                                                  \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }

void rosString(rosidl_runtime_c__String* s, const char* str)
{
    int n = strlen(str) + 1;
    s->data = (char*)malloc(n);
    s->data[n] = 0;
    s->size = n;
    strcpy(s->data, str);
}

void r2n_cmd_vel_callback(const void* msgin)
{
    const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
    if(msg != NULL)
    {
        ESP_LOGW(TAG, "r2n_cmd_vel_callback()");
    }
}

void r2n_joy_callback(const void* msgin)
{
    const sensor_msgs__msg__Joy* msg = (const sensor_msgs__msg__Joy*)msgin;
    if(msg != NULL)
    {
        int button_1 = msg->buttons.data[0];
        //int button_2 = msg->buttons.data[1];
        //int button_3 = msg->buttons.data[2];
        //int button_4 = msg->buttons.data[3];

        float axes_0 = msg->axes.data[0];
        float axes_1 = msg->axes.data[1];
        //float axes_2 = msg->axes.data[2];
        //float axes_3 = msg->axes.data[3];
        //float axes_4 = msg->axes.data[4];
        //float axes_5 = msg->axes.data[5];
#if 0      
      ESP_LOGW(TAG, "sensor_msgs__msg__Joy() %f,%f,%f,%f,%f,%f %d,%d,%d,%d ",
        axes_0,axes_1,axes_2,axes_3,axes_4,axes_5,
        button_1,button_2,button_3,button_4);
#endif
        if( button_1 == 1 )
        {
            i2c_set_cmd_vel(axes_1 / 50.0, 0, -axes_0 * 2 * M_PI / 20.0 /* rad/sec*/);
        }
    }
}

/**
 * @brief
 * @param timer
 * @param last_call_time
 */
void timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    (void)last_call_time;
    if(timer != NULL)
    {
#if 0        
        rcl_clock_t* clock;
        rcl_timer_clock(timer, &clock);
        rcl_time_point_value_t now;
        clock->get_now(NULL, &now);
#endif
        // I2C sensor data
        I2CROS2SensorData* pData;
        
        pData = i2c_lock_data();
        if(pData != NULL)
        {
            if(pData->msg_ubat_valid == true)
            {
                std_msgs__msg__Float32 msg = pData->msg_ubat;
                pData->msg_ubat_valid = false;
                i2c_release_data();
                RCSOFTCHECK(rcl_publish(&r2n_md.pub.pub_ubat, &msg, NULL));
            }
            else
            {
                i2c_release_data();
            }
        }

        pData = i2c_lock_data();
        if(pData != NULL)
        {
#ifdef I2CROS2SENSORDATA_USE_NAV_MSG_ODOMETRY    
            if(pData->msg_odom_tf_valid == true)
            {
                nav_msgs__msg__Odometry msg = pData->msg_odom_tf;
                pData->msg_odom_tf_valid = false;
                i2c_release_data();
                //RCSOFTCHECK(rcl_publish(&r2n_md.pub.pub_odom_tf, &msg, NULL));
            }
            else
            {
                i2c_release_data();                
            }
#endif
#ifdef I2CROS2SENSORDATA_USE_GEOMETRY_MSG_POSE_2D
            if(pData->msg_pose_2d_valid == true)
            {
                //geometry_msgs__msg__Pose2D msg = pData->msg_pose_2d;
                pData->msg_pose_2d_valid = false;
                i2c_release_data();
                RCSOFTCHECK(rcl_publish(&r2n_md.pub.pub_pose_2d, &pData->msg_pose_2d, NULL));
            }
            else
            {
                i2c_release_data();                
            }
#endif
        }

        pData = i2c_lock_data();
        if(pData != NULL)
        {
            if(pData->msg_imu_valid == true)
            {
                sensor_msgs__msg__Imu msg = pData->msg_imu;
                pData->msg_imu_valid = false;
                i2c_release_data();
                RCSOFTCHECK(rcl_publish(&r2n_md.pub.pub_imu, &msg, NULL));
            }
            else
            {
                i2c_release_data();
            }
        }

        for(int i = 0; i < I2CROS2SENSORDATA_NUM_RANGE; i++)
        {
            pData = i2c_lock_data();
            if(pData != NULL)
            {
                if(pData->msg_range_valid[i] == true)
                {
                    sensor_msgs__msg__Range msg = pData->msg_range[i];
                    pData->msg_range_valid[i] = false;
                    i2c_release_data();
                    //ESP_LOGI(TAG, "i2c_lidar_handle %d range_mm=%f", i, msg.range);
                    RCSOFTCHECK(rcl_publish(&r2n_md.pub.pub_range[i], &msg, NULL));
                }
                else
                {
                    i2c_release_data();
                }
            }
        }

        // WIFI
        {
            wifi_ap_record_t ap_info;
            if(ESP_OK == esp_wifi_sta_get_ap_info(&ap_info))
            {
                r2n_md.pub.msg_wifi_rssi.data = ap_info.rssi;
                RCSOFTCHECK(rcl_publish(&r2n_md.pub.pub_wifi_rssi, &r2n_md.pub.msg_wifi_rssi, NULL));
            }
        }
    }
}

  
// lifecycle callback
rcl_ret_t my_on_configure() {
  ESP_LOGW(TAG, "my_lifecycle_node: on_configure() callback called.");
  return RCL_RET_OK;
}

/**
 * @brief
 * @param param
 */
static void r2n_task(void* param)
{
    ESP_LOGI(TAG, "ros2node_task() ...");
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    //usleep(5000);

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));

    // create init_options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, ROS2_NODENAME, "", &support));

    // create timer,
    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    const unsigned int timer_timeout = 100;
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

#if 0
    // rcl state machine
    rcl_lifecycle_state_machine_t state_machine =
        rcl_lifecycle_get_zero_initialized_state_machine();
  
    // create the lifecycle node
    rclc_lifecycle_node_t my_lifecycle_node;
    rcl_ret_t rc = rclc_make_node_a_lifecycle_node(
      &my_lifecycle_node,
      &node,
      &state_machine,
      &allocator);
  
    // register callbacks
    //rclc_lifecycle_register_on_configure(&my_lifecycle_node, &my_on_configure);
#endif
  
    // create publisher
    RCCHECK(rclc_publisher_init_default(&r2n_md.pub.pub_wifi_rssi, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), ROS2_NODENAME "/wifi_rssi"));
        
    RCCHECK(rclc_publisher_init_default(
        &r2n_md.pub.pub_ubat, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), ROS2_NODENAME "/ubat"));
#if 0        
    RCCHECK(rclc_publisher_init_default(
        &r2n_md.pub.pub_iout, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), ROS2_NODENAME "/iout"));
    RCCHECK(rclc_publisher_init_default(
        &r2n_md.pub.pub_isolar, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), ROS2_NODENAME "/isolar"));
    RCCHECK(rclc_publisher_init_default(
        &r2n_md.pub.pub_icharge, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), ROS2_NODENAME "/icharge"));
#endif


#ifdef I2CROS2SENSORDATA_USE_NAV_MSG_ODOMETRY    
    RCCHECK(rclc_publisher_init_default(&r2n_md.pub.pub_odom_tf, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), ROS2_NODENAME "/odom_tf"));
#endif
#ifdef I2CROS2SENSORDATA_USE_GEOMETRY_MSG_POSE_2D    
    RCCHECK(rclc_publisher_init_default(&r2n_md.pub.pub_pose_2d, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D), ROS2_NODENAME "/pose2d"));
#endif

    RCCHECK(rclc_publisher_init_default(
        &r2n_md.pub.pub_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), ROS2_NODENAME "/imu"));

#ifdef CONFIG_ROS2NODE_HW_ROS2ZUMO
    for(int i = 0; i < I2CROS2SENSORDATA_NUM_RANGE; i++)
    {
        char tmpstr[64];
        sprintf(tmpstr, ROS2_NODENAME "/range_%d", i);
        RCCHECK(rclc_publisher_init_default(
            &r2n_md.pub.pub_range[i], &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), tmpstr));
    }
#endif

#if 1
    {
        rcl_ret_t rc;
        ESP_LOGI(TAG,
            "ros2node_task() init sub "
            "/" ROS2_NODENAME "/joy"
            " ...");
        r2n_md.sub.sub_joy = rcl_get_zero_initialized_subscription();
        rc = rclc_subscription_init_default(
            &r2n_md.sub.sub_joy, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy), "/" ROS2_NODENAME "/joy");
        if(rc != RCL_RET_OK)
        {
            ESP_LOGE(TAG,
                "ros2node_task() Failed to init subscriber "
                "/" ROS2_NODENAME "/joy");
            RCCHECK(rc);
        }
    }
#endif

    // create executor
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

    unsigned int rcl_wait_timeout = 50; // in ms
    RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

    {
        rcl_ret_t rc;
        // sensor_msgs__msg__Joy__init(&r2n_md.sub.msg_joy);
        std_msgs__msg__Header__init(&r2n_md.sub.msg_joy.header);
        r2n_md.sub.msg_joy.header.frame_id.data = (char*)malloc(32);
        r2n_md.sub.msg_joy.header.frame_id.size = 0;
        r2n_md.sub.msg_joy.header.frame_id.capacity = 32;
        rosidl_runtime_c__float__Sequence__init(&r2n_md.sub.msg_joy.axes, 32);
        rosidl_runtime_c__int32__Sequence__init(&r2n_md.sub.msg_joy.buttons, 32);
        rc = rclc_executor_add_subscription(
            &executor, &r2n_md.sub.sub_joy, &r2n_md.sub.msg_joy, &r2n_joy_callback, ON_NEW_DATA);
        if(rc != RCL_RET_OK)
        {
            ESP_LOGE(TAG,
                "ros2node_task() Failed to add subscriber "
                "/" ROS2_NODENAME "/joy");
            RCCHECK(rc);
        }
        else
        {
            ESP_LOGI(TAG,
                "ros2node_task() "
                "/" ROS2_NODENAME "/joy"
                " ok");
        }
    }

    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    while(r2n_md.active == 1)
    {
        rclc_executor_spin_some(&executor, 10);
        usleep(1000);
    }

    rclc_executor_spin_some(&executor, 100);
    usleep(100000);

    sensor_msgs__msg__Joy__fini(&r2n_md.sub.msg_joy);

    ESP_LOGI(TAG, "ros2node_task() ... done");
    vTaskDelete(NULL);
}


/**
 * @brief
 */
uint8_t ros2node_connected()
{
    if( rcl_publisher_is_valid(&r2n_md.pub.pub_ubat) )
    {
#if 0        
        size_t subscription_count = 0;
        rcl_ret_t ret = 
            rcl_publisher_get_subscription_count(&r2n_md.pub.pub_ubat, &subscription_count);
        if( ret == RCL_RET_OK )
        {
            if( subscription_count != 0 )
            {
                return 1;
            }
        }
#endif        
    }
    return 0;
}

/**
 * @brief
 */
void ros2node_init()
{
}

/**
 * @brief
 */
void ros2node_start()
{
    // ESP_LOGI(TAG, "ros2node_start");
    r2n_md.active = 1;
    r2n_md.taskhandle = (TaskHandle_t)xTaskCreate(&r2n_task, "r2n_task", 8*8192, NULL, 5, NULL);
}

/**
 * @brief
 */
void ros2node_stop()
{
    // ESP_LOGI(TAG, "ros2node_stop");
    r2n_md.active = 0;
    vTaskDelay(50 / portTICK_PERIOD_MS);
    vTaskDelete(r2n_md.taskhandle);
    r2n_md.taskhandle = NULL;
}

/**
 * EOF
 */
