// Copyright 2015-2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
#include "ota_server.h"

#ifdef CONFIG_ENABLE_ROS2
#include <ros2esp.h>
#endif

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_vfs_fat.h"
#include "esp_wifi.h"
#include "esp_sleep.h"
#include "i2chandler.h"
#include "nvs_flash.h"
#include "sdmmc_cmd.h"
#include "rom/uart.h"
#include "esp_pm.h"

extern "C" {
#include "ulp-util.h" // my ulp_init(), ulp_start()
}

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "ssd1306.h"
#include "ssd1306_default_if.h"
#include "ssd1306_draw.h"
#include "ssd1306_font.h"

#include "gps.h"
#include "cam.h"

#include "console.h"

#define ENABLE_SLEEP_MODE

// This example can use SDMMC and SPI peripherals to communicate with SD card.
// By default, SDMMC peripheral is used.
// To enable SPI mode, uncomment the following line:

#undef USE_SPI_MODE

// When testing SD and SPI modes, keep in mind that once the card has been
// initialized in SPI mode, it can not be reinitialized in SD mode without
// toggling power to the card.

#ifdef USE_SPI_MODE
// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#define PIN_NUM_MISO 2
#define PIN_NUM_MOSI 15
#define PIN_NUM_CLK 14
#define PIN_NUM_CS 13
#endif // USE_SPI_MODE

static void handle_grayscale_pgm(http_context_t http_ctx, void* ctx);
static void handle_rgb_bmp(http_context_t http_ctx, void* ctx);
static void handle_rgb_bmp_stream(http_context_t http_ctx, void* ctx);
static void handle_jpg(http_context_t http_ctx, void* ctx);
static void handle_jpg_stream(http_context_t http_ctx, void* ctx);
static esp_err_t event_handler(void* ctx, system_event_t* event);
static void initialise_wifi(void);

sdmmc_card_t* card = NULL;
bool sdcard_ready = true;

bool camera_ready = true;

static const char* TAG = "MAIN";

EventGroupHandle_t s_wifi_event_group;
const int CONNECTED_BIT = BIT0;
ip4_addr_t s_ip_addr = {};
uint8_t s_ip_addr_changed = 1;

int ros2_sock = -1;

static void ota_server_task(void* param)
{
    xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    ota_server_start();
    vTaskDelete(NULL);
}

static uint8_t sd_test_buf[32768] = { 0 };
static void sd_test_task(void* param)
{
    if(sdcard_ready == true && card != NULL) {
	// Card has been initialized, print its properties
	sdmmc_card_print_info(stdout, card);

	while(1) {
	    // Use POSIX and C standard library functions to work with files.
	    // First create a file.
	    FILE* f = fopen("/sdcard/hello.bin", "w");
	    if(f == NULL) {
		ESP_LOGE(TAG, "Failed to open file for writing");
	    } else {
		uint32_t i = 0;
		uint32_t s = 1024 * 1000;
		int64_t t = esp_timer_get_time();
		while(i < s) {
		    fwrite(sd_test_buf, 1, sizeof(sd_test_buf), f);
		    i += sizeof(sd_test_buf);
		}
		if((esp_timer_get_time() - t) != 0) {
		    ESP_LOGI(TAG, "File written %dkB/s", (int)((1000 * i) / (esp_timer_get_time() - t)));
		}
		fclose(f);
	    }

#if 1
	    {
		f = fopen("/sdcard/hello.bin", "r");
		if(f == NULL) {
		    ESP_LOGE(TAG, "Failed to open file for reading");
		} else {
		    int64_t t = esp_timer_get_time();
		    size_t i = 0;
		    while(!feof(f)) {
			i += fread(sd_test_buf, 1, sizeof(sd_test_buf), f);
		    }
		    if((esp_timer_get_time() - t) != 0) {
			ESP_LOGI(TAG, "File read %dkB/s", (int)((1000 * i) / (esp_timer_get_time() - t)));
		    }
		    fclose(f);
		}
	    }
#endif
	    vTaskDelay(5000 / portTICK_PERIOD_MS);
	}
    }
    vTaskDelete(NULL);
}

#if 0

    switch (ip_protocol)
    {
        case UXR_IPv4:
            platform->poll_fd = socket(AF_INET, SOCK_DGRAM, 0);
            break;
        case UXR_IPv6:
            platform->poll_fd = socket(AF_INET6, SOCK_DGRAM, 0);
            break;
    }


    if (-1 != platform->poll_fd)
    {
        struct addrinfo hints;
        struct addrinfo* result;
        struct addrinfo* ptr;

        memset(&hints, 0, sizeof(hints));
        switch (ip_protocol)
        {
            case UXR_IPv4:
                hints.ai_family = AF_INET;
                break;
            case UXR_IPv6:
                hints.ai_family = AF_INET6;
                break;
        }
        hints.ai_socktype = SOCK_DGRAM;

        if (0 == getaddrinfo(ip, port, &hints, &result))
        {
            for (ptr = result; ptr != NULL; ptr = ptr->ai_next)
            {
                if (0 == connect(platform->poll_fd, ptr->ai_addr, ptr->ai_addrlen))
                {
                    rv = true;
                    break;
                }
            }
        }
        freeaddrinfo(result);
    }

#endif
static vprintf_like_t my_deflog = NULL;
static char mylog_linebuf[512];
static int mylog_fd = -1;
struct sockaddr_in destAddr;
int my_log(const char *format, va_list args)
{
	if( my_deflog != NULL )
	{
		//my_deflog(format,args);
	}

	vsnprintf (mylog_linebuf, sizeof(mylog_linebuf)-1, format, args);
	int n = strlen(mylog_linebuf);	
	if(n > 0 )
	{
#if 0
		if( mylog_fd == -1 )
		{
		}

#endif
		
#if 1		
		if( mylog_fd == -1 )
		{
			mylog_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
			if( mylog_fd != -1 )
			{
				char addr_str[128];
				int addr_family;
				int ip_protocol;
				destAddr.sin_addr.s_addr = inet_addr("192.168.1.63");
				destAddr.sin_family = AF_INET;
				destAddr.sin_port = htons(30000);
				addr_family = AF_INET;
				inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
			}
		}
		if( mylog_fd != -1 )
		{
			int bytes_sent = sendto(mylog_fd, (uint8_t*)&mylog_linebuf[0], n, 0, (struct sockaddr *)&destAddr, sizeof(destAddr));
			if (-1 != bytes_sent)
			{
				return n;
			}
			else
			{
				close(mylog_fd);
				mylog_fd = -1;
			}					
		}
#endif		
	}
	return 0;
}

extern "C" {
void app_main();
}

void app_main()
{
    /* Print chip information */
    //esp_log_level_set("wifi", ESP_LOG_WARN);
    //esp_log_level_set("gpio", ESP_LOG_WARN);

    ESP_LOGI(TAG, "init ULP ...");
    ulp_init();
    ulp_start();

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "This is ESP32 chip with %d CPU cores, WiFi%s%s, ", chip_info.cores,
        (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "", (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    ESP_LOGI(TAG, "silicon revision %d, ", chip_info.revision);

    ESP_LOGI(TAG, "%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    esp_err_t err = nvs_flash_init();
    if(err != ESP_OK) {
	ESP_ERROR_CHECK(nvs_flash_erase());
	ESP_ERROR_CHECK(nvs_flash_init());
    }

    initialise_wifi();
    xTaskCreate(&ota_server_task, "ota_server_task", 4096, NULL, 5, NULL);

	my_deflog = esp_log_set_vprintf(my_log);
	ESP_LOGW(TAG, "ready");

#ifdef CONFIG_ENABLE_ROS2
    ros2::init(ros2_sock, "z600", 2018);
#endif

    i2c_handler_init();
	
	gps_init();

	camera_init();

#if 0
#ifndef USE_SPI_MODE
		ESP_LOGI(TAG, "Using SDMMC peripheral");
		sdmmc_host_t host = SDMMC_HOST_DEFAULT();

		host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

		// This initializes the slot without card detect (CD) and write protect (WP) signals.
		// Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
		sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

		// To use 1-line SD mode, uncomment the following line:
		slot_config.width = 1;

		// GPIOs 15, 2, 4, 12, 13 should have external 10k pull-ups.
		// Internal pull-ups are not sufficient. However, enabling internal pull-ups
		// does make a difference some boards, so we do that here.
		gpio_set_pull_mode(15, GPIO_PULLUP_ONLY);   // CMD, needed in 4- and 1- line modes
		gpio_set_pull_mode(2, GPIO_PULLUP_ONLY);    // D0, needed in 4- and 1-line modes
		//gpio_set_pull_mode(4, GPIO_PULLUP_ONLY);    // D1, needed in 4-line mode only
		//gpio_set_pull_mode(12, GPIO_PULLUP_ONLY);   // D2, needed in 4-line mode only
		gpio_set_pull_mode(13, GPIO_PULLUP_ONLY);   // D3, needed in 4- and 1-line modes

#else
		ESP_LOGI(TAG, "Using SPI peripheral");

		sdmmc_host_t host = SDSPI_HOST_DEFAULT();
		sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
		slot_config.gpio_miso = PIN_NUM_MISO;
		slot_config.gpio_mosi = PIN_NUM_MOSI;
		slot_config.gpio_sck  = PIN_NUM_CLK;
		slot_config.gpio_cs   = PIN_NUM_CS;
		// This initializes the slot without card detect (CD) and write protect (WP) signals.
		// Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
#endif // USE_SPI_MODE

		// Options for mounting the filesystem.
		// If format_if_mount_failed is set to true, SD card will be partitioned and
		// formatted in case when mounting fails.
		esp_vfs_fat_sdmmc_mount_config_t mount_config = {
			.format_if_mount_failed = true,
			.max_files = 5,
			.allocation_unit_size = 16 * 1024
		};

		// Use settings defined above to initialize SD card and mount FAT filesystem.
		// Note: esp_vfs_fat_sdmmc_mount is an all-in-one convenience function.
		// Please check its source code and implement error recovery when developing
		// production applications.
		esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

		if (ret != ESP_OK) {
			if (ret == ESP_FAIL) {
				ESP_LOGE(TAG, "Failed to mount filesystem. "
				"If you want the card to be formatted, set format_if_mount_failed = true.");
			} else {
				ESP_LOGE(TAG, "Failed to initialize the card (%s). "
				"Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
			}
			sdcard_ready = false;
		}

		xTaskCreate(&sd_test_task, "sd_test_task", 4096, NULL, 5, NULL);
#endif

    // my_deflog = esp_log_set_vprintf(my_i2clog);

#ifdef ENABLE_SLEEP_MODE
#if CONFIG_PM_ENABLE
        // Configure dynamic frequency scaling:
        // maximum and minimum frequencies are set in sdkconfig,
        // automatic light sleep is enabled if tickless idle support is enabled.
        esp_pm_config_esp32_t pm_config = {};
		pm_config.max_freq_mhz = 240;
		pm_config.min_freq_mhz = 40;
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
	    pm_config.light_sleep_enable = true;
#endif
        ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
#endif // CONFIG_PM_ENABLE

#endif

#if 0
		while(1)
		{
			static char tmpstr[8192] = {};
			vTaskGetRunTimeStats(tmpstr);
			ESP_LOGI(TAG, "FreeRTOSStats:\n%d %s\n", strlen(tmpstr),tmpstr);
			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
#endif

		console();

        ESP_LOGI(TAG, "... init done. free heap: %u", xPortGetFreeHeapSize());
}

static esp_err_t event_handler(void* ctx, system_event_t* event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
	esp_wifi_connect();
	break;
    case SYSTEM_EVENT_STA_GOT_IP:
	xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
	s_ip_addr = event->event_info.got_ip.ip_info.ip;
	s_ip_addr_changed = 1;

	esp_wifi_set_ps(WIFI_PS_MAX_MODEM);

	break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
	esp_wifi_connect();
	xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
	s_ip_addr.addr = 0;
	s_ip_addr_changed = 1;
	break;
    default:
	break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    //    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
	.sta =
	    {
	        { .ssid = CONFIG_WIFI_SSID },
	        { .password = CONFIG_WIFI_PASSWORD },
	    },
    };
	wifi_config.sta.listen_interval = 10;
	
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, "ros2mower");
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "Connecting to \"%s\"", wifi_config.sta.ssid);
    xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected");
}