// Copyright 2015-2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// under the License is distributed on an "AS IS" BASIS,
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
#include "ros2esp.h"
#endif

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_event_loop.h"
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
#include "rom/uart.h"
#include "sdmmc_cmd.h"

extern "C" {
//#include "ulp-util.h" // my ulp_init(), ulp_start()
}

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "ssd1306.h"
#include "ssd1306_default_if.h"
#include "ssd1306_draw.h"
#include "ssd1306_font.h"

#include "cam.h"
#include "gps.h"

#include "console.h"

#define ENABLE_SLEEP_MODE

#define SPIHOST_PIN_NUM_MISO (gpio_num_t)23
#define SPIHOST_PIN_NUM_MOSI (gpio_num_t)19
#define SPIHOST_PIN_NUM_CLK  (gpio_num_t)18
#define SPIHOST_PIN_NUM_CS   (gpio_num_t)5

static void handle_grayscale_pgm(http_context_t http_ctx, void* ctx);
static void handle_rgb_bmp(http_context_t http_ctx, void* ctx);
static void handle_rgb_bmp_stream(http_context_t http_ctx, void* ctx);
static void handle_jpg(http_context_t http_ctx, void* ctx);
static void handle_jpg_stream(http_context_t http_ctx, void* ctx);
static esp_err_t event_handler(void* ctx, system_event_t* event);
static void initialise_wifi(void);

static char my_wifi_ssid[64] = {};
static char my_wifi_psk[64] = {};
static bool my_wifi_save_on_connected = false;

sdmmc_card_t* card = NULL;
bool sdcard_ready = true;

bool camera_ready = true;

static const char* TAG = "MAIN";

EventGroupHandle_t s_wifi_event_group;
const int CONNECTED_BIT = BIT0;
// esp_ip4_addr_t s_ip_addr = {};
ip4_addr_t s_ip_addr = {};
uint8_t s_ip_addr_changed = 1;

int ros2_sock = -1;

/**
 * @brief 
 * @param param
 */
static void ota_server_task(void* param)
{
    xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    ota_server_start();
    vTaskDelete(NULL);
}

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
#define SDSPI_PIN_NUM_MISO (gpio_num_t)2
#define SDSPI_PIN_NUM_MOSI (gpio_num_t)15
#define SDSPI_PIN_NUM_CLK (gpio_num_t)14
#define SDSPI_PIN_NUM_CS (gpio_num_t)13
#endif // USE_SPI_MODE


/**
 * @brief 
 * @param param
 */
static void sd_test_task(void* param)
{
    if(sdcard_ready == true && card != NULL) 
	{
		ESP_LOGW(TAG, "sd_test_task()");

		uint8_t fn_idx = 0;
		char fn[64];
		// Card has been initialized, print its properties
		sdmmc_card_print_info(stdout, card);


		while(1) {
			size_t sd_test_buf_size = 2 * 1024 * 1024;
			uint8_t* sd_test_buf = (uint8_t*)malloc(sd_test_buf_size);
			if(sd_test_buf != NULL) {
				memset(sd_test_buf,0,sd_test_buf_size);
				// Use POSIX and C standard library functions to work with files.
				// First create a file.
				sprintf(fn,"/sdcard/hello-%d.bin",fn_idx++);
				FILE* f = fopen(fn, "w");
				if(f == NULL) {
					ESP_LOGE(TAG, "Failed to open file for writing");
				} else {
					uint32_t i = 0;
					int64_t t = esp_timer_get_time();
					i += fwrite(sd_test_buf, 1, sd_test_buf_size, f);
					if((esp_timer_get_time() - t) != 0) {
					ESP_LOGW(TAG, "File (%s) written %dMByte %dkB/s", 
						fn,
						(int)(sd_test_buf_size / (1024 * 1024)),
						(int)((1000 * i) / (esp_timer_get_time() - t)));
					}
					fclose(f);
				}

				{
					f = fopen(fn, "r");
					if(f == NULL) {
					ESP_LOGE(TAG, "Failed to open file for reading");
					} else {
					int64_t t = esp_timer_get_time();
					size_t i = 0;
					while(!feof(f)) {
						i += fread(sd_test_buf, 1, sd_test_buf_size, f);
					}
					if((esp_timer_get_time() - t) != 0) {
						ESP_LOGW(TAG, "File (%s) read %dkB/s", 
							fn,
							(int)((1000 * i) / (esp_timer_get_time() - t)));
					}
					fclose(f);
					}
				}
				free(sd_test_buf);
			}

			vTaskDelay(5000 / portTICK_PERIOD_MS);
		}
    }
    vTaskDelete(NULL);
}

/**
 * @brief 
 */
void sdmmc_init()
{
    esp_err_t ret;
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
    gpio_set_pull_mode((gpio_num_t)15, GPIO_PULLUP_ONLY); // CMD, needed in 4- and 1- line modes
    gpio_set_pull_mode((gpio_num_t)2, GPIO_PULLUP_ONLY);  // D0, needed in 4- and 1-line modes
    // gpio_set_pull_mode(4, GPIO_PULLUP_ONLY);    // D1, needed in 4-line mode only
    // gpio_set_pull_mode(12, GPIO_PULLUP_ONLY);   // D2, needed in 4-line mode only
    gpio_set_pull_mode((gpio_num_t)13, GPIO_PULLUP_ONLY); // D3, needed in 4- and 1-line modes
    gpio_set_pull_mode((gpio_num_t)14, GPIO_PULLUP_ONLY); // CLK, needed in 4- and 1-line modes

#else
    ESP_LOGI(TAG, "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
    slot_config.gpio_miso = SDSPI_PIN_NUM_MISO;
    slot_config.gpio_mosi = SDSPI_PIN_NUM_MOSI;
    slot_config.gpio_sck = SDSPI_PIN_NUM_CLK;
    slot_config.gpio_cs = SDSPI_PIN_NUM_CS;
    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
#endif // USE_SPI_MODE

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
	.format_if_mount_failed = true, .max_files = 16, .allocation_unit_size = 16 * 1024
    };

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc_mount is an all-in-one convenience function.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    if(ret != ESP_OK) {
	if(ret == ESP_FAIL) {
	    ESP_LOGE(TAG,
	        "Failed to mount filesystem. "
	        "If you want the card to be formatted, set format_if_mount_failed = true.");
	} else {
	    ESP_LOGE(TAG,
	        "Failed to initialize the card (%s). "
	        "Make sure SD card lines have pull-up resistors in place.",
	        esp_err_to_name(ret));
	}
	sdcard_ready = false;
    }
}

spi_device_handle_t spi;

spi_bus_config_t buscfg = {};
spi_device_interface_config_t devcfg = {};

void spihost_init()
{
    esp_err_t ret;
	
    gpio_set_direction(SPIHOST_PIN_NUM_CS, GPIO_MODE_OUTPUT);


	buscfg.miso_io_num=SPIHOST_PIN_NUM_MISO;
    buscfg.mosi_io_num=SPIHOST_PIN_NUM_MOSI;
	buscfg.sclk_io_num=SPIHOST_PIN_NUM_CLK;
	buscfg.quadwp_io_num=-1;
	buscfg.quadhd_io_num=-1;
	buscfg.max_transfer_sz=1024*8;
	
	devcfg.clock_speed_hz=4*1000*1000;           //Clock out at 26 MHz
	devcfg.mode=0;                         		  //SPI mode 0
    devcfg.spics_io_num=SPIHOST_PIN_NUM_CS;       //CS pin
    devcfg.queue_size=1;                          //We want to be able to queue 7 transactions at a time
    //.pre_cb=lcd_spi_pre_transfer_callback,      //Specify pre-transfer callback to handle D/C line
	devcfg.command_bits = 8;
	devcfg.address_bits = 8;
	//devcfg.dummy_bits = 32;
	devcfg.cs_ena_pretrans = 1;
	devcfg.cs_ena_posttrans = 16;
    //devcfg.flags = SPI_DEVICE_HALFDUPLEX;
		
    //Initialize the SPI bus
    ret=spi_bus_initialize(VSPI_HOST, &buscfg, 1 /*dma*/ );
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

	
}

/**
 * @brief 
 * @param param
 */
static void spihost_test_task(void* param)
{
	ESP_LOGW(TAG, "spihost_test_task()");
	while(1) 
	{
		size_t test_buf_size = 1024;
		uint8_t* test_buf = (uint8_t*)malloc(test_buf_size);
		if(test_buf != NULL) 
		{		
			//ESP_LOGW(TAG, "spihost_test_task() loop ...");
			for( int i=0;i<test_buf_size;i++)
			{
				test_buf[i] = i&0xff;
			}
			
		    esp_err_t ret;
			spi_transaction_t t = {};
			t.cmd = 0xAA;
			t.addr = 0x55;
			t.length=test_buf_size*8;       //Len is in bytes, transaction length is in bits.
			t.rxlength=test_buf_size*8;     //Len is in bytes, transaction length is in bits.
			t.tx_buffer=test_buf;           //Data
			t.rx_buffer=test_buf;           //Data
			//t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
			ret=spi_device_polling_transmit(spi, &t);  //Transmit!
			assert(ret==ESP_OK);            //Should have had no issues.

			for( int i=0;i<test_buf_size;i++)
			{
				if( test_buf[i] != (i&0xff) )
				{
					ESP_LOGW(TAG, "spihost_test_task() payload error test_buf[%04x] != %02x",i,i&0xff);
					break;
				}
			}

			free(test_buf);
		}		
	}
    vTaskDelete(NULL);
}

void check_wifi_config()
{
    /* Install UART driver for interrupt-driven reads and writes */
    uart_driver_install((uart_port_t)CONFIG_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0);
    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(CONFIG_CONSOLE_UART_NUM);

    /* Disable buffering on stdin and stdout */
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_uart_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_uart_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    while(1) {
	nvs_handle my_handle;
	esp_err_t err = nvs_open("wifi", NVS_READWRITE, &my_handle);
	if(err == ESP_OK) {
	    size_t my_wifi_ssid_size = sizeof(my_wifi_ssid);
	    size_t my_wifi_psk_size = sizeof(my_wifi_psk);
	    nvs_get_str(my_handle, "ssid", my_wifi_ssid, &my_wifi_ssid_size);
	    nvs_get_str(my_handle, "psk", my_wifi_psk, &my_wifi_psk_size);
	    nvs_close(my_handle);
	    if(strlen(my_wifi_ssid) != 0 && strlen(my_wifi_psk) != 0) {
		return;
	    }
	    my_wifi_save_on_connected = true;
	    printf("\nenter Wifi SSID: ");
	    gets(my_wifi_ssid);
	    printf("\nenter Wifi PSK: ");
	    gets(my_wifi_psk);
	}
    }
}

extern "C" {
void app_main(void);
}

void app_main(void)
{
    /* Print chip information */
    // esp_log_level_set("i2c", ESP_LOG_INFO);
    // esp_log_level_set("gpio", ESP_LOG_WARN);

    ESP_LOGI(TAG, "init ULP ...");
    // ulp_init();
    // ulp_start();

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

    check_wifi_config();

    esp_vfs_spiffs_conf_t conf = {
	.base_path = "/spiffs", .partition_label = NULL, .max_files = 5, .format_if_mount_failed = true
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if(ret != ESP_OK) {
	if(ret == ESP_FAIL) {
	    ESP_LOGE(TAG, "Failed to mount or format filesystem");
	} else if(ret == ESP_ERR_NOT_FOUND) {
	    ESP_LOGE(TAG, "Failed to find SPIFFS partition");
	} else {
	    ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
	}
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if(ret != ESP_OK) {
	ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
	ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    initialise_wifi();
    xTaskCreate(&ota_server_task, "ota_server_task", 4096, NULL, 5, NULL);

    // my_deflog = esp_log_set_vprintf(my_log);
    ESP_LOGW(TAG, "ready");

#ifdef CONFIG_ENABLE_ROS2
    {
	nvs_handle my_handle;
	esp_err_t err = nvs_open("ros", NVS_READWRITE, &my_handle);
	if(err == ESP_OK) {
	    char host_name[64] = {};
	    size_t host_name_size = sizeof(host_name);
	    int host_port = 0;
	    nvs_get_str(my_handle, "host", host_name, &host_name_size);
	    nvs_get_i32(my_handle, "port", &host_port);
	    nvs_close(my_handle);
	    if(strlen(host_name) != 0 && host_port != 0) {
		ros2::init(ros2_sock, host_name, host_port);
	    } else {
		// ros2::init(ros2_sock, "", 0);
	    }
	} else {
	    // ros2::init(ros2_sock, "", 0);
	}
    }
#endif

#if 0
    gpio_num_t gpio = (gpio_num_t)18;
    gpio_reset_pin(gpio);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
    while(1) 
	{
		gpio_set_level(gpio, 0);
		vTaskDelay(10 / portTICK_PERIOD_MS);
		gpio_set_level(gpio, 1);
		vTaskDelay(10 / portTICK_PERIOD_MS);
    }
#endif

    i2c_handler_init();

    sdmmc_init();
    xTaskCreate(&sd_test_task, "sd_test_task", 4096, NULL, 5, NULL);
    //while(1) vTaskDelay(1000 / portTICK_PERIOD_MS);

    spihost_init();
    xTaskCreate(&spihost_test_task, "spihost_test_task", 4096, NULL, 5, NULL);

    gps_init();

    camera_init();

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

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    console();

    ESP_LOGI(TAG, "... init done. free heap: %u", xPortGetFreeHeapSize());
}

static esp_err_t event_handler(void* ctx, system_event_t* event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
	esp_wifi_connect();

    case SYSTEM_EVENT_STA_GOT_IP:
	xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
	s_ip_addr = event->event_info.got_ip.ip_info.ip;
	s_ip_addr_changed = 1;

	esp_wifi_set_ps(WIFI_PS_MAX_MODEM);

	if(my_wifi_save_on_connected) {
	    my_wifi_save_on_connected = false;
	    nvs_handle my_handle;
	    esp_err_t err = nvs_open("wifi", NVS_READWRITE, &my_handle);
	    if(err == ESP_OK) {
		size_t my_wifi_ssid_size = sizeof(my_wifi_ssid);
		size_t my_wifi_psk_size = sizeof(my_wifi_psk);
		nvs_set_str(my_handle, "ssid", my_wifi_ssid);
		nvs_set_str(my_handle, "psk", my_wifi_psk);
		nvs_close(my_handle);
		printf("SSID and PSK stored to nvs\n");
	    }
	}

	ESP_LOGI(TAG, "Wifi Connected");
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
    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, my_wifi_ssid);
    strcpy((char*)wifi_config.sta.password, my_wifi_psk);
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
