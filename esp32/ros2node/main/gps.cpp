#include "sdkconfig.h"
#ifdef CONFIG_ENABLE_GPS
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

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

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_event_loop.h"
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "hwconfig.h"
#include "gps.h"

static const char* TAG = "GPS";

#define PORT 20000

static void tcp_server_task(void* pvParameters)
{
    uint8_t rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = { .baud_rate = 115200,
	.data_bits = UART_DATA_8_BITS,
	.parity = UART_PARITY_DISABLE,
	.stop_bits = UART_STOP_BITS_1,
	.flow_ctrl = UART_HW_FLOWCTRL_DISABLE };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, GPS_UART1_TXD, GPS_UART1_RXD, -1, -1);
    uart_driver_install(UART_NUM_1, sizeof(rx_buffer) * 2, 0, 0, NULL, 0);

    while(1) {

	struct sockaddr_in6 destAddr;
	bzero(&destAddr.sin6_addr.un, sizeof(destAddr.sin6_addr.un));
	destAddr.sin6_family = AF_INET6;
	destAddr.sin6_port = htons(PORT);
	addr_family = AF_INET6;
	ip_protocol = IPPROTO_IPV6;
	inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);

	int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
	if(listen_sock < 0) {
	    ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
	    break;
	}
	ESP_LOGI(TAG, "Socket created");

	int err = bind(listen_sock, (struct sockaddr*)&destAddr, sizeof(destAddr));
	if(err != 0) {
	    ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
	    break;
	}
	ESP_LOGI(TAG, "Socket binded");

	while(1) {

	    err = listen(listen_sock, 1);
	    if(err != 0) {
		ESP_LOGE(TAG, "Error occured during listen: errno %d", errno);
		break;
	    }
	    ESP_LOGI(TAG, "Socket listening");

	    struct sockaddr_in6 sourceAddr; // Large enough for both IPv4 or IPv6
	    uint addrLen = sizeof(sourceAddr);
	    int sock = accept(listen_sock, (struct sockaddr*)&sourceAddr, &addrLen);
	    if(sock < 0) {
		ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
		break;
	    }
	    ESP_LOGI(TAG, "Socket accepted");

#if 0
        char initstr[] = 
            "!UBX CFG-GNSS 0 32 32 1 0 10 32 0 1\n"
            "!UBX CFG-GNSS 0 32 32 1 6 8 16 0 1\n"
            "!UBX CFG-MSG 3 15 0 1 0 1 0 0\n"
            "!UBX CFG-MSG 3 16 0 1 0 1 0 0\n"
            "!UBX CFG-MSG 1 32 0 1 0 1 0 0\n";
        uart_write_bytes(UART_NUM_1, initstr, sizeof(initstr));

	    ESP_LOGI(TAG, "Socket accepted");
#endif
	    while(1) {

            int len;
            len = recv(sock, rx_buffer, sizeof(rx_buffer), MSG_DONTWAIT);

            if( len == EWOULDBLOCK )
            {
                vTaskDelay(1 / portTICK_PERIOD_MS);
            }

            if(len > 0) {
                rx_buffer[len] = 0;
                ESP_LOGW(TAG, "TX[%d]:%s", len,rx_buffer);
                uart_write_bytes(UART_NUM_1, (const char*)rx_buffer, len);
            } else if(len == 0) {
                ESP_LOGW(TAG, "Connection closed");
                break;
            }

            // Read data from the UART
            len = uart_read_bytes(UART_NUM_1, rx_buffer, sizeof(rx_buffer), 1 / portTICK_RATE_MS);
            len = 0;
            // printf("RX:%d\n",len);
            // Write data back to the UART
            if(len > 0) {
                rx_buffer[len] = 0;
                //ESP_LOGW(TAG, "RX[%d]:%s", len,rx_buffer);
                int err = send(sock, rx_buffer, len, 0);
                if(err < 0) {
                ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
                break;
                }
            }
	    }

	    if(sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
	    }
	}
    }
    vTaskDelete(NULL);
}

void gps_init()
{
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
}

void gps_exit()
{
}

#endif
