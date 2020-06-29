/* Console example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
//#include "cmd_decl.h"
#include "esp_vfs_fat.h"
#include "esp_sleep.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "cmd_system.h"
//#include "cmd_wifi.h"
#include "cmd_nvs.h"


static const char* TAG = "CON";

#define PORT 23
#define PROMPT "ros2mower>"

extern double ubat;

static int con_sock = -1;

static vprintf_like_t con_deflog = NULL;
static char con_log_linebuf[1024];

/**
 * @brief 
 * @return 
 */
bool console_connected()
{
    return (con_sock!=-1)?true:false;
}

/**
 * @brief 
 * @param fmt
 */
void con_printf(const char* fmt, ...)
{ 
    if( con_sock != -1 )
    {
        //char linebuf[180];
        va_list arglist;
        va_start( arglist, fmt );
        int n = vsnprintf( con_log_linebuf, sizeof(con_log_linebuf), fmt, arglist );
        if(n > 0)
        {
            if( send(con_sock, con_log_linebuf, n, 0) != n )
            {
                shutdown(con_sock, 0);
                close(con_sock);
                con_sock = -1;            
            }
        }
        va_end( arglist );   
    }
}

/**
 * @brief 
 * @param format
 * @param args
 * @return 
 */
int con_log(const char *format, va_list args)
{
    if( con_sock != -1 )
    {
        vsnprintf (con_log_linebuf, sizeof(con_log_linebuf)-1, format, args);
        int n = strlen(con_log_linebuf);	
        if(n > 0)
        {
            if( send(con_sock, con_log_linebuf, n, 0) != n )
            {
                shutdown(con_sock, 0);
                close(con_sock);
                con_sock = -1;            
            }
        }
    }
    else if( con_deflog != NULL )
    {
        con_deflog(format,args);
    }
	return 1;
}

/**
 * @brief 
 */
static void con_handle()
{
    int len;
    char rx_buffer[128];

    do {
        con_printf("%s", PROMPT);
        len = recv(con_sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
        } else {
            rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            
            while( len>0 && rx_buffer[len-1]<=32 ) rx_buffer[--len] = 0; 
            if(strcasecmp("help",rx_buffer)==0)
            {
                con_printf("* help                    # print available list of commands\n");
#ifdef CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
                con_printf("* rtstats                 # print runtime stats\n");
#endif
                con_printf("* log_set_level TAG LEVEL # set log level for given tag\n");
                con_printf("* close                   # close the connection\n");
            }
            else if(strcasecmp("restart",rx_buffer)==0 || strcasecmp("reboot",rx_buffer)==0)
            {
                con_printf("restarting ...\n");
                esp_restart();
                return;
            }
            else if(strstr(rx_buffer,"log_set_level")==rx_buffer)
            {
                char p1[64] = "";
                char p2[64] = "";
                char p3[64] = "";
                int n = sscanf(rx_buffer,"%s %s %s",p1,p2,p3);
                if( n==3 && strcmp(p1,"log_set_level")==0 && strcmp(p3,"none")==0)
                {
                    esp_log_level_set(p2,ESP_LOG_NONE);
                }
                else if( n==3 && strcmp(p1,"log_set_level")==0 && strcmp(p3,"error")==0)
                {
                    esp_log_level_set(p2,ESP_LOG_ERROR);
                }
                else if( n==3 && strcmp(p1,"log_set_level")==0 && strcmp(p3,"warn")==0)
                {
                    esp_log_level_set(p2,ESP_LOG_WARN);
                }
                else if( n==3 && strcmp(p1,"log_set_level")==0 && strcmp(p3,"info")==0)
                {
                    esp_log_level_set(p2,ESP_LOG_INFO);
                }
                else if( n==3 && strcmp(p1,"log_set_level")==0 && strcmp(p3,"debug")==0)
                {
                    esp_log_level_set(p2,ESP_LOG_DEBUG);
                }
                else if( n==3 && strcmp(p1,"log_set_level")==0 && strcmp(p3,"verbose")==0)
                {
                    esp_log_level_set(p2,ESP_LOG_VERBOSE);
                }                
            }
            else if(strcasecmp("logerror",rx_buffer)==0)
            {
                con_printf("logoff ...\n");
                esp_log_level_set("*",ESP_LOG_ERROR);
            }
            else if(strcasecmp("logwarn",rx_buffer)==0)
            {
                con_printf("logwarn ...\n");
                esp_log_level_set("*",ESP_LOG_WARN);
            }
            else if(strcasecmp("loginfo",rx_buffer)==0)
            {
                con_printf("loginfo ...\n");
                esp_log_level_set("*",ESP_LOG_INFO);
            }
            else if(strcasecmp("info",rx_buffer)==0)
            {
                esp_chip_info_t chip_info;
                esp_chip_info(&chip_info);
                con_printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, \n", chip_info.cores,
                    (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "", (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

                con_printf("silicon revision %d, \n", chip_info.revision);

                con_printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
                    (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
 
               con_printf("ubat = %1.3f\n", ubat);
            }
            else if(strcasecmp("close",rx_buffer)==0)
            {
                con_printf("closing ...\n");
                shutdown(con_sock, 0);
                close(con_sock);
                con_sock = -1;
                return;
            }
#ifdef CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
            else if(strcasecmp("rtstats",rx_buffer)==0)
            {
                vTaskGetRunTimeStats(con_log_linebuf);
                con_printf("%s",con_log_linebuf);
            }
#endif
            else
            {
                con_printf("?%s\n",rx_buffer);
            }
        }
    } while (con_sock!=-1);
}

void console()
{
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

	con_deflog = esp_log_set_vprintf(con_log);

    if (addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;
    } else if (addr_family == AF_INET6) {
        bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        ip_protocol = IPPROTO_IPV6;
    }

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
    // Note that by default IPV6 binds to both protocols, it is must be disabled
    // if both protocols used at the same time (used in CI)
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    setsockopt(listen_sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
#endif

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) {

        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
        uint addr_len = sizeof(source_addr);
        con_sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (con_sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Convert ip address to string
        if (source_addr.sin6_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
        } else if (source_addr.sin6_family == PF_INET6) {
            inet6_ntoa_r(source_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
        }
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);
        ESP_LOGI(TAG, "Socket closed");

        con_handle();
        con_sock = -1;
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}



