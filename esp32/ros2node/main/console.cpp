/* 
 * ROS2 Mower Network Console
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

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
#include "esp_spiffs.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "cmd_system.h"
//#include "cmd_wifi.h"
#include "cmd_nvs.h"

#include "i2chandler.h"

static const char* TAG = "CON";

#define PORT 23
#define PROMPT "ros2mower>"
static bool promt_active = false;

extern double ubat;

static int con_sock = -1;

static vprintf_like_t con_deflog = NULL;
static char con_log_linebuf[1024];
static bool shutdown_active = false;
static bool con_log_on = true;

/**
 * @brief 
 * @return 
 */
bool console_connected()
{
    return (shutdown_active==false && con_sock!=-1)?true:false;
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
            if( promt_active == true )
            {
                char cr[] = "\n";
                promt_active = false;
                send(con_sock, cr, sizeof(cr), 0);
            }
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
 */
void console_try_remote()
{
    nvs_handle my_handle;
    esp_err_t err = nvs_open("console", NVS_READWRITE, &my_handle);
    if(err == ESP_OK) {
        char addr_str[128];
        int addr_family;
        int ip_protocol;

        char host_name[64] = {};
        size_t host_name_size = sizeof(host_name);
        int host_port = 0;
        nvs_get_str(my_handle, "host", host_name, &host_name_size);
        nvs_get_i32(my_handle, "port", &host_port);
        nvs_close(my_handle);
        if(strlen(host_name) != 0 && host_port != 0) {
            struct sockaddr_in destAddr;
            destAddr.sin_addr.s_addr = inet_addr(host_name);
            destAddr.sin_family = AF_INET;
            destAddr.sin_port = htons(host_port);
            addr_family = AF_INET;
            ip_protocol = IPPROTO_IP;
            inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);

            con_printf("socket ...\n");
            int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
            con_printf("connect ... %d\n",sock);
            int err = connect(sock, (struct sockaddr*)&destAddr, sizeof(destAddr));

            con_printf("%d\n",err);

            if(err != -1) {
                con_sock = sock;
                return;
            }

            shutdown(sock, 0);
            close(sock);
        }
    }
}

void con_i2c_detect()
{
    uint8_t address;
    con_printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
    for (int i = 0; i < 128; i += 16) {
        con_printf("%02x: ", i);
        for (int j = 0; j < 16; j++) {
            address = i + j;
            esp_err_t ret = i2cnode_check(address);
            if (ret == ESP_OK) {
                con_printf("%02x ", address);
            } else if (ret == ESP_ERR_TIMEOUT) {
                con_printf("UU ");
            } else {
                con_printf("-- ");
            }
        }
        con_printf("\r\n");
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
	if( con_log_on == false )
	{
		return 1;
	}
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

#define RX_BUFFER_SIZE 128
/**
 * @brief 
 */
static void con_handle()
{
    int len;
    char rx_buffer[RX_BUFFER_SIZE+1];

    {
        esp_chip_info_t chip_info;
        esp_chip_info(&chip_info);
        con_printf( "This is ESP32 chip with %d CPU cores, WiFi%s%s, ", chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "", (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
        con_printf("silicon revision %d, ", chip_info.revision);
        con_printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    }

    do {
        con_printf("%s", PROMPT);
        promt_active = true;
        len = recv(con_sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
        } else {
            promt_active = false;
            rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            
            while( len>0 && rx_buffer[len-1]<=32 ) rx_buffer[--len] = 0; 
            if(strcasecmp("help",rx_buffer)==0)
            {
                con_printf("* help                    # print available list of commands\n");
#ifdef CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
                con_printf("* top                     # print runtime stats\n");
#endif
                con_printf("* log_set_level TAG LEVEL # set log level for given tag\n");
                con_printf("* close                   # close the connection\n");
                con_printf("* con_remote_set          # set the ip and port for the remote console\n");
                con_printf("* con_remote_get          # get ip and port for the remote console\n");
                con_printf("* ros_remote_set          # set the ip and port for the ros host\n");
                con_printf("* ros_remote_get          # get ip and port for the ros host\n");
                con_printf("* spifs_info              # show some SPIFS information\n");
            }
            else if(strcasecmp("restart",rx_buffer)==0 )
            {
                con_printf("restarting ...\n");
                esp_restart();
                return;
            }
#ifdef CONFIG_ENABLE_I2C_POWER
            else if(strcasecmp("reboot",rx_buffer)==0 )
            {
                i2c_lock();
                i2cnode_set_u16(PWRNODE_I2C_ADDR, 0x16, 5); /* TWI_MEM_PWRUPREL */
                i2cnode_set_u16(PWRNODE_I2C_ADDR, 0x10, 2); /* TWI_MEM_SHDWNCNT */
                //i2c_unlock();
                con_printf("rebooting ...\n");
            }
#endif
            else if(strcasecmp("test",rx_buffer)==0 )
            {
                i2cnode_init_motor();
            }
            else if(strcasecmp("spifs_info",rx_buffer)==0)
            {
                size_t total = 0, used = 0;
                esp_err_t ret = esp_spiffs_info(NULL, &total, &used);
                if (ret != ESP_OK) {
                    con_printf("Failed to get SPIFFS partition information (%s)\n", esp_err_to_name(ret));
                } else {
                    con_printf("Partition size: total: %d, used: %d\n", total, used);
                }
            }
            else if(strcasecmp("i2cdetect",rx_buffer)==0)
            {
                con_i2c_detect();
            }
            else if(strstr(rx_buffer,"i2cget")==rx_buffer)
            {
                /* i2cget i2caddr regaddr numbytes repeats
				 * i2cget 0x0a 0x00 32 10000000
                 */
				 int errcnt = 0;
                int p1 = 0;
                int p2 = 0;
                int p3 = 0;
                int p4 = 0;
                if( ((sscanf(rx_buffer,"i2cget 0x%02x 0x%02x %d %d",&p1,&p2,&p3,&p4) == 4)||
					 (sscanf(rx_buffer,"i2cget 0x%02x 0x%02x %d",&p1,&p2,&p3) == 3)) )
                {
					for( int k=0; k<=p4; k++ )
					{
						con_printf("i2cget 0x%02x 0x%02x [%d]=",p1,p2,p3);
						if(i2cnode_read(p1,p2,(uint8_t*)rx_buffer,p3)==ESP_OK)
						{
							for(int i=0;i<p3;i++)
							{
								con_printf("%02x",rx_buffer[i]);
							}
						}
						else
						{
							con_printf("... ERROR %d",++errcnt);
						}
						con_printf(" errcnt=%d\n",errcnt);
					}
                }
            }
            else if(strstr(rx_buffer,"i2cset")==rx_buffer)
            {
                /* i2cset i2caddr regaddr numbytes xxXXxxXXxxXX
                 */
                int p1 = 0;
                int p2 = 0;
                char p3[RX_BUFFER_SIZE+1] = {};
                int p4 = 0;
                if( ((sscanf(rx_buffer,"i2cset 0x%02x 0x%02x %s",&p1,&p2,p3) == 3) || 
				     (sscanf(rx_buffer,"i2cset 0x%02x 0x%02x %s %d",&p1,&p2,p3,&p4) == 4)) &&
					strlen(p3) <= RX_BUFFER_SIZE )
                {
					if( p4 == 0 )
					{
						p4 = 1;
					}
                    con_printf("i2cset 0x%02x 0x%02x %s %d",p1,p2,p3,p4);
					
                    for(int i=0;i<(strlen(p3)/2);i++)
                    {
                        char tmp[10]="0x00";
                        tmp[0] = '0';
                        tmp[1] = 'x';
                        tmp[2] = p3[2*i+0];
                        tmp[3] = p3[2*i+1];
                        tmp[4] = 0;
                        int p3_ = 0;
                        sscanf(tmp,"0x%02x",&p3_);
                        rx_buffer[i] = p3_;
                        con_printf("%02x",rx_buffer[i]);
                    }
                    i2cnode_write(p1,p2,(uint8_t*)rx_buffer,strlen(p3)/2);
                    con_printf("\n");
                }
            }
            else if(strstr(rx_buffer,"con_remote_set")==rx_buffer)
            {
                char p1[64] = "";
                int p2 = 0;
                int n = sscanf(rx_buffer,"con_remote_set %s %d",p1,&p2);
                if( n==2 && strlen(p1) != 0 && p2 != 0 )
                {
                    nvs_handle my_handle;
                    esp_err_t err = nvs_open("console", NVS_READWRITE, &my_handle);
                    if (err == ESP_OK) 
                    {
                        nvs_set_str(my_handle, "host", p1);
                        nvs_set_i32(my_handle, "port", p2);
                        nvs_close(my_handle);
                    }
                }
            }
            else if(strstr(rx_buffer,"con_remote_get")==rx_buffer)
            {
                nvs_handle my_handle;
                esp_err_t err = nvs_open("console", NVS_READWRITE, &my_handle);
                if (err == ESP_OK) 
                {
                    char host_name[64] = {};
                    size_t host_name_size = sizeof(host_name);
                    int host_port = 0;
                    nvs_get_str(my_handle, "host", host_name, &host_name_size);
                    nvs_get_i32(my_handle, "port", &host_port);
                    nvs_close(my_handle);
                    con_printf("remote console host=%s port=%d\n",host_name,host_port);
                }
            }
            else if(strstr(rx_buffer,"ros_remote_set")==rx_buffer)
            {
                char p1[64] = "";
                int p2 = 0;
                int n = sscanf(rx_buffer,"ros_remote_set %s %d",p1,&p2);
                con_printf("wr %d %s %d\n",n,p1,p2);
                if( n==2 && strlen(p1) != 0 && p2 != 0 )
                {
                    nvs_handle my_handle;
                    esp_err_t err = nvs_open("ros", NVS_READWRITE, &my_handle);
                    if (err == ESP_OK) 
                    {
                        nvs_set_str(my_handle, "host", p1);
                        nvs_set_i32(my_handle, "port", p2);
                        nvs_close(my_handle);
                    }
                }
            }
            else if(strstr(rx_buffer,"ros_remote_get")==rx_buffer)
            {
                nvs_handle my_handle;
                esp_err_t err = nvs_open("ros", NVS_READWRITE, &my_handle);
                if (err == ESP_OK) 
                {
                    char host_name[64] = {};
                    size_t host_name_size = sizeof(host_name);
                    int host_port = 0;
                    nvs_get_str(my_handle, "host", host_name, &host_name_size);
                    nvs_get_i32(my_handle, "port", &host_port);
                    nvs_close(my_handle);
                    con_printf("ros host=%s port=%d\n",host_name,host_port);
                }
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
            else if(strcasecmp("logoff",rx_buffer)==0)
            {
				con_log_on = false;
                con_printf("logoff ...\n");
            }
            else if(strcasecmp("logon",rx_buffer)==0)
            {
				con_log_on = true;
                con_printf("logon ...\n");
            }
            else if(strcasecmp("logerror",rx_buffer)==0)
            {
                con_printf("logerror ...\n");
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
            else if(strcasecmp("shutdown",rx_buffer)==0)
            {
                shutdown_active = true;
                con_printf("shutdown ...\n");
                //try { i2cnode_set_u16(PWRNODE_I2C_ADDR, 0x10,  10); } catch(int err) { }
                //shutdown(con_sock, 0);
                //close(con_sock);
                //con_sock = -1;
                //return;
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
            else if(strcasecmp("top",rx_buffer)==0)
            {
                vTaskGetRunTimeStats(con_log_linebuf);
                con_printf("%s",con_log_linebuf);
            }
#endif
#ifdef CONFIG_ENABLE_I2C_MOTOR
#define CMD_VEL_SPEED       0.1
#define CMD_VEL_ROTSLOW     (2.0*M_PI / 30.0)  
#define CMD_VEL_ROTFAST     (2.0*M_PI / 10.0)  
            else if(strcasecmp("8",rx_buffer)==0)
            {
                i2c_set_cmd_vel(CMD_VEL_SPEED,0.0,0.0);
            }
            else if(strcasecmp("7",rx_buffer)==0)
            {
                i2c_set_cmd_vel(CMD_VEL_SPEED,0.0,CMD_VEL_ROTSLOW);
            }
            else if(strcasecmp("9",rx_buffer)==0)
            {
                i2c_set_cmd_vel(CMD_VEL_SPEED,0.0,-CMD_VEL_ROTSLOW);
            }
            else if(strcasecmp("2",rx_buffer)==0)
            {
                i2c_set_cmd_vel(-CMD_VEL_SPEED,0.0,0.0);
            }
            else if(strcasecmp("5",rx_buffer)==0)
            {
                i2c_set_cmd_vel(0.0,0.0,0.0);
            }
            else if(strcasecmp("4",rx_buffer)==0)
            {
                i2c_set_cmd_vel(0.0,0.0,CMD_VEL_ROTFAST);
            }
            else if(strcasecmp("6",rx_buffer)==0)
            {
                i2c_set_cmd_vel(0.0,0.0,-CMD_VEL_ROTFAST);
            }
            else if(strcasecmp("1",rx_buffer)==0)
            {
                i2c_set_cmd_vel(-CMD_VEL_SPEED,0.0,CMD_VEL_ROTSLOW);
            }
            else if(strcasecmp("3",rx_buffer)==0)
            {
                i2c_set_cmd_vel(-CMD_VEL_SPEED,0.0,-CMD_VEL_ROTSLOW);
            }
#endif            
            else
            {
                for(int i=0;i<len;i++) con_printf("%02x",rx_buffer[i]); 
                con_printf("|%s\n",rx_buffer);
            }
        }
    } while (con_sock!=-1);
}

/**
 * @brief 
 */
void console()
{
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

	con_deflog = esp_log_set_vprintf(con_log);

    console_try_remote();
    
    if (con_sock != -1) {
        con_handle();
    }
    
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

/*
 * EOF
 */

