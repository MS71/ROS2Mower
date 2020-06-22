#include "udp_transport_internal.h"

//#include <uxr/client/profile/transport/ip/udp/udp_transport_posix.h>

#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <netdb.h>
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

static const char* TAG = "UDPIO";

typedef struct uxrUDPPlatform
{
    int poll_fd;

} uxrUDPPlatform;

bool uxr_init_udp_platform(
        uxrUDPPlatform* platform,
        uxrIpProtocol ip_protocol,
        const char* ip,
        const char* port)
{
    bool rv = false;

//    ESP_LOGI(TAG, "uxr_init_udp_platform %s %s",ip,port);


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
//                    platform->poll_fd.events = POLLIN;
                    rv = true;
                    break;
                }
            }
        }
        freeaddrinfo(result);
    }

    return rv;
}

bool uxr_close_udp_platform(
        uxrUDPPlatform* platform)
{
//    ESP_LOGI(TAG, "uxr_close_udp_platform");

    return (-1 == platform->poll_fd) ? true : (0 == close(platform->poll_fd));
    return true;
}

size_t uxr_write_udp_data_platform(
        uxrUDPPlatform* platform,
        const uint8_t* buf,
        size_t len,
        uint8_t* errcode)
{
    size_t rv = 0;

//    ESP_LOGI(TAG, "uxr_write_udp_data_platform");

    ssize_t bytes_sent = send(platform->poll_fd, (void*)buf, len, 0);
    if (-1 != bytes_sent)
    {
        rv = (size_t)bytes_sent;
        *errcode = 0;
    }
    else
    {
        *errcode = 1;
    }
    return rv;
}

size_t uxr_read_udp_data_platform(
        uxrUDPPlatform* platform,
        uint8_t* buf,
        size_t len,
        int timeout,
        uint8_t* errcode)
{
    size_t rv = 0;

//    ESP_LOGI(TAG, "uxr_read_udp_data_platform");

#ifdef POLLIN
    int poll_rv = poll(&platform->poll_fd, 1, timeout);
    if (0 < poll_rv)
    {
#else
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = timeout*1000;
    setsockopt(platform->poll_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
#endif

        ssize_t bytes_received = recv(platform->poll_fd, (void*)buf, len, 0);
        if (-1 != bytes_received)
        {
            rv = (size_t)bytes_received;
            *errcode = 0;
        }
        else
        {
            *errcode = 1;
        }
#ifdef POLLIN
    }
    else
    {
      *errcode = (0 == poll_rv) ? 0 : 1;
    }
#endif
    return rv;
}

