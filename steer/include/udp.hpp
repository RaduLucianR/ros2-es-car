#ifndef UDP_SERVER_H
#define UDP_SERVER_H

#include <stdint.h>
#include "rt_task.hpp"
#include "logConf.hpp"

/**
 * Allows to disable the reception of own multicast messages
 */
#define UDP_DISABLE_LOOPBACK

/**
 * Multicast group and port used for communication
 */
#define MULTICAST_GROUP  "239.255.42.99"
#define UDP_PORT 32768

/**
 * Logging of status messages
 */
#ifdef UDP_INFO_MSG
  #define UDP_PRINT_INFO( fmt, ...) fprintf( stdout, "[UDP] " fmt "\r\n", ##__VA_ARGS__)
#else
  #define UDP_PRINT_INFO(fmt, ...)
#endif

/**
 * Logging of error messages
 */
#ifdef UDP_PRINT_ERROR_MSG
  #define UDP_PRINT_ERROR( fmt, ...) fprintf( stdout, "[UDP ERROR]" fmt " [%s:%d]\r\n", ##__VA_ARGS__, __FUNCTION__, __LINE__)
#else
  #define UDP_PRINT_ERROR( fmt, ...)
#endif

/**
 * Message content for a com message that is sent via UDP
 */
typedef struct udp_msg {
    uint8_t  id;
    double    val;
}udp_msg_t;

/**
 * Functions to manage the task. Handles initialization and receives messages.
 */
void* udp_init(void * arg);
void* udp_deinit(void * arg);
void* udp_job(void * arg);

/**
 * Sends a message
 */
void* udp_send(udp_msg_t msg);

#endif //UDP_SERVER_H
