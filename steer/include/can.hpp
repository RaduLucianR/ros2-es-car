#ifndef CAN_SERVER_H
#define CAN_SERVER_H

#include <stdint.h>
#include "com.hpp"
#include "rt_task.hpp"
#include "logConf.hpp"

/**
 * Logging of status messages
 */
#ifdef CAN_INFO_MSG
  #define CAN_PRINT_INFO( fmt, ...) fprintf( stdout, "[CAN] " fmt "\r\n", ##__VA_ARGS__)
#else
  #define CAN_PRINT_INFO(fmt, ...)
#endif

/**
 * Logging of error messages
 */
#ifdef CAN_PRINT_ERROR_MSG
  #define CAN_PRINT_ERROR( fmt, ...) fprintf( stdout, "[CAN ERROR]" fmt " [%s:%d]\r\n", ##__VA_ARGS__, __FUNCTION__, __LINE__)
#else
  #define CAN_PRINT_ERROR( fmt, ...)
#endif

/**
 * The CAN-id of a signal is calculated by adding the offset described here
 * and the com-id of the signal.
 */
#define CAN_ID_OFFSET 0x100

/**
 * Message content for a com message that is sent via CAN
 */
typedef struct can_msg {
    uint8_t   com_id;
    uint8_t   length;
    uint8_t   payload[8]; //max length of CAN frame
}can_msg_t;

/**
 * Functions to manage the task. Handles initialization and receives messages.
 */
void* can_init(void * arg);
void* can_deinit(void * arg);
void* can_job(void * arg);

/**
 * Sends a message
 */
void* can_send(can_msg_t msg);

#endif //CAN_SERVER_H
