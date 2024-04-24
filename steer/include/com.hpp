/**
 * @file com.h
 * @author Matthias Becker
 * @brief Communication handler
 * @date 2022-12-21
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef COM_H
#define COM_H

#include <stdint.h>
#include "rt_task.hpp"
#include "udp.hpp"
#include "can.hpp"
#include "logConf.hpp"

/**
 * Flag to indicate of a signal is used locally only or sent via UDP or CAN
 */
#define COM_LOCAL  0
#define COM_UDP    1
#define COM_CAN    2

/**
 * Logging of status messages
 */
#ifdef COM_INFO_MSG
  #define COM_PRINT_INFO( fmt, ...) fprintf( stdout, "[COM] " fmt "\r\n", ##__VA_ARGS__)
#else
  #define COM_PRINT_INFO(fmt, ...)
#endif

/**
 * Logging of error messages
 */
#ifdef COM_PRINT_ERROR_MSG
  #define COM_PRINT_ERROR( fmt, ...) fprintf( stdout, "[COM ERROR]" fmt " [%s:%d]\r\n", ##__VA_ARGS__, __FUNCTION__, __LINE__)
#else
  #define COM_PRINT_ERROR( fmt, ...)
#endif

/**
 * type to identify a signal.
 */
typedef uint8_t com_id_t;

/**
 * Type to represent one signal value sample
 */
typedef struct{
  long long ts;   // timestamp of this data value in ms
  double val;  // value of the sensor (unit depends on sensor)
} signal_val_t;

/**
 * Type for a function pointer used for callbacks
 */
typedef void* (*com_user_callback)(signal_val_t);

/**
 * Struct used to keep track of update callbacks that are gegistered for a
 * signal
 */
struct update_cbk {
  com_user_callback update_cb;
  struct update_cbk* next;
};

/**
 * Description of a signal
 */
struct com_desc_t {
  com_id_t            id;         /* signal id */
  char*               name;       /* name of the signal */
  double              min;        /* min value of this signal */
  double              max;        /* max value of this signal */
  uint8_t             transport;  /* 0 if local only, 1 for UDP, 2 for CAN */
  signal_val_t        value;      /* shared memory id */
  struct update_cbk*  update_cb;  /* list of registered update callbacks */
  struct com_desc_t*  next;       /* pointer to the next signal */
};


uint8_t com_read(com_id_t id, double* val, int64_t* ts);
uint8_t com_write(com_id_t id, double val);
uint32_t com_get_signal_count(void);
struct com_desc_t* com_add_signal(com_id_t id, char* name, double min, double max, double initial, uint8_t transport);
void com_print_signals(void);
struct com_desc_t* com_get_signal(com_id_t signal_id);
uint8_t com_reg_update_cb(com_id_t id, com_user_callback update_callback);

#endif //COM_H
