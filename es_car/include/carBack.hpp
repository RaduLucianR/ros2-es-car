#ifndef CAR_BACK_H
#define CAR_BACK_H

#include "logConf.hpp"

/**
 * Logging of status messages
 */
#ifdef CAR_BACK_INFO_MSG
  #define CAR_BACK_PRINT_INFO( fmt, ...) fprintf( stdout, "[CAR BACK] " fmt "\r\n", ##__VA_ARGS__)
#else
  #define CAR_BACK_PRINT_INFO(fmt, ...)
#endif

/**
 * Logging of error messages
 */
#ifdef CAR_BACK_PRINT_ERROR_MSG
  #define CAR_BACK_PRINT_ERROR( fmt, ...) fprintf( stdout, "[CAR BACK ERROR]" fmt " [%s:%d]\r\n", ##__VA_ARGS__, __FUNCTION__, __LINE__)
#else
  #define CAR_BACK_PRINT_ERROR( fmt, ...)
#endif

#define SPI_PATH "/dev/spidev1.0"

#define SPI_SS_PIN48    48
#define SPI_SS_PIN60    60
#define BITS_PER_WORD   8
#define SPI_MODE        0
#define SPI_SPEED       100000

/**
 * Commands used for SPI communication
 */
#define SPI_CMD_INACTIVE            0x99
#define SPI_CMD_SET_ESC             0x60
#define SPI_CMD_READ_US_DISTANCE    0x61

int carBack_init(int spi, int cs);
void* carBack_deinit();
bool setEsc(double val_percent);

#endif //SENSOR_FRONT_H
