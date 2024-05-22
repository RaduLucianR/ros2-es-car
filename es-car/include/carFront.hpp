#ifndef CAR_FRONT_H
#define CAR_FRONT_H

#include "logConf.hpp"

/**
 * Logging of status messages
 */
#ifdef CAR_FRONT_INFO_MSG
  #define CAR_FRONT_PRINT_INFO( fmt, ...) fprintf( stdout, "[CAR FRONT] " fmt "\r\n", ##__VA_ARGS__)
#else
  #define CAR_FRONT_PRINT_INFO(fmt, ...)
#endif

/**
 * Logging of error messages
 */
#ifdef CAR_FRONT_PRINT_ERROR_MSG
  #define CAR_FRONT_PRINT_ERROR( fmt, ...) fprintf( stdout, "[CAR FRONT ERROR]" fmt " [%s:%d]\r\n", ##__VA_ARGS__, __FUNCTION__, __LINE__)
#else
  #define CAR_FRONT_PRINT_ERROR( fmt, ...)
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
#define SPI_CMD_SET_STEERING_SERVO  0x50
#define SPI_CMD_READ_RPM            0x51
#define SPI_CMD_READ_US_DISTANCE    0x52

int carFront_init(int spi, int cs);
void* carFront_deinit();
bool setSteering(double val_deg);
bool getDistance(double dist[2]);

#endif //SENSOR_FRONT_H
