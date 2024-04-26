/**
 * @file carBack.c
 * @author Matthias Becker
 * @brief Throttle
 * @date 2023-01-010
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <pthread.h>

#include "spidriver.hpp"
#include "carBack.hpp"

/**
 * Range of values for ESC command send to the controller
 */
#define ESC_MIN       (80)
#define ESC_MAX       (180)
#define ESC_RANGE     (ESC_MAX - ESC_MIN)

/**
 * Range of speed commands received as input
 */
#define ESC_MIN_IN    (-100)
#define ESC_MAX_IN    (100)
#define ESC_RANGE_IN  (ESC_MAX_IN - ESC_MIN_IN)

static int spi_fd;
static bool initialized = false;
static pthread_mutex_t spi_mutex;

int carBack_init() {
  spi_fd = SPI_open(0, 1);
  if (spi_fd < 0) {
      CAR_BACK_PRINT_ERROR("Could not open SPI bus %s", SPI_PATH);
      return 1;
  }

  SPI_setMaxFrequency(spi_fd, SPI_SPEED);
  SPI_setBitsPerWord(spi_fd, BITS_PER_WORD);
  SPI_setMode(spi_fd, SPI_MODE);

  initialized = true;

  pthread_mutex_init(&spi_mutex, NULL);

  CAR_BACK_PRINT_INFO("Initialized!");

  return 0;
}

void* carBack_deinit() {
  SPI_close(spi_fd);
  initialized = false;

  CAR_BACK_PRINT_INFO("Deinitialized!");

  return 0;
}

bool setEsc(double val_percent) {
  bool retval = true;
  uint8_t value = (int) val_percent;

  //make sure values are within the allowed range
  if(value < ESC_MIN) value = ESC_MIN;
  if(value > ESC_MAX) value = ESC_MAX;

  uint8_t payload_out = SPI_CMD_SET_ESC;

  CAR_BACK_PRINT_INFO("Sent command SPI_CMD_SET_ESC -> %i!", value);

  pthread_mutex_lock(&spi_mutex);
  write(spi_fd, &payload_out, 1);
  write(spi_fd, &value, 1);
  pthread_mutex_unlock(&spi_mutex);
  
  return retval;
}
