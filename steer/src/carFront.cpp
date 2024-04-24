/**
 * @file carFront.cpp
 * @author Matthias Becker
 * @brief Ultrasonic distance sensor and RPM task
 * @date 2023-01-07
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <pthread.h> 

#include "spidriver.hpp"
#include "carFront.hpp"

/**
 * Range of values for steering commands, smaller values are left, larger are right
 */
#define STEERING_MIN_PWM    50
#define STEERING_MAX_PWM    100
#define STEERING_RANGE_PWM  (STEERING_MAX_PWM - STEERING_MIN_PWM)

/**
 * Range of values for the steering we receive as input
 */
#define STEERING_MIN_DEG    (-20)
#define STEERING_MAX_DEG    (20)
#define STEERING_RANGE_DEG  (STEERING_MAX_DEG - STEERING_MIN_DEG)

/**
 * Declarations
 */
static int spi_fd;
static bool initialized = false;
static pthread_mutex_t spi_mutex;

int carFront_init() {
  spi_fd = SPI_open(0, 0); // ######## WARNING #######: this is (1, 0) in the OG code
  
  if (spi_fd < 0) {
      CAR_FRONT_PRINT_ERROR("Could not open SPI bus %s", SPI_PATH);
      return 1;
  }

  SPI_setMaxFrequency(spi_fd, SPI_SPEED);
  SPI_setBitsPerWord(spi_fd, BITS_PER_WORD);
  SPI_setMode(spi_fd, SPI_MODE);

  initialized = true;

  pthread_mutex_init(&spi_mutex, NULL);
  
  return 0;
}

void* carFront_deinit() {

  SPI_close(spi_fd);
  initialized = false;

  CAR_FRONT_PRINT_INFO("Deinitialized!");

  return 0;
}

bool setSteering(double val_deg) {
  bool retval = true;

  // Compute the PWM value based on the steering input
  double pwmIncrementDeg = STEERING_RANGE_PWM / STEERING_RANGE_DEG;
  double pwm_ref = STEERING_MIN_PWM + (STEERING_RANGE_PWM / 2);
  uint8_t value = (uint8_t)(pwm_ref + (pwmIncrementDeg * (-val_deg)));
  CAR_FRONT_PRINT_INFO("Computed steering value -> %i for %lf degree!", value, val_deg);

  //make sure values are within the allowed range
  if(value < STEERING_MIN_PWM) value = STEERING_MIN_PWM;
  if(value > STEERING_MAX_PWM) value = STEERING_MAX_PWM;

  uint8_t payload_out = SPI_CMD_SET_STEERING_SERVO;

  CAR_FRONT_PRINT_INFO("Sent command SPI_CMD_SET_STEERING_SERVO -> %i!", value);

  // Write the value to the servo
  pthread_mutex_lock(&spi_mutex);
  write(spi_fd, &payload_out, 1);
  write(spi_fd, &value, 1);
  pthread_mutex_unlock(&spi_mutex);

  return retval;
}
