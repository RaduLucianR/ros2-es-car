/**
 * @file carFront.c
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

#include "rt_task.hpp"
#include "user_conf.hpp"
#include "com.hpp"
#include "spidriver.hpp"
#include "carFront.hpp"

// ########## ROS DEP ########
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

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
static unsigned int spi_fd;
static bool initialized = false;
static pthread_mutex_t spi_mutex;

static bool setSteering(double val_deg);
static bool getDistance(void);
static bool getRpm(void);

void* steeringCallback(signal_val_t signal);

void* carFront_init(void * arg) {
  struct task_data_t *ps = (struct task_data_t *) arg;

  spi_fd = SPI_open(0, 1); // ######## WARNING #######: this is (1, 0) in the OG code
  if (spi_fd < 0) {
      CAR_FRONT_PRINT_ERROR("Could not open SPI bus %s", SPI_PATH);
      return RT_ERROR;
  }

  SPI_setMaxFrequency(spi_fd, SPI_SPEED);
  SPI_setBitsPerWord(spi_fd, BITS_PER_WORD);
  SPI_setMode(spi_fd, SPI_MODE);

  initialized = true;

  pthread_mutex_init(&spi_mutex, NULL);

  com_reg_update_cb(SIG_STEERING, steeringCallback);
  CAR_FRONT_PRINT_INFO("Initialized!");

  return RT_SUCCESS;
}

void* carFront_deinit(void * arg) {
  struct task_data_t *ps = (struct task_data_t *) arg;

  SPI_close(spi_fd);
  initialized = false;

  CAR_FRONT_PRINT_INFO("Deinitialized!");

  return 0;
}

void* carFront_job(void * arg) {
  struct task_data_t *ps = (struct task_data_t *) arg;

  CAR_FRONT_PRINT_INFO("Hello from the thread instance %lu!", ps->current_job_id);

  // Read all sensor values (results are written to COM directly)
  getDistance();
  getRpm();

  return 0;
}

static bool setSteering(double val_deg) {
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
  uint8_t payload_in  = 0xff;

  CAR_FRONT_PRINT_INFO("Sent command SPI_CMD_SET_STEERING_SERVO -> %i!", value);

  // Write the value to the servo
  pthread_mutex_lock(&spi_mutex);
  write(spi_fd, &payload_out, 1);
  write(spi_fd, &value, 1);
  pthread_mutex_unlock(&spi_mutex);

  return retval;
}

static bool getDistance(void) {
  bool retval = true;

  uint8_t payload = SPI_CMD_READ_US_DISTANCE;
  uint8_t data[4];

  uint16_t sensorValue1, sensorValue2;

  pthread_mutex_lock(&spi_mutex);
  write(spi_fd, &payload, 1);
  SPI_read(spi_fd, data, 4);
  pthread_mutex_unlock(&spi_mutex);

  sensorValue1 = data[0] | (data[1] << 8);
  sensorValue2 = data[2] | (data[3] << 8);

  com_write(SIG_DISTANCE_FRONT_LEFT, (double)sensorValue2 / 100.0);
  com_write(SIG_DISTANCE_FRONT_RIGHT, (double)sensorValue1 / 100.0);

  CAR_FRONT_PRINT_INFO("Distance Left -> %lf m", (double)sensorValue2 / 100.0);
  CAR_FRONT_PRINT_INFO("Distance Right -> %lf m", (double)sensorValue2 / 100.0);
  return retval;
}

static bool getRpm(void) {
  bool retval = true;

  uint8_t payload = SPI_CMD_READ_RPM;
  uint8_t data[4];

  uint16_t sensorValue1, sensorValue2;

  pthread_mutex_lock(&spi_mutex);
  write(spi_fd, &payload, 1);
  SPI_read(spi_fd, data, 4);
  pthread_mutex_unlock(&spi_mutex);

  sensorValue1 = data[0] | (data[1] << 8);
  sensorValue2 = data[2] | (data[3] << 8);

  com_write(SIG_SPEED_LEFT,(double)sensorValue1);
  com_write(SIG_SPEED_RIGHT,(double)sensorValue2);
  CAR_FRONT_PRINT_INFO("RPM Left -> %lf m", (double)sensorValue1);
  CAR_FRONT_PRINT_INFO("RPM Right -> %lf m", (double)sensorValue2);
  return retval;
}

/**
 * Callback that is used when COM receives a new steering signal update.
 */
void* steeringCallback(signal_val_t signal) {
  CAR_FRONT_PRINT_INFO("Steering Callback Value -> %lf", signal.val);
  setSteering(signal.val);
}

// ##################### ROS #####################
class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      setSteering(std::stod(msg->data)); 
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  //######## SPI ############
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
  
  //setSteering(argv[1]);

  //SPI_close(spi_fd);
  //############ ROS ################
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

