#ifndef LOGCONF_H
#define LOGCONF_H

/**********************************************************
 * Uncomment respective defines to enable logging and
 * error output of a module.
 * Note that not all modules are present on all nodes.
 **********************************************************/

/**
 * COM Module
 */
//#define COM_INFO_MSG
#define COM_PRINT_ERROR_MSG

/**
 * UDP Module
 */
//#define UDP_INFO_MSG
#define UDP_PRINT_ERROR_MSG

/**
 * Car Front Module
 */
//#define CAR_FRONT_INFO_MSG
#define CAR_FRONT_PRINT_ERROR_MSG

/**
 * Car Back Module
 */
//#define CAR_BACK_INFO_MSG
#define CAR_BACK_PRINT_ERROR_MSG

/**
 * Dashboard Module
 */
//#define DASH_INFO_MSG
#define DASH_PRINT_ERROR_MSG

/**
 * Remote Command Module
 */
//#define RCMD_INFO_MSG
#define RCMD_PRINT_ERROR_MSG

/**
 * FileCom Module
 */
#define FILECOM_INFO_MSG
#define FILECOM_PRINT_ERROR_MSG

/**
 * RTPPL Connector Module
 */
#define RTPPLCON_INFO_MSG
#define RTPPLCON_PRINT_ERROR_MSG

/**
 * Time of Flight Sensor Module
 */
//#define TOF_INFO_MSG
#define TOF_PRINT_ERROR_MSG

/**
 * CAN Module
 */
//#define CAN_INFO_MSG
#define CAN_PRINT_ERROR_MSG

/**
 * I2C Module
 */
//#define I2C_INFO_MSG
#define I2C_ERROR_MSG

/**
 * SPI Module
 */
//#define SPI_INFO_MSG
#define SPI_PRINT_ERROR_MSG

/**
 * Real-Time Task Module
 */
#define RT_INFO_MSG
#define RT_PRINT_ERROR_MSG

/**
 * Main Module
 */
#define MAIN_INFO_MSG
#define MAIN_PRINT_ERROR_MSG

/**
 * VL53L0X Module
 */
//#define PRINT_VL53L0X_INFO_MSG
#define PRINT_VL53L0X_ERROR_MSG

#endif //LOGCONF_H
