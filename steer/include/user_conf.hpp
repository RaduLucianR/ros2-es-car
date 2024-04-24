/**
 * @file user_conf.h
 * @author Matthias Becker
 * @brief Configuration of the RT-Car application
 * @date generated file
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef USER_CONF_H
#define USER_CONF_H

#include "com.hpp"
#include "logConf.hpp"
#include "carFront.hpp"
#include "udp.hpp"

/**
 * Static IP Address of this node
 */
#define LOCAL_IP_ADDRESS "192.168.0.4"

/**
 * Platform Defines
 */
#define MAX_US_DISTANCE	(4.0)
#define MIN_US_DISTANCE	(0.0)
#define MAX_TOF_DISTANCE	(2.0)
#define MIN_TOF_DISTANCE	(0.0)
#define MAX_RPM	(1000.0)
#define MIN_RPM	(0.0)
#define MAX_STEERING	(20)
#define MIN_STEERING	(-20)
#define MAX_ESC	(180)
#define MIN_ESC	(80)
#define UDP_PERIOD_MS	(100)

/**
 * Signal Names
 */
#define SIG_STEERING	0
#define SIG_ESC	1
#define SIG_DISTANCE_FRONT_LEFT	2
#define SIG_DISTANCE_FRONT_RIGHT	3
#define SIG_DISTANCE_BACK_LEFT	4
#define SIG_DISTANCE_BACK_RIGHT	5
#define SIG_SPEED_LEFT	6
#define SIG_SPEED_RIGHT	7
#define SIG_DISTANCE_SIDE_RIGHT	8
#define SIG_DISTANCE_SIDE_LEFT	9
#define SIG_DISTANCE_FRONT_CENTER	10
#define SIG_EMERGENCY_BRAKE	11

/**
 * Logging of status messages
 */
#ifdef MAIN_INFO_MSG
  #define MAIN_PRINT_INFO( fmt, ...) fprintf( stdout, "[MAIN] " fmt "\r\n", ##__VA_ARGS__)
#else
  #define MAIN_PRINT_INFO(fmt, ...)
#endif

/**
 * Logging of error messages
 */
#ifdef MAIN_PRINT_ERROR_MSG
  #define MAIN_PRINT_ERROR( fmt, ...) fprintf( stdout, "[MAIN ERROR]" fmt " [%s:%d]\r\n", ##__VA_ARGS__, __FUNCTION__, __LINE__)
#else
  #define MAIN_PRINT_ERROR( fmt, ...)
#endif

/**
 * Register Tasks
 */
#define REGISTER_TASKS  MAIN_PRINT_INFO("Register Tasks");\
				add_thread_description(carFront_job, carFront_init, carFront_deinit, "Car Front", 150, 200, 2);\
				//add_thread_description(udp_job, udp_init, udp_deinit, "UDP Server", UDP_PERIOD_MS, 200, 2);\

/**
 * Register Signals
 */

#define REGISTER_SIGNALS  MAIN_PRINT_INFO("Register Signals");\
				com_add_signal(SIG_STEERING, "Steering Angle", MIN_STEERING, MAX_STEERING, ((MAX_STEERING - MIN_STEERING) / 2), COM_LOCAL);\
				com_add_signal(SIG_ESC, "ESC Value", MIN_ESC, MAX_ESC, ((MAX_ESC - MIN_ESC) / 2), COM_LOCAL);\
				com_add_signal(SIG_DISTANCE_FRONT_LEFT, "Dist. Front Left [m]", MIN_US_DISTANCE, MAX_US_DISTANCE, 0, COM_UDP);\
				com_add_signal(SIG_DISTANCE_FRONT_RIGHT, "Dist. Front Right [m]", MIN_US_DISTANCE, MAX_US_DISTANCE, 0, COM_UDP);\
				com_add_signal(SIG_DISTANCE_BACK_LEFT, "Dist. Back Left [m]", MIN_US_DISTANCE, MAX_US_DISTANCE, 0, COM_LOCAL);\
				com_add_signal(SIG_DISTANCE_BACK_RIGHT, "Dist. Back Right [m]", MIN_US_DISTANCE, MAX_US_DISTANCE, 0, COM_LOCAL);\
				com_add_signal(SIG_SPEED_LEFT, "Speed Left", MIN_RPM, MAX_RPM, 0, COM_UDP);\
				com_add_signal(SIG_SPEED_RIGHT, "Speed Right", MIN_RPM, MAX_RPM, 0, COM_UDP);\
				com_add_signal(SIG_DISTANCE_SIDE_RIGHT, "Dist. Side Right [m]", MIN_TOF_DISTANCE, MAX_TOF_DISTANCE, 0, COM_LOCAL);\
				com_add_signal(SIG_DISTANCE_SIDE_LEFT, "Dist. Side Left [m]", MIN_TOF_DISTANCE, MAX_TOF_DISTANCE, 0, COM_LOCAL);\
				com_add_signal(SIG_DISTANCE_FRONT_CENTER, "Dist. Front Center [m]", MIN_TOF_DISTANCE, MAX_TOF_DISTANCE, 0, COM_LOCAL);\
				com_add_signal(SIG_EMERGENCY_BRAKE, "Emergency Brake", 0, 100, 0, COM_LOCAL);\

#endif //USER_CONF_H
