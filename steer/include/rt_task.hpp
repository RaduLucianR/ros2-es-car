/**
 * @file rt_task.h
 * @author Matthias Becker
 * @brief Functionality to realize periodic tasks.
 * @date 2022-12-21
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef RT_TASK_H
#define RT_TASK_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <pthread.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include "logConf.hpp"

#define RT_SUCCESS   ((void*)0)
#define RT_ERROR     ((void*)-1) //OG IS UNCOMMENTED!!!

#define MS_TO_NANOS(X)   (X * 1000000UL)
#define MSEC_TO_USEC(X)  (X * 1000UL)
#define USEC_TO_NANOS(X) (X * 1000UL)

#define TIMING_GIGA (1000000000)

/**
 * Logging of status messages
 */
#ifdef RT_INFO_MSG
  #define RT_PRINT_INFO( fmt, ...) fprintf( stdout, "[RT] " fmt "\r\n", ##__VA_ARGS__)
#else
  #define RT_PRINT_INFO(fmt, ...)
#endif

/**
 * Logging of error messages
 */
#ifdef RT_PRINT_ERROR_MSG
  #define RT_PRINT_ERROR( fmt, ...) fprintf( stdout, "[RT ERROR]" fmt " [%s:%d]\r\n", ##__VA_ARGS__, __FUNCTION__, __LINE__)
#else
  #define RT_PRINT_ERROR( fmt, ...)
#endif

/* timespec difference (monotonic) right - left */

//#ifdef __MACH__
/* **** */
/* MACH */

    /* emulate clock_nanosleep for CLOCK_MONOTONIC and TIMER_ABSTIME */
  //  inline int clock_nanosleep_abstime ( const struct timespec *req )
  //  {
  //      struct timespec ts_delta;
  //      int retval = clock_gettime ( CLOCK_MONOTONIC, &ts_delta );
  //      if (retval == 0) {
  //          timespec_monodiff_rml ( &ts_delta, req );
  //          retval = nanosleep ( &ts_delta, NULL );
  //      }
  //      return retval;
  //  }

/* MACH */
/* **** */
//#else
/* ***** */
/* POSIX */

    /* clock_nanosleep for CLOCK_MONOTONIC and TIMER_ABSTIME */
//    inline clock_nanosleep_abstime( req ) {
//      clock_nanosleep ( CLOCK_MONOTONIC, TIMER_ABSTIME, (req), NULL );
//    }

/* POSIX */
/* ***** */
//#endif

/**
 * Prototype for the user task code that is called periodically
 */
typedef void* (*thread_func_pointer)(void*);

static inline void timespec_add (struct timespec *sum, const struct timespec *left, const struct timespec *right)
{
  sum->tv_sec = left->tv_sec + right->tv_sec;
  sum->tv_nsec = left->tv_nsec + right->tv_nsec;
  if (sum->tv_nsec >= 1000000000)
    {
      ++sum->tv_sec;
      sum->tv_nsec -= 1000000000;
    }
}

/**
 * Data structure to describe one task
 */
struct task_data_t{
  pthread_t           task_id;
  char*               name;
  struct timespec     period;
  bool                periodic;
  int                 priority;
  int                 core;
  thread_func_pointer task_code;
  thread_func_pointer init_code;
  thread_func_pointer deinit_code;
  int                 terminated;
  struct timespec     maxExecTime;
  unsigned long       current_job_id;
  struct timespec     current_activation;
  struct task_data_t* next;
};

struct task_data_t* add_thread_description(thread_func_pointer _task_code, thread_func_pointer _init_code, thread_func_pointer _deinit_code, char* _name, int _period_ms, int _priority, int _core);
void create_tasks(void);
void set_common_release();
struct timespec* get_common_release_time();
int initTasks();
void join_threads();
void release_condition();
void stopTasks(int signum);
void terminate_tasks(void);
void print_tasks(void);

#endif //RT_TASK_H
