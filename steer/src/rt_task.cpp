/**
 * @file rt_task.c
 * @author Matthias Becker
 * @brief Functionality to realize periodic tasks.
 * @date 2022-12-21
 *
 * @copyright Copyright (c) 2022
 *
 */
#define _GNU_SOURCE
#include <rt_task.hpp>
#include <signal.h>
#include <assert.h>
#include <sched.h>
#include <errno.h>
#include <string.h>

/*******************************************************************************
 * Private Variables
 ******************************************************************************/

/**
 * Start of the linked list with all thread information
 */
struct task_data_t* threads;

/**
 * Number of threads
 */
int thread_count = 0;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

/**
 * Holds the common release time of all threads
 */
struct timespec common_release;

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

void* periodic_task_skeleton(void * arg);
int set_affinity(int core_id);
void sleep_until_next_activation(struct task_data_t *tsk);
void thread_setCancelation();
struct task_data_t* find_taskData(pthread_t tid);

/*******************************************************************************
 * Function Code
 ******************************************************************************/

/**
 * Returns the number of all user tasks created.
 */
int get_thread_count(void) {
	return thread_count;
}

/**
 * Returns the pointer to the linked list of all tasks.
 */
struct task_data_t* get_thread_pointer(void) {
	return threads;
}

/**
 * Function to register a thread description to the system
 */
struct task_data_t* add_thread_description(thread_func_pointer _task_code, thread_func_pointer _init_code, thread_func_pointer _deinit_code, char* _name, int _period_ms, int _priority, int _core) {
	struct task_data_t* list_pointer = threads;

	struct task_data_t* new_thread = static_cast<task_data_t*>(malloc(sizeof(struct task_data_t)));
	if (new_thread != NULL) {
		new_thread->period.tv_sec = 0;
		new_thread->period.tv_nsec = MS_TO_NANOS(_period_ms);
		new_thread->priority = _priority;
		new_thread->core = _core;
		new_thread->name = _name;
		new_thread->task_code = _task_code;
    new_thread->init_code = _init_code;
    new_thread->deinit_code = _deinit_code;

		if (_period_ms <= 0) {
			new_thread->periodic = false;
		} else {
			new_thread->periodic = true;
		}
		// get the end of the list of threads
		if (list_pointer == NULL) {
			threads = new_thread;
		} else {
			while (list_pointer->next != NULL) {
				list_pointer = list_pointer->next;
			}
			list_pointer->next = new_thread;
		}

		thread_count++;

		return new_thread;

	} else {
		RT_PRINT_ERROR("Could not create new thread");
		return NULL;
	}
}

/**
 * This function creates all defined threads.
 */
void create_tasks(void){
	pthread_attr_t my_attr;
	pthread_attr_init(&my_attr);
	//pthread_attr_setinheritsched(&my_attr,PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&my_attr, SCHED_FIFO);
	struct sched_param param;

	struct task_data_t* list_pointer = threads;

	if (list_pointer != NULL) {
		do {
			param.sched_priority = list_pointer->priority;
			pthread_attr_setschedparam(&my_attr, &param);
			int err = pthread_create(&list_pointer->task_id, &my_attr, periodic_task_skeleton, (void *) list_pointer);
			if (err != 0) {
				RT_PRINT_ERROR("ERROR: Could not create new thread", __FILE__, __LINE__);
				RT_PRINT_ERROR("%i %s", err, strerror(err));
			}
			list_pointer = list_pointer->next;

		} while (list_pointer != NULL);
	}

	sleep(1);
	return;
}

/**
 * Set the time all jobs see as the initial release. This is currently 2ms after all threads are released in order to give enough time for each
 * thread to execute the initialization code.
 */
void set_common_release() {
	struct timespec release_delay;
	release_delay.tv_sec = 0;
	release_delay.tv_nsec = 5000000;

	int err = clock_gettime(CLOCK_MONOTONIC, &common_release);
	assert(err == 0);

	timespec_add(&common_release, &common_release, &release_delay);
}

/**
 * Returns the common release time of all threads
 */
struct timespec* get_common_release_time() {
	return &common_release;
}

/*
void timespec_monodiff_rml(struct timespec *ts_out,
                                    const struct timespec *ts_in) {

    ts_out->tv_sec = ts_in->tv_sec - ts_out->tv_sec;
    ts_out->tv_nsec = ts_in->tv_nsec - ts_out->tv_nsec;
    if (ts_out->tv_sec < 0) {
        ts_out->tv_sec = 0;
        ts_out->tv_nsec = 0;
    } else if (ts_out->tv_nsec < 0) {
        if (ts_out->tv_sec == 0) {
            ts_out->tv_sec = 0;
            ts_out->tv_nsec = 0;
        } else {
            ts_out->tv_sec = ts_out->tv_sec - 1;
            ts_out->tv_nsec = ts_out->tv_nsec + TIMING_GIGA;
        }
    } else {}
}

int clock_nanosleep_abstime ( const struct timespec *req )
{
    struct timespec ts_delta;
    int retval = clock_gettime ( CLOCK_MONOTONIC, &ts_delta );
    if (retval == 0) {
        timespec_monodiff_rml ( &ts_delta, req );
        retval = nanosleep ( &ts_delta, NULL );
    }
    return retval;
}
*/

/**
 * Wait until it’s time for the next “job” of the task.
 */
void sleep_until_next_activation(struct task_data_t *tsk) {
	int err;
	do {
		// perform an absolute sleep until tsk->current_activation
		//printf("Thread_%d going to sleep until %ds %dns\r\n", tsk->thread_id, tsk->current_activation.tv_sec, tsk->current_activation.tv_nsec);
		//err = clock_nanosleep_abstime(&tsk->current_activation);
		err = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tsk->current_activation, NULL);
		if (err == EINTR) {
			RT_PRINT_INFO("Sleep till next activation of task %s got interrupted.", tsk->name);
			return;
		}
		// if err is nonzero, we might have woken up too early
	} while (err != 0 && errno == EINTR);
	assert(err == 0);
}

/**
 * Pin the task to a specific core.
 */
int set_affinity(int core_id) {
   int num_cores = sysconf(_SC_NPROCESSORS_ONLN);
   if (core_id < 0 || core_id >= num_cores)
      return EINVAL;

   cpu_set_t cpuset;
   CPU_ZERO(&cpuset);
   CPU_SET(core_id, &cpuset);

   pthread_t current_thread = pthread_self();
   return pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
}

/**
 * Trigger the common release of all tasks
 */
void release_condition(){
	RT_PRINT_INFO("Now releasing all threads");
	set_common_release();	//set the time all threads start executing
	pthread_cond_broadcast(&cond);
}

/**
 * Calls the init function of each registered task.
 * If one task initialization fails -1 is returned.
 * Otherwise 0 is returned on success.
 */
int initTasks() {
	struct task_data_t* list_pointer = threads;

	if (list_pointer != NULL) {
		do {
			if (list_pointer->init_code(list_pointer) != 0) {
				RT_PRINT_ERROR("Could not initialize Task %s", list_pointer->name);
				return -1;
			}
			RT_PRINT_INFO("Initialized Task %s", list_pointer->name);
			list_pointer = list_pointer->next;

		} while (list_pointer != NULL);
	}
	return 0;
}

/**
 * Enables cancelation points for a thread and sets the cancelation type to deferred.
 */
void thread_setCancelation() {
	int err;

	err = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
	if (err != 0) {
		RT_PRINT_ERROR("Can't set cancellation state: %s", strerror(err));
	}

	pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
	if (err != 0) {
		RT_PRINT_ERROR("Can't set cancellation type: %s", strerror(err));
	}
}

/**
 * Frame for a periodic task. This is the basis for most of the tasks we use.
 * The user provided functions will be called for each job and during init and
 * deinit.
 */
void* periodic_task_skeleton(void * arg) {
  struct task_data_t *ps = (struct task_data_t *) arg;
  ps->current_job_id = 1;
  ps->terminated = 0;
  clockid_t cid;

	thread_setCancelation();

  //pthread_getcpuclockid(pthread_self(), &cid);
	set_affinity(ps->core);

	/* NOTE: The init function is called in main before tasks are created */
  /* Call the task specific initialization code if it was registered */
  //if (ps->init_code != NULL) {
  //  ps->init_code(ps);
  //}

  /**
	 * Wait for synchronous release
	 */
	pthread_mutex_lock(&mutex);
	RT_PRINT_INFO("[Task %s] Waiting for release", ps->name);
	pthread_cond_wait(&cond, &mutex);
	pthread_mutex_unlock(&mutex); // unlocking for all other threads

  ps->current_activation = *get_common_release_time();

  while (!ps->terminated) {
		if (ps->periodic == true) {							//only sleep if this is a periodic task
    	sleep_until_next_activation(ps); 	// Wait until release of next job
		}

    //execute...
    ps->task_code(ps);

    // Advance the job count in preparation of the next job
		ps->current_job_id++;

    // Compute the next activation time if this is a periodic task
		if (ps->periodic == true) {
			timespec_add(&ps->current_activation, &ps->period, &ps->current_activation);
		}
  }

  /* Call the task specific deinitialization code if it was registered */
  if (ps->deinit_code != NULL) {
		RT_PRINT_INFO("Deinit task %s", ps->name);
    ps->deinit_code(ps);
  }

  return 0;
}

/**
 * Wait for all application threads to complete before terminating the program.
 */
void join_threads() {
	struct task_data_t* list_pointer = threads;

	RT_PRINT_INFO("Waiting for threads to finish");
	if (list_pointer != NULL) {
		do {
		pthread_join(list_pointer->task_id, NULL);

		list_pointer = list_pointer->next;
		} while (list_pointer != NULL);
	}
	RT_PRINT_INFO("All threads finished");
	return;
}

/**
 * Called from signal handler.
 * Terminates all tasks.
 */
void stopTasks(int signum){

	if (signum != SIGUSR1) {
  	RT_PRINT_INFO("SIGNAL HANDLER -- Terminating tasks.");
  	terminate_tasks();
	} else {
		struct task_data_t* task_data = find_taskData(pthread_self());
		if (task_data != NULL) {
			RT_PRINT_INFO("Termination signal for thread %s.", task_data->name);
		} else {
			RT_PRINT_INFO("Termination signal for unknown thread.");
		}
	}
}

/**
 * This function sets the termination variable for each task to 1.
 * This leads all tasks to stop executing after they completed
 * the next task instance.
 */
void terminate_tasks(void) {
	struct task_data_t* list_pointer = threads;

	if (list_pointer != NULL) {
    do {
    	list_pointer->terminated = 1;
			pthread_kill(list_pointer->task_id, SIGUSR1);
    	list_pointer = list_pointer->next;
    } while (list_pointer != NULL);
  }
}

/**
 * Helper function to get the task data for a specific task id
 */
struct task_data_t* find_taskData(pthread_t tid) {
	struct task_data_t* list_pointer = threads;

	if (list_pointer != NULL) {
    do {
    	if (list_pointer->task_id == tid) return list_pointer;
    	list_pointer = list_pointer->next;
    } while (list_pointer != NULL);
  }
	return NULL;
}

/**
 * This function prints a list of all configured signals on stdout.
 */
void print_tasks(void) {
  struct task_data_t* list_pointer = threads;

#ifdef RT_INFO_MSG

	printf("\n---------------------------------------------------------\n");
	printf("| Name                      | Period  | Priority | Core |\n");
	printf("---------------------------------------------------------\n");

	if (list_pointer != NULL) {
		do {
			if (list_pointer->periodic == true) {

			printf("| %25s | %4d ms |   %4d   |  %2d  |\n", list_pointer->name,
																					         (list_pointer->period.tv_nsec / 1000000UL),
																					         list_pointer->priority,
																					         list_pointer->core);
			} else {
				printf("| %25s |    - ms |   %4d   |  %2d  |\n", list_pointer->name,
																						         list_pointer->priority,
																						         list_pointer->core);
			}
			list_pointer = list_pointer->next;
		} while (list_pointer != NULL);
	} else {
		printf("list pointer is NULL\r\n");
	}

	printf("---------------------------------------------------------\n");
#endif //RT_INFO_MSG
}
