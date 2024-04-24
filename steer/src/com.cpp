/**
 * @file com.c
 * @author Matthias Becker
 * @brief Communication handler
 * @date 2022-12-21
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <string.h>
#include "com.hpp"

/*******************************************************************************
 * Private Variables
 ******************************************************************************/
struct com_desc_t* signals; /* Pointer to the list of registered signals */
uint32_t numSig;            /* Number of registered signals */

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/
int64_t getTimestamp_ns(void);

/*******************************************************************************
 * Function Implementation
 ******************************************************************************/

/**
 * Returns the number of signals that have been registered.
 */
uint32_t com_get_signal_count(void) {
  return numSig;
}

/**
 * Register a new signal to the system
 */
struct com_desc_t* com_add_signal(com_id_t id, char* name, double min, double max, double initial, uint8_t transport) {
  struct com_desc_t* list_pointer = signals;

  struct com_desc_t* new_signal = static_cast<com_desc_t*>(malloc(sizeof(struct com_desc_t)));
  if (new_signal != NULL) {
    new_signal->id = id;
    new_signal->name = name;
    new_signal->min = min;
    new_signal->max = max;
    new_signal->transport = transport;
    new_signal->value.val = initial;
    new_signal->value.ts = getTimestamp_ns();
    new_signal->update_cb = NULL;

    // get the end of the list of threads
		if (list_pointer == NULL) {
			signals = new_signal;
		} else {
			while (list_pointer->next != NULL) {
				list_pointer = list_pointer->next;
			}
			list_pointer->next = new_signal;
		}

		numSig++;

		return new_signal;

  } else {
		COM_PRINT_ERROR("ERROR: Could not create new signal (%s, %d)", __FILE__, __LINE__);
		return NULL;
	}
}

/**
 * Register an update callback for this signal.
 * There can be multiple update callbacks for each signal.
 */
uint8_t com_reg_update_cb(com_id_t id, com_user_callback update_callback) {

  struct com_desc_t* signal_ptr = com_get_signal(id);
  struct update_cbk** cb_ptr= &(signal_ptr->update_cb);

  struct update_cbk* new_cb = static_cast<update_cbk*>(malloc(sizeof(struct update_cbk)));
  new_cb->update_cb = update_callback;

  if (new_cb != NULL) {
    if (*cb_ptr == NULL) {
			*cb_ptr = new_cb;
		} else {
			while ((*cb_ptr)->next != NULL) {
				*cb_ptr = (*cb_ptr)->next;
			}
			(*cb_ptr)->next = new_cb;
		}
    COM_PRINT_INFO("Registered Update Callback for signal %s", signal_ptr->name);
    return 0;
  } else {
		COM_PRINT_ERROR("Could not create new update callback for %s (%s, %d)", signal_ptr->name,__FILE__, __LINE__);
		return 1;
	}
}

/**
 * Function to read a signal value. This function is called from the user code
 * and reads the current value from shared memory.
 */
uint8_t com_read(com_id_t id, double* val, int64_t* ts) {
  struct com_desc_t* signal_ptr = com_get_signal(id);

  COM_PRINT_INFO("Read Signal %s", signal_ptr->name);

  *val = signal_ptr->value.val;
  *ts = signal_ptr->value.ts;

  return 0;
}

/**
 * This function writes a new value to a signal. The shared memory value is
 * always updated. If a signal is registered for COM_NET the value is sent to
 * the multicast group as well.
 */
uint8_t com_write(com_id_t id, double val) {

  /* Get the signal description based on the signal id */
  struct com_desc_t* sig = com_get_signal(id);

  /* Prepare the signal with the current timestamp */
  signal_val_t sigUpdate;
  sigUpdate.ts = getTimestamp_ns();
  sigUpdate.val = val;

  /* Check the signal boundary values */
  if (sigUpdate.val > sig->max) sigUpdate.val = sig->max;
  else if (sigUpdate.val < sig->min) sigUpdate.val = sig->min;

  /* Store the new value in the local memory */
  sig->value.ts = sigUpdate.ts;
  sig->value.val = sigUpdate.val;

  COM_PRINT_INFO("ts: %9llu val: %4.3lf write -> %s", sigUpdate.ts, sigUpdate.val, sig->name);

  /* Check if a user callbacks are registered for this signal */
  if (sig->update_cb != NULL) {
    struct update_cbk* cbk = sig->update_cb;
    do{
      cbk->update_cb(sigUpdate);
      cbk = cbk->next;
    } while(cbk != NULL);
  }

  /* Check if the signal should be distributed to other nodes */
  if (sig->transport == COM_UDP) {
    udp_msg_t udpMsg;
    udpMsg.id = sig->id;
    udpMsg.val = sigUpdate.val;
    if (udp_send(udpMsg) != 0) {
      COM_PRINT_ERROR("Could not send UDP msg for signal %s", sig->name);
    }
  }
  if (sig->transport == COM_CAN) {
    can_msg_t canMsg;
    canMsg.com_id = sig->id;
    memcpy(canMsg.payload, &sigUpdate.val, sizeof(sigUpdate.val));
    canMsg.length = sizeof(sigUpdate.val);
    if (can_send(canMsg) != 0) {
      COM_PRINT_ERROR("Could not send CAN msg for signal %s", sig->name);
    }
  }
}

/**
 * Function to return the signal description with the specified id.
 */
struct com_desc_t* com_get_signal(com_id_t signal_id) {
  struct com_desc_t* list_pointer = signals;

  while (list_pointer != NULL) {
    if (list_pointer->id == signal_id) return list_pointer;
    list_pointer = list_pointer->next;
  }
  return NULL;
}

/**
 * Get the current timestamp and compensate for the offset at program startup.
 * We assume the first call to this function will set the reference time 0.
 */
int64_t getTimestamp_ns(void) {
//	static long long startTime = 0;
	long long 			ns;		// Nanoseconds
  struct timespec spec;

  clock_gettime(CLOCK_REALTIME, &spec);

	/**
	 * We compenssate for the start time. Yet we can still use the same function
	 * to get the value for the startTime as it is initially set to 0.
	 */
	ns = spec.tv_nsec + (spec.tv_sec * 1000000000);// - startTime;

//if (startTime == 0) {
//		startTime = ns; // set the offset to compensate future readings
//    ns = 0;         // return 0 as this is the start time
//	}
	return ns;
}

/**
 * This function prints a list of all configured signals on stdout.
 */
void com_print_signals(void) {
  struct com_desc_t* list_pointer = signals;

#ifdef COM_INFO_MSG
	printf("\n-------------------------------------------------------------------\n");
	printf("| ID | Name                      | Min     | Max      | UDP       |\n");
	printf("-------------------------------------------------------------------\n");

	if (list_pointer != NULL) {
		do {
			printf("| %2i | %25s | %06.2f  | %07.2f  |", list_pointer->id,
																					         list_pointer->name,
																					         list_pointer->min,
																					         list_pointer->max);

			if (list_pointer->transport == 0) printf(" COM_LOCAL |\n");
			if (list_pointer->transport == 1) printf(" COM_NET   |\n");

			list_pointer = list_pointer->next;
		} while (list_pointer != NULL);
	} else {
		printf("list pointer is NULL\r\n");
	}

	printf("-------------------------------------------------------------------\n");
#endif //COM_INFO_MSG
}
