/**
 * @file CanServer.c
 * @author Matthias Becker
 * @brief Communication handler
 * @date 2023-02-24
 *
 * @copyright Copyright (c) 2023
 *
 */
 #include <stdio.h>
 #include <unistd.h>
 #include <stdlib.h>
 #include <string.h>
 #include <errno.h>
 #include <net/if.h>
 #include <sys/ioctl.h>
 #include <sys/socket.h>
 #include <linux/can.h>
 #include <linux/can/raw.h>
 #include "can.hpp"
 #include "com.hpp"
 #include "user_conf.hpp"

/*******************************************************************************
 * Private Variables
 ******************************************************************************/
int canfd;  // File descriptor for the receive socket.

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Function Implementation
 ******************************************************************************/

/**
 * Function is executed once at initialization
 */
void* can_init(void * arg) {
  struct ifreq ifr;
  struct sockaddr_can addr;

  /* Create a socket */
  if ((canfd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    CAN_PRINT_ERROR("Create Socket: %s (%i)", strerror(errno), errno);
    return (void*)1;
  }

  /* Retrieve the interface id of the CAN interface we want to connect to */
  strcpy(ifr.ifr_name, "can1" );
	ioctl(canfd, SIOCGIFINDEX, &ifr);

  /*
  * Set a timepout for the receive call.
  * -> 500 us blocking on rcv call.
  */
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 500;
  if (setsockopt(canfd, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0) {
     CAN_PRINT_ERROR("Error");
     return RT_ERROR;
  }

  memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

  /* Bind the socket to the CAN interface */
	if (bind(canfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		CAN_PRINT_ERROR("Could not bind: %s (%i)", strerror(errno), errno);
		return RT_ERROR;
	}

  return RT_SUCCESS;
}

/**
 * Function is executed once at shutdown
 */
void* can_deinit(void * arg) {
  if (close(canfd) < 0) {
  	CAN_PRINT_ERROR("Could not close socket: %s (%i)", strerror(errno), errno);
  	return (void*)1;
  }

  return (void*)0;
}

/**
 * Function is executed for every job of the thread
 */
void* can_job(void * arg) {
  struct can_frame frame;
  int i;
  int nbytes;
  bool rcvFrame = true;

  /* Read as many frames as available. */
  while (rcvFrame == true) {

    nbytes = read(canfd, &frame, sizeof(struct can_frame));
    if (nbytes < 0) {
      if (errno != EAGAIN) {  // We expect EAGAIN if the call timed out (see timeout config above)
    	   CAN_PRINT_ERROR("Could not receive frame: %s (%i)", strerror(errno), errno);
         return (void*)1;
      }
    	rcvFrame = false;
    } else {
      /* Handle the CAN frame */

      //For now just print the content on stdio
      printf("0x%03X [%d] ",frame.can_id, frame.can_dlc);
      for (i = 0; i < frame.can_dlc; i++)
      	printf("%02X ",frame.data[i]);
    	printf("\r\n");

      /* Get the signal id and value before notifying the com layer */
/*      uint8_t sigId = frame.can_id - CAN_ID_OFFSET;
      if (frame.can_dlc == sizeof(double)) {
        double value = (double)frame.data;
        com_write(sigId, value);
      } else {
        CAN_PRINT_ERROR("Received frame of wrong size %i expected %i", frame.can_dlc, sizeof(double));
      }
*/
    }
  }
  return (void*)0;
}

/**
 * Send a comm message on the CAN bus
 */
void* can_send(can_msg_t msg) {
  struct can_frame frame;

  /* Prepare the can frame */
  frame.can_id = CAN_ID_OFFSET + msg.com_id;
  frame.can_dlc = msg.length;
  memcpy(frame.data, msg.payload, msg.length);

  if (write(canfd, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
    CAN_PRINT_ERROR("Could not send frame: %s (%i)", strerror(errno), errno);
    return (void*)1;
  }
  return (void*)0;
}
