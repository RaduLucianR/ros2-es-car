/*
* The task checks if packets have been received. A timeout of 100ms is set.
*/
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <stdbool.h>
#include "udp.hpp"
#include "com.hpp"
#include "user_conf.hpp"

#define BUFSIZE 1024

int sockfd; /* socket */
int clientlen; /* byte size of client's address */
int sendlen; /* length of the send address */
struct sockaddr_in serveraddr; /* server's addr */
struct sockaddr_in clientaddr; /* client addr */
struct sockaddr_in sendaddr; /* sending addr */
struct hostent *hostp; /* client host info */
char buf[BUFSIZE]; /* message buf */
char *hostaddrp; /* dotted decimal host addr string */
int optval; /* flag value for setsockopt */
int n; /* message byte size */

void* udp_init(void * arg) {
   UDP_PRINT_INFO("Task Init!");

  /*
  * socket: create the parent socket
  */
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    UDP_PRINT_ERROR("ERROR opening socket");
    return RT_ERROR;
  }

  /* setsockopt: Handy debugging trick that lets
  * us rerun the server immediately after we kill it;
  * otherwise we have to wait about 20 secs.
  * Eliminates "ERROR on binding: Address already in use" error.
  */
  optval = 1;
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR,(const void *)&optval , sizeof(int));

  /*
  * Set a timepout for the receive call.
  * -> 500 us blocking on rcv call.
  */
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 500;
  if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0) {
     UDP_PRINT_ERROR("Error");
     return RT_ERROR;
  }

  /*
  * build the server's Internet address
  */
  bzero((char *) &serveraddr, sizeof(serveraddr));
  serveraddr.sin_family = AF_INET;
  serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);//htonl(INADDR_ANY);
  serveraddr.sin_port = htons((unsigned short)UDP_PORT);

  /*
  * bind: associate the parent socket with a port
  */
  if (bind(sockfd, (struct sockaddr *) &serveraddr,sizeof(serveraddr)) < 0){
    UDP_PRINT_ERROR("ERROR on binding");
    return RT_ERROR;
  }

  clientlen = sizeof(clientaddr);

#ifdef UDP_DISABLE_LOOPBACK
  /**
   * Disable loopback to avoid receiving own multicast messages
   */
  char loopch = 0;
  if(setsockopt(sockfd, IPPROTO_IP, IP_MULTICAST_LOOP, (char *)&loopch, sizeof(loopch)) < 0){
    UDP_PRINT_ERROR("Setting IP_MULTICAST_LOOP error");
    close(sockfd);
    return (void*)1;
  }
#endif

  /*
  * use setsockopt() to request that the kernel join a multicast group
  */
  struct ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = inet_addr(MULTICAST_GROUP);
  mreq.imr_interface.s_addr = inet_addr(LOCAL_IP_ADDRESS);//htonl(INADDR_ANY);
  if (setsockopt(sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*) &mreq, sizeof(mreq)) < 0){
      UDP_PRINT_ERROR("setsockopt");
      close(sockfd);
      return RT_ERROR;
  }

  /*
   * Setup the send address. Since we always send to the multicast group
   * we can do it once here.
   */
  bzero((char *)&sendaddr, sizeof(sendaddr));
  sendaddr.sin_family = AF_INET;
  sendaddr.sin_addr.s_addr = inet_addr(MULTICAST_GROUP);
  sendaddr.sin_port = htons(UDP_PORT);
  sendlen = sizeof(sendaddr);

   return RT_SUCCESS;
 }

 void* udp_deinit(void * arg) {
   UDP_PRINT_INFO("Task Deinit");
 }

 void* udp_job(void * arg) {
  UDP_PRINT_INFO("Task Instance");
  bool rcvPacket = true; // Flag to indicate that we keep reading packets until all packets are processed

  /**
   * Read and process all packets that arrived
   */
  while (rcvPacket == true) {
    /*
     * recvfrom: receive a UDP datagram from a client
     */
    bzero(buf, BUFSIZE);

    // ################ RADU'S CHANGE ###########
    // C++ complains that clientlen is usigned int but it receives an int. Thus, we cast it to socklen_t
    //n = recvfrom(sockfd, buf, BUFSIZE, 0,(struct sockaddr *) &clientaddr, &clientlen);
    socklen_t tempClientlen = static_cast<socklen_t>(clientlen);
    n = recvfrom(sockfd, buf, BUFSIZE, 0, (struct sockaddr *)&clientaddr, &tempClientlen);
    clientlen = static_cast<int>(tempClientlen);
    // ################ RADU'S CHANGE ############

    if (n < 0) {
      if (errno != EAGAIN) {  // We expect EAGAIN if the call timed out (see timeout config above)
        UDP_PRINT_ERROR("ERROR in recvfrom");
      }
      rcvPacket = false;  //error or timeout
    } else {
      // For now we expect each udp msg to contain one update only.
      // Timestamping is done in the com function call.
      if (n > sizeof(udp_msg_t)) {
        UDP_PRINT_ERROR("Received more data than expected! n = %i", n);
      } else {
        udp_msg_t* msg = (udp_msg_t*) buf;
        UDP_PRINT_INFO("Received a udp message! [Id: %i val: %4.3lf]", msg->id, msg->val);
        com_write(msg->id, msg->val);
      }
    }
  }

  return 0;
}

/**
 * Send a comm message to the multicast group
 */
void* udp_send(udp_msg_t msg) {
  /*
   * sendto: echo the input back to the client
   */
  n = sendto(sockfd, &msg, sizeof(msg), 0,
         (struct sockaddr *) &sendaddr, sendlen);
  if (n < 0) {
    UDP_PRINT_ERROR("ERROR in sendto");
    return (void*)1;
  } else {
    UDP_PRINT_INFO("Sent a udp message! [Id: %i val: %4.3lf]", msg.id, msg.val);
  }
  return 0;
}
