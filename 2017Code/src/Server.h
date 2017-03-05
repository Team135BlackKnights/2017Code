/*
 * Server.h
 *
 *  Created on: Jan 23, 2017
 *      Author: robot
 */

#ifndef SRC_CLASSES_SERVER_H_
#define SRC_CLASSES_SERVER_H_

#include<stdio.h>
#include<string.h>
#include<unistd.h>
#include<sys/socket.h>
#include<sys/types.h>
#include<netdb.h>
#include<arpa/inet.h>
#include "YeevidiaTasker.h"

class Server: public YeevidiaTasker {
public:
	Server();
	virtual ~Server();
	int accept_connections();
	int connectionStatus = -1;
	double get_angle(int cameraNumber);
	void Run();
	void Reset(void);
	bool isDone();
	bool Go();
private:
	void * get_in_addr(struct sockaddr * sa);
	void init();
	int status;
	struct addrinfo hints, * res;
	int listner;
	int new_conn_fd;
	struct sockaddr_storage client_addr;
	socklen_t addr_size;
	struct sockaddr_in  serverAddr;    /* server's socket address */
	    struct sockaddr_in  clientAddr;    /* client's socket address */
	    int                 sockAddrSize;  /* size of socket address structure */
	    int                 sFd;           /* socket file descriptor */
	    int                 newFd;         /* socket descriptor from accept */
	char s[INET6_ADDRSTRLEN];
};

#endif /* SRC_CLASSES_SERVER_H_ */
