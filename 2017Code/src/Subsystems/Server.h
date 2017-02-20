#ifndef Server_H
#define Server_H

#include <Commands/Subsystem.h>
#include<stdio.h>
#include<string.h>
#include<unistd.h>
#include<sys/socket.h>
#include<sys/types.h>
#include<netdb.h>
#include<arpa/inet.h>

class Server : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
	void * get_in_addr(struct sockaddr * sa);
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
public:
	Server();
	void InitDefaultCommand();
	int accept_connections();
		int connectionStatus = -1;
		int get_angle(int cameraNumber);
		void Run();
		void Reset(void);
		bool isDone();
		bool Go();
};

#endif  // Server_H
