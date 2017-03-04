/*
 * Server.cpp
 *
 *  Created on: Jan 23, 2017
 *      Author: robot
 */

#include <Server.h>
#include <iostream>
#include <cstring>
#include <string>

Server::Server() {
		std::cout << "constructor" << "\n\n\n";
		memset(& hints, 0, sizeof hints);
		hints.ai_family = AF_UNSPEC;
		hints.ai_socktype = SOCK_STREAM;
		hints.ai_flags = AI_PASSIVE;
		status = getaddrinfo(NULL, "7821" , &hints, &res);
		if(status != 0)
		{
			fprintf(stderr,"getaddrinfo error: %s\n",gai_strerror(status));
		}
		listner = socket(res->ai_family,res->ai_socktype, res->ai_protocol);
		if(listner < 0 )
		{
			fprintf(stderr,"socket error: %s\n",gai_strerror(status));
		}
		status = bind(listner, res->ai_addr, res->ai_addrlen);
		if(status < 0)
		{
			fprintf(stderr,"bind: %s\n",gai_strerror(status));
		}

		status = listen(listner, 1);

		if(status < 0)
		{
			fprintf(stderr,"listen: %s\n",gai_strerror(status));
		}

		freeaddrinfo(res);

		addr_size = sizeof client_addr;
		std::cout << "finished constructing" << std::endl;
		new_conn_fd = -1;
		Server::Start();
}

Server::~Server() {
	// TODO Auto-generated destructor stub
}

void * Server::get_in_addr(struct sockaddr * sa)
{
	if(sa->sa_family == AF_INET)
	{
		return &(((struct sockaddr_in *)sa)->sin_addr);
	}

	return &(((struct sockaddr_in6 *)sa)->sin6_addr);
}

void Server::Run()
{
		std::cout << "accepting";
		if(new_conn_fd < 0)
		{
			new_conn_fd = accept(listner, (struct sockaddr *) & client_addr, &addr_size);
			std::cout << "\n new conn fd: " << new_conn_fd << std::endl;
			if(new_conn_fd < 0)
			{
				fprintf(stderr,"accept: %s\n",gai_strerror(new_conn_fd));
				return;
			}
			inet_ntop(client_addr.ss_family, get_in_addr((struct sockaddr *) &client_addr),s ,sizeof s);
		}
		else{
			std::cout << get_angle(0);

		}

}
int Server::accept_connections()
{
	try{
		new_conn_fd = accept(listner, (struct sockaddr *) & client_addr, &addr_size);

		if(new_conn_fd < 0)
		{
			fprintf(stderr,"accept: %s\n",gai_strerror(new_conn_fd));
			return -1;
		}
		inet_ntop(client_addr.ss_family, get_in_addr((struct sockaddr *) &client_addr),s ,sizeof s);
		std::cout << "connected to: " << s << "\n";
	}catch(const std::exception& ex)
	{
		return -1;
	}
	return 0;
}


double Server::get_angle(int cameraNumber)
{
	int numbytes = 0;
	char buf[20];
	std::string str = std::to_string(cameraNumber);
	std::strcpy( buf, str.c_str());
	double angle =0;
	send(new_conn_fd, buf, 20, 0);
	numbytes = recv(new_conn_fd, buf, 20, 0);
	if(numbytes < 1)
		new_conn_fd = -1;
	std::cout << "received: " << buf << "\n";
	char * st;
	st = strtok(buf, ",");
	if(cameraNumber == 0) angle = atof(st);
	st = strtok(NULL, ",");
	if(cameraNumber == 1)
		angle = atof(st);
	return angle;
	return atof(buf);
}