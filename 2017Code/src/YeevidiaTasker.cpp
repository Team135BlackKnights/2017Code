/*
 * YeevidiaTasker.cpp
 *
 *  Created on: Jan 25, 2017
 *      Author: robot
 */

#include <YeevidiaTasker.h>

YeevidiaTasker::YeevidiaTasker(const char* taskName) {
	// TODO Auto-generated constructor stub

	enabled_ = false;
	running_ = true;
	isDead_ = false;

	pthread_create(&ptask, NULL, &YeevidiaTasker::YeevidiaStarterTask, (void*)this);

}

YeevidiaTasker::~YeevidiaTasker() {
	void* res;
	this->Terminate();
	pthread_join(ptask, &res);
	free(res);
}


void* YeevidiaTasker::YeevidiaStarterTask(void* ntask)
{
	YeevidiaTasker* task = (YeevidiaTasker*)ntask;
	while(task->running_)
	{
		if(task->enabled_)
		{
			//std::cout<<"enabled" << std::endl;
			task->Run();
			frc::Wait(.05);
		}
		else
			frc::Wait(.05);
	}
	std::cout << "dead";
	task->isDead_ = true;
	return NULL;
}


void YeevidiaTasker::Start()
{
	std::cout <<"started";
	enabled_ = true;
}

void YeevidiaTasker::Pause()
{
	enabled_ = false;
}

void YeevidiaTasker::Terminate()
{
	running_ = false;

	while(!isDead_)
	{
		frc::Wait(.02);
	}
}
