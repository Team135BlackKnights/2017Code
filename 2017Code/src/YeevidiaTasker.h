/*
 * YeevidiaTasker.h
 *
 *  Created on: Jan 25, 2017
 *      Author: robot
 */

#ifndef SRC_CLASSES_YEEVIDIATASKER_H_
#define SRC_CLASSES_YEEVIDIATASKER_H_
#include "WPILib.h"
#include <pthread.h>
class YeevidiaTasker {
public:
	YeevidiaTasker(const char* taskName = NULL);
	static void* YeevidiaStarterTask(void* task);
	void Start();
	void Pause();
	void Terminate();
	virtual void Run() = 0;
	virtual ~YeevidiaTasker();

private:
	bool enabled_;
	bool running_;
	bool isDead_;
	pthread_t ptask;
	// nice job sam very nice
};

#endif /* SRC_CLASSES_YEEVIDIATASKER_H_ */
