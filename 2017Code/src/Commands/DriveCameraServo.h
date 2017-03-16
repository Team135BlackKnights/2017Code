#ifndef DriveCameraServo_H
#define DriveCameraServo_H

#include "../CommandBase.h"

class DriveCameraServo : public CommandBase {
public:
	DriveCameraServo();
	void Initialize();
	void Execute();
	bool IsFinished(); //std::hi conner you suck
	void End();
	void Interrupted();
};

#endif  // DriveCameraServo_H
