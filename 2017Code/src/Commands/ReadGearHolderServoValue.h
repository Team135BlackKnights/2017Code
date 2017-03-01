#ifndef ReadGearHolderServoValue_H
#define ReadGearHolderServoValue_H

#include "../CommandBase.h"

class ReadGearHolderServoValue : public CommandBase {
private:
	double servoPosition = 0.0;
public:
	ReadGearHolderServoValue();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // ReadGearHolderServoValue_H
