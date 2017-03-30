#ifndef SwitchDriveTrainMotorPower_H
#define SwitchDriveTrainMotorPower_H

#include "../CommandBase.h"

class SwitchDriveTrainMotorPower : public CommandBase {
private:
	bool setDriveTrainMotorPower = false;
	bool fastDriveTrainMotorPower = false;
public:
	SwitchDriveTrainMotorPower();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // SwitchDriveTrainMotorPower_H
