#ifndef DriveUntilLidarIsCertainValue_H
#define DriveUntilLidarIsCertainValue_H

#include "../CommandBase.h"

class DriveUntilLidarIsCertainValue : public CommandBase {
private:
	double desiredLidarValueToDriveUntil;
	double motorPower;

	bool receivedFirstLidarValue = false;
	double currentLidarValueInches = 0.0;

	bool drivenUntilDesiredLidarValue = false;
public:
	DriveUntilLidarIsCertainValue(double, double);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveUntilLidarIsCertainValue_H
