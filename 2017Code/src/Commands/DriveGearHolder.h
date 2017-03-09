#ifndef DriveGearHolder_H
#define DriveGearHolder_H

#include "../CommandBase.h"

class DriveGearHolder : public CommandBase {
private:
	static constexpr double GEAR_HOLDER_MOTOR_POWER = .5;

	double gearHolderServoValue = 0.0;

	bool gearHolderUpwardsButtonPressed = false;
	bool gearHolderDownwardsButtonPressed = false;
public:
	DriveGearHolder();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveGearHolder_H
