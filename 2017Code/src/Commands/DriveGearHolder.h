#ifndef DriveGearHolder_H
#define DriveGearHolder_H

#include "../CommandBase.h"

class DriveGearHolder : public CommandBase {
private:
	bool driveUpwards;
	static constexpr double GEAR_HOLDER_MOTOR_POWER = .5;
public:
	DriveGearHolder(bool);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveGearHolder_H
