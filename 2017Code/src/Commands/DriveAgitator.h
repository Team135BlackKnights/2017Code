#ifndef DriveAgitator_H
#define DriveAgitator_H

#include "../CommandBase.h"

class DriveAgitator : public CommandBase {
private:
	//double agitatorMotorPower = 0.0;
	//double liftHangMotorPower = 0.0;
	static constexpr double AGITATOR_MOTOR_POWER = 1.0;
	static constexpr double AGITATOR_2_MOTOR_POWER = 1.0;
	bool driveForwards;
public:
	DriveAgitator(bool);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveAgitator_H
