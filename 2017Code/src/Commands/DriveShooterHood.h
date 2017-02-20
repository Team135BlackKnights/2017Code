#ifndef DriveShooterHood_H
#define DriveShooterHood_H

#include "../CommandBase.h"

class DriveShooterHood : public CommandBase {
private:
	bool driveUpwards = false;
	static constexpr double SHOOTER_HOOD_MOTOR_POWER = .8;
public:
	DriveShooterHood(bool);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveShooterHood_H
