#ifndef AutoDriveShooterHood_H
#define AutoDriveShooterHood_H

#include "../CommandBase.h"

class AutoDriveShooterHood : public CommandBase {
private:
	int desiredShooterHoodEncoderValue;
	bool shooterHoodAtDesiredEncoderPosition = false;
public:
	AutoDriveShooterHood(int);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // AutoDriveShooterHood_H
