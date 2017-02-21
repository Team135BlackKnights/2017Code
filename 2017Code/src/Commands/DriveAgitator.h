#ifndef DriveAgitator_H
#define DriveAgitator_H

#include "../CommandBase.h"

class DriveAgitator : public CommandBase {
private:
	double agitatorMotorSpeed = 0.0;
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