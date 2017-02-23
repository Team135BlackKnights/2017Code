#ifndef AimBot_H
#define AimBot_H

#include "../CommandBase.h"
#include "../Subsystems/DriveTrain.h"
#include "../Subsystems/Server.h"
#include "../OI.h"

class AimBot : public CommandBase {
public:
	AimBot(int camNumber);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	int cameraNumber = 0;
	Timer time;
};

#endif  // AimBot_H
