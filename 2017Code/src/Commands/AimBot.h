#ifndef AimBot_H
#define AimBot_H

#include "../CommandBase.h"
#include "../Subsystems/DriveTrain.h"
#include "../Server.h"
#include "../OI.h"
#include <cmath>

class AimBot : public CommandBase {
public:
	AimBot(int camNumber);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	int cameraNumber = 0;
	bool isbad = false;
	Timer time;
	double leftUltrasonicSensorValue = 0.0;
	double rightUltrasonicSensorValue = 0.0;
	const double CAMERA_TO_GEAR_IN = 10;
	const double SPRING_IN = 14.5;
};

#endif  // AimBot_H
