#ifndef AimBotWithUltrasonicSensors_H
#define AimBotWithUltrasonicSensors_H

#include "../CommandBase.h"
#include <Timer.h>

class AimBotWithUltrasonicSensors : public CommandBase {
private:
	double leftUltrasonicSensorValue = 0.0;
	double rightUltrasonicSensorValue = 0.0;

	double desiredAngleToTurnDriveTrain = 0.0;
	bool *sonBool;
	frc::Timer* timer;
	double timerValue = 0.0;
	bool initializeAimBotWithUltrasonicSensors = false;
	static constexpr double MAX_TIME_FOR_EXECUTING_DRIVE_TRAIN_PID = .5;
public:
	AimBotWithUltrasonicSensors(bool* sonarGood);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // AimBotWithUltrasonicSensors_H
