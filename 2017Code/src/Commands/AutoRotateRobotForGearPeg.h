#ifndef AutoRotateRobotForGearPeg_H
#define AutoRotateRobotForGearPeg_H

#include "../CommandBase.h"
#include <Timer.h>

class AutoRotateRobotForGearPeg : public CommandBase {
private:
	double motorPower;
	double desiredAngleToRotateRobot;

	static const bool TURN_RIGHT = false;
	static const bool TURN_LEFT = !TURN_RIGHT;

	bool zeroedGyro = false;

	frc::Timer* timer;
	double timerValue = 0.0;
	static constexpr double FIRST_ROTATION_TIMEOUT = .35;
	static constexpr double TIME_TO_WAIT_BETWEEN_FIRST_AND_SECOND_ROTATIONS = .05;
	static constexpr double SECOND_ROTATION_TIMEOUT = .7;
	static constexpr double TIME_TO_WAIT_BETWEEN_SECOND_AND_THIRD_ROTATIONS = .05;
	static constexpr double THIRD_ROTATION_TIMEOUT = .35;

	bool initializeFirstRotationTimer = false;
	bool initializeFirstTimeWait = false;
	bool initializeSecondRotationTimer = false;
	bool initializeSecondTimeWait = false;
	bool initializeThirdRotationTimer = false;

	bool firstRotationComplete = false;
	bool waitBetweenFirstAndSecondRotations = false;
	bool secondRotationComplete = false;
	bool waitBetweenSecondAndThirdRotations = false;
	bool thirdRotationComplete = false;

	double angleTurnedDuringFirstRotation = 0.0;
	double angleToTurnForSecondRotation = 0.0;
	double angleToTurnForThirdRotation = 0.0;

	bool initializedNewAutoRotateRobotFirstRotation = false;
	bool initializedNewAutoRotateRobotSecondRotation = false;
	bool initializedNewAutoRotateRobotThirdRotation = false;
public:
	AutoRotateRobotForGearPeg(double, double);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // AutoRotateRobotForGearPeg_H
