#ifndef AutoGearOnPeg_H
#define AutoGearOnPeg_H

#include "../CommandBase.h"

class AutoGearOnPeg : public CommandBase {
private:
	double ultrasonicValue = 0.0;

	bool startMeasuringUltrasonicValue = true;
	bool startMovingTowardsGear = false;
	bool startPuttingGearOnPeg = false;
	bool gearOnPeg = false;

	static constexpr double DISTANCE_AWAY_FROM_AIRSHIP_TO_DROP_GEAR_IN = 8.25;

	static constexpr double DRIVE_TRAIN_MOTOR_POWER = .4;

	static constexpr double GEAR_HOLDER_MOTOR_POWER = .5;

	bool lowerLimitSwitchValue = false;

	int rightDriveTrainEncoderRPM = 0;

	static const int DRIVE_TRAIN_ENCODER_RPM_THRESHOLD = 6.0;

	double initialRightEncoderDistance = 0.0;
	double currentRightEncoderDistance = 0.0;
	double desiredRightEncoderDistance = 0.0;
	static constexpr double DISTANCE_TO_MOVE_AWAY_FROM_GEAR = 3.0;  //  In Inches
	bool retryGearLineUp = false;
	bool moveGearHolderDown = false;
	bool stopGearHolderDown = false;

	static constexpr double TIME_TO_LOWER_GEAR_HOLDER = .35;
	bool startMovingRobot = true;

	frc::Timer* timer;
	double timerValue = 0.0;
public:
	AutoGearOnPeg();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // AutoGearOnPeg_H
