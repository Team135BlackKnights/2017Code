#ifndef AutoGearOnPeg_H
#define AutoGearOnPeg_H

#include "../CommandBase.h"
#include <Timer.h>

class AutoGearOnPeg : public CommandBase {
private:
	double ultrasonicValue = 0.0;

	bool startMovingTowardsGear = false;
	bool startPuttingGearOnPeg = false;
	bool gearHolderDown = false;
	bool gearOnPeg = false;

	static constexpr double DISTANCE_AWAY_FROM_AIRSHIP_TO_DROP_GEAR_IN = 8.5;

	static constexpr double DRIVE_TRAIN_MOTOR_POWER = .45;

	double gearHolderMotorPower = 0.0;
	static constexpr double GEAR_HOLDER_MOTOR_POWER = .6;

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
	bool initializeTimerLimitSwitch = false;
	static constexpr double WAIT_TIME_FOR_LIMIT_SWITCH_TO_LOWER = 1.0;

	static constexpr double TIME_TO_LOWER_GEAR_HOLDER = .2;
	bool startMovingRobot = true;

	static constexpr double TIME_ROBOT_IS_ABOVE_DRIVE_TRAIN_RPM_THRESHOLD = .25;
	bool startTimerForDriveTrainRPMThreshold = false;
	bool waitingForDriveTrainRPMConfigure = false;

	frc::Timer* timer;
	double timerValue = 0.0;

	bool initializeTimerForRamIntoAirship = false;
	bool initializeTimerToUnRamFromAirship = false;
	static constexpr double TIME_TO_RAM_INTO_AIRSHIP = .35;
	static constexpr double TIME_TO_WAIT_BEFORE_UNRAMMING = .1;
	static constexpr double TIME_TO_REVERSE_FROM_RAMMING_INTO_AIRSHIP = .35;
	static constexpr double RAMMING_MOTOR_POWER = -.55;
	static constexpr double UNRAMMING_MOTOR_POWER = .65;
	bool rammedIntoAirship = false;
	bool initializeTimeWaitBeforeUnRamming = false;
	bool waitTimeForUnRammming = false;

	int moveGearHolderUpDownCounter = 1;

	static const bool RAM_INTO_GEAR_PEG = true;
public:
	AutoGearOnPeg();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // AutoGearOnPeg_H
