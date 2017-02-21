#ifndef AutoGearOnPeg_H
#define AutoGearOnPeg_H

#include "../CommandBase.h"

class AutoGearOnPeg : public CommandBase {
private:
	int gearLidarUpperByte = 0;
	int gearLidarLowerByte = 0;
	int gearLidarValueInCentimeters = 0;

	bool startMeasuringLidarValue = true;
	bool lidarConfigureTimePassed = false;
	bool startMovingTowardsGear = false;
	bool startPuttingGearOnPeg = false;
	bool gearOnPeg = false;

	static constexpr double DISTANCE_AWAY_FROM_PEG_TO_DROP_GEAR_IN = 10.0;
	static constexpr double DISTANCE_AWAY_FROM_PEG_TO_DROP_GEAR_CM = (DISTANCE_AWAY_FROM_PEG_TO_DROP_GEAR_IN * 2.54);

	static constexpr double DRIVE_TRAIN_MOTOR_POWER = .35;

	static constexpr double GEAR_HOLDER_MOTOR_POWER = .5;

	bool lowerLimitSwitchValue = false;
public:
	AutoGearOnPeg();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // AutoGearOnPeg_H
