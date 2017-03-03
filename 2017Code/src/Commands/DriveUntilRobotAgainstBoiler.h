#ifndef DriveUntilRobotAgainstBoiler_H
#define DriveUntilRobotAgainstBoiler_H

#include "../CommandBase.h"

class DriveUntilRobotAgainstBoiler : public CommandBase {
private:
	static constexpr double DRIVE_TRAIN_MOTOR_POWER = .4;

	int lidarValue_CM = 0.0;
	double lidarValue_IN = 0.0;

	static constexpr double LIDAR_DISTANCE_TO_TRAVEL_UNTIL = 6.0;

	bool lidarResetVaribales = false;
	bool receivedFirstLidarValue = false;
	bool againstBoiler = false;
public:
	DriveUntilRobotAgainstBoiler();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveUntilRobotAgainstBoiler_H
