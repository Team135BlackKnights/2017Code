#ifndef AutoDriveShooter_H
#define AutoDriveShooter_H

#include "../CommandBase.h"
#include <Timer.h>

class AutoDriveShooter : public CommandBase {
private:
	int setpointRPM;
	int currentShooterWheelRPM = 0;
	bool shooterVoltageMode = false;
	bool shooterPIDMode = false;

	bool beginTimerToLevelOutSetpoint = false;
	bool moveAgitatorToShootFuel = false;

	frc::Timer* timer;
	double timerValue = 0.0;
	static constexpr double WAIT_TIME_FOR_SHOOTER = .05;

	static constexpr double WAIT_TIME_FOR_FUEL_TO_SHOOT_WITHOUT_COLLECTION = 6.0;
	static constexpr double WAIT_TIME_FOR_FUEL_TO_SHOOT_WITH_COLLECTION = 6.0;

	static constexpr double TIME_TO_WAIT_UNTIL_RUNNING_COLLECTION_AFTER_AGITATOR_STARTS = 2.75;

	static constexpr double TIME_TO_DRIVE_COLLECTION = 1.0;

	static constexpr double AUTO_DRIVE_SHOOTER_TIMEOUT = 6.0;

	static constexpr double AGITATOR_MOTOR_POWER = .3;

	bool startTimerForAgitator = false;
	bool shotFuelIntoBoiler = false;

	static constexpr double COLLECTION_MOTOR_POWER = 1.0;
	bool startRunningCollection = false;

	bool initializeTimerToRunCollection = false;
	double initialTimeForRunningCollection = 0.0;
	double differenceBetweenCurrentTimeAndInitialTimeForRunningCollection = 0.0;

	bool doneDrivingCollection = false;
public:
	AutoDriveShooter(int);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // AutoDriveShooter_H
