#ifndef AutoDriveAgitator_H
#define AutoDriveAgitator_H

#include "../CommandBase.h"
#include <Timer.h>

class AutoDriveAgitator : public CommandBase {
private:
	bool stopCollection;
	double timeToDriveAgitator;
	double timeToStartToDriveCollection;

	bool shooterUpToSpeed = false;
	bool turnOffCollection = false;

	static constexpr double AGITATOR_MOTOR_POWER = .35;

	frc::Timer* timer;
	double timerValue = 0.0;
	static constexpr double TIME_TO_WAIT_FOR_FUEL_TO_SHOOT = 8.5;

	bool startTimer = false;
	bool doneDrivingAgitator = false;

	static constexpr double AUTO_DRIVE_AGITATOR_TIMEOUT = 9.0;

	bool startDrivingCollection = false;
	static constexpr double TIME_TO_WAIT_BEFORE_RUNNING_COLLECTION = 4.5;
	static constexpr double TIME_TO_RUN_COLLECTION = 1.0;
	bool initializeTimerForCollection = false;
	double initialTimeForRunningCollection = 0.0;
	double differenceBetweenCurrentAndInitialTimeForRunningCollection = 0.0;
	bool doneDrivingCollection = false;

	static constexpr double COLLECTION_MOTOR_POWER = 1.0;

	bool initializeContinueDrivingShooter = false;
public:
	AutoDriveAgitator(bool, double, double);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // AutoDriveAgitator_H
