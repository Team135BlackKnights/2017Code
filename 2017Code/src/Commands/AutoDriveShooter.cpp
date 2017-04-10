#include "AutoDriveShooter.h"

AutoDriveShooter::AutoDriveShooter(int setpointRPM) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::shooter.get());
	Requires(CommandBase::agitator.get());
	Requires(CommandBase::collection.get());
	this->setpointRPM = setpointRPM;
	timer = new frc::Timer();
}

// Called just before this Command runs the first time
void AutoDriveShooter::Initialize() {
	SetTimeout(AUTO_DRIVE_SHOOTER_TIMEOUT);

	CommandBase::shooter->ConfigureShooterPID();
	shooterPIDMode = true;
	shooterVoltageMode = false;

	beginTimerToLevelOutSetpoint = false;
	moveAgitatorToShootFuel = false;
	startTimerForAgitator = false;
	shotFuelIntoBoiler = false;

	timer->Reset();

	initializeTimerToRunCollection = false;
	startRunningCollection = false;

	doneDrivingCollection = false;
}

// Called repeatedly when this Command is scheduled to run
void AutoDriveShooter::Execute() {
	timerValue = timer->Get();

	if (shooterPIDMode == false) {
		CommandBase::shooter->ConfigureShooterPID();
		shooterPIDMode = true;
		shooterVoltageMode = false;
	}

	currentShooterWheelRPM = CommandBase::shooter->GetShooterWheelRPM();
	CommandBase::shooter->DriveShooterMotor(this->setpointRPM);

	if (currentShooterWheelRPM >= this->setpointRPM && beginTimerToLevelOutSetpoint == false) {
		beginTimerToLevelOutSetpoint = true;
		timer->Reset();
		timer->Start();
	}
	else if (beginTimerToLevelOutSetpoint && timerValue >= WAIT_TIME_FOR_SHOOTER && moveAgitatorToShootFuel == false) {
		moveAgitatorToShootFuel = true;
		timer->Stop();
		timer->Reset();
	}

	if (moveAgitatorToShootFuel) {
		if (startTimerForAgitator == false) {
			CommandBase::agitator->DriveAgitator(AGITATOR_MOTOR_POWER);
			timer->Reset();
			timer->Start();
			startTimerForAgitator = true;
		}
		else if (startTimerForAgitator && timerValue < WAIT_TIME_FOR_FUEL_TO_SHOOT_WITH_COLLECTION) {
			CommandBase::agitator->DriveAgitator(AGITATOR_MOTOR_POWER);
		}
		else if (startTimerForAgitator && timerValue >= WAIT_TIME_FOR_FUEL_TO_SHOOT_WITH_COLLECTION) {
			CommandBase::agitator->DriveAgitator(0.0);
			timer->Stop();
			timer->Reset();
			if (shooterVoltageMode == false) {
				CommandBase::shooter->ConfigureShooterVoltageMode();
				shooterPIDMode = false;
				shooterVoltageMode = true;
			}
			CommandBase::shooter->DriveShooterMotor(0.0);
			shotFuelIntoBoiler = true;
		}
	}

	if (timerValue >= TIME_TO_WAIT_UNTIL_RUNNING_COLLECTION_AFTER_AGITATOR_STARTS && moveAgitatorToShootFuel && startRunningCollection == false && doneDrivingCollection == false) {
		startRunningCollection = true;
	}

	if (startRunningCollection && doneDrivingCollection == false) {
		timerValue = timer->Get();
		if (initializeTimerToRunCollection == false) {
			initialTimeForRunningCollection = timerValue;
			initializeTimerToRunCollection = true;
		}
		differenceBetweenCurrentTimeAndInitialTimeForRunningCollection = (timerValue - initialTimeForRunningCollection);

		if (differenceBetweenCurrentTimeAndInitialTimeForRunningCollection < TIME_TO_DRIVE_COLLECTION) {
			CommandBase::collection->DriveCollection(COLLECTION_MOTOR_POWER);
		}
		else if (differenceBetweenCurrentTimeAndInitialTimeForRunningCollection >= TIME_TO_DRIVE_COLLECTION) {
			CommandBase::collection->DriveCollection(0.0);
			doneDrivingCollection = true;
		}
	}
}

// Make this return true when this Command no longer needs to run execute()
bool AutoDriveShooter::IsFinished() {
	return shotFuelIntoBoiler || IsTimedOut();
}

// Called once after isFinished returns true
void AutoDriveShooter::End() {
	CommandBase::shooter->ConfigureShooterVoltageMode();
	shooterPIDMode = false;
	shooterVoltageMode = true;
	CommandBase::shooter->DriveShooterMotor(0.0);
	CommandBase::agitator->DriveAgitator(0.0);
	CommandBase::collection->DriveCollection(0.0);

	beginTimerToLevelOutSetpoint = false;
	moveAgitatorToShootFuel = false;
	startTimerForAgitator = false;
	shotFuelIntoBoiler = false;

	startRunningCollection = false;
	initializeTimerToRunCollection = false;
	doneDrivingCollection = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutoDriveShooter::Interrupted() {
	End();
}
