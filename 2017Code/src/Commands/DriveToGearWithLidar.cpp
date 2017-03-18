#include "DriveToGearWithLidar.h"

DriveToGearWithLidar::DriveToGearWithLidar(double desiredDistanceToTravel, double driveTrainMotorPower) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get());
	Requires(CommandBase::pwmLidar.get());

	this->desiredDistanceToTravel = desiredDistanceToTravel;
	this->driveTrainMotorPower = driveTrainMotorPower;

	timer = new frc::Timer();
}

// Called just before this Command runs the first time
void DriveToGearWithLidar::Initialize() {
	initialDistanceTraveled = CommandBase::driveTrain->GetDistance(DriveTrain::RIGHT_SIDE_ENCODER);
	initialDistanceConfigured = true;
	distanceTraveledWithEncoder = false;
	CommandBase::driveTrain->ZeroGyroAngle();
	zeroGyro = true;
	CommandBase::driveTrain->is_aiming = true;
	requiresDriveTrainSubsystem = true;

	timer->Reset();
	timer->Start();

	CommandBase::pwmLidar->StartReceivingLidarValues();
	startReceivingLidarValues = true;
	startReceivingLidarValues = false;
	startGettingLidarValues = false;
	waitTimeForFirstRoundLidarValue = false;
	waitForLidarHighSignal = false;
	waitForSavingSecondRoundValue = false;
	waitForThirdRoundValue = false;
	lidarDetectedPeg = false;
}

// Called repeatedly when this Command is scheduled to run
void DriveToGearWithLidar::Execute() {
	if (requiresDriveTrainSubsystem == false) {
		CommandBase::driveTrain->is_aiming = true;
		requiresDriveTrainSubsystem = true;
	}

	if (initialDistanceConfigured == false) {
		initialDistanceTraveled = CommandBase::driveTrain->GetDistance(DriveTrain::RIGHT_SIDE_ENCODER);
		initialDistanceConfigured = true;
	}

	if (zeroGyro == false) {
		CommandBase::driveTrain->ZeroGyroAngle();
		zeroGyro = true;
	}

	gyroAngle = CommandBase::driveTrain->GetGyroAngle();

	currentDistanceTraveled = CommandBase::driveTrain->GetDistance(DriveTrain::RIGHT_SIDE_ENCODER);
	differenceBetweenCurrentAndInitialDistanceTraveled = fabs(currentDistanceTraveled - initialDistanceTraveled);

	if (differenceBetweenCurrentAndInitialDistanceTraveled >= this->desiredDistanceToTravel) {
		CommandBase::driveTrain->DriveTank(0.0, 0.0);
		distanceTraveledWithEncoder = true;
	}
	else {
		CommandBase::driveTrain->DriveStraightWithGyro(this->driveTrainMotorPower, gyroAngle);
		distanceTraveledWithEncoder = false;
	}

	if ((differenceBetweenCurrentAndInitialDistanceTraveled >= DISTANCE_TO_START_GETTING_LIDAR_VALUES) && startGettingLidarValues == false) {
		startGettingLidarValues = true;
	}

	if (startReceivingLidarValues == false) {
		CommandBase::pwmLidar->StartReceivingLidarValues();
		startReceivingLidarValues = true;
	}

	lidarValueIN = CommandBase::pwmLidar->GetLidarPWMValue(PWMLidars::INCHES);
	timerValue = timer->Get();

	if (startGettingLidarValues) {
		if (waitTimeForFirstRoundLidarValue == false) {
			timer->Reset();
			timer->Start();
			waitTimeForFirstRoundLidarValue = true;
		}
		else if (waitTimeForFirstRoundLidarValue && timerValue >= TIME_TO_WAIT_TO_GET_ACCURATE_FIRST_ROUND_LIDAR_VALUE) {
			savedFirstRoundLidarValue = lidarValueIN;
			timer->Stop();
			timer->Reset();
			waitForLidarHighSignal = true;
		}
		else if (waitTimeForFirstRoundLidarValue && (timerValue < TIME_TO_WAIT_TO_GET_ACCURATE_FIRST_ROUND_LIDAR_VALUE)) {
			waitForLidarHighSignal = false;
		}
	}
	else if (waitForLidarHighSignal) {
		if (lidarValueIN >= (savedFirstRoundLidarValue + ADDED_DISTANCE_OF_LIDAR_THROUGH_AIRSHIP)) {
			if (waitForSavingSecondRoundValue == false) {
				timer->Reset();
				timer->Start();
				waitForSavingSecondRoundValue = true;
			}
			else if (waitForSavingSecondRoundValue && (timerValue >= TIME_TO_WAIT_TO_GET_ACCURATE_LIDAR_VALUE_THROUGH_AIRSHIP)) {
				savedSecondRoundOfLidarValue = lidarValueIN;
				timer->Stop();
				timer->Reset();
				waitForThirdRoundValue = true;
			}
			else if (waitForSavingSecondRoundValue && (timerValue < TIME_TO_WAIT_TO_GET_ACCURATE_LIDAR_VALUE_THROUGH_AIRSHIP)) {
				waitForThirdRoundValue = false;
			}
		}
		else {
			timer->Stop();
			timer->Reset();
			waitForSavingSecondRoundValue = false;
			waitForThirdRoundValue = false;
		}
	}
	else if (waitForThirdRoundValue) {
		if ((lidarValueIN + DISTANCE_DROP_FROM_AIRSHIP_TO_GEAR_PEG) <= savedSecondRoundOfLidarValue) {
			thirdRoundLidarValueCounter++;
		}
		else {
			thirdRoundLidarValueCounter = 0;
		}

		if (thirdRoundLidarValueCounter >= THIRD_ROUND_COUNTER_MAX_VALUE) {
			CommandBase::driveTrain->DriveTank(0.0, 0.0);
			lidarDetectedPeg = true;
		}
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveToGearWithLidar::IsFinished() {
	return (distanceTraveledWithEncoder || lidarDetectedPeg);
}

// Called once after isFinished returns true
void DriveToGearWithLidar::End() {
	CommandBase::driveTrain->DriveTank(0.0, 0.0);
	requiresDriveTrainSubsystem = false;
	initialDistanceConfigured = false;
	CommandBase::driveTrain->is_aiming = false;
	zeroGyro = false;
	distanceTraveledWithEncoder = false;

	startReceivingLidarValues = false;
	startGettingLidarValues = false;
	waitTimeForFirstRoundLidarValue = false;
	waitForLidarHighSignal = false;
	waitForSavingSecondRoundValue = false;
	waitForThirdRoundValue = false;
	lidarDetectedPeg = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveToGearWithLidar::Interrupted() {
	End();
}
