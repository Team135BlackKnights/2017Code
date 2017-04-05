#include "DriveBackwardsWithLidar.h"
#include <Timer.h>

DriveBackwardsWithLidar::DriveBackwardsWithLidar(double initialDriveTrainMotorPower, double desiredLidarValueToDriveUntil) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get());
	Requires(CommandBase::lidars.get());

	this->initialDriveTrainMotorPower = initialDriveTrainMotorPower;
	this->desiredLidarValueToDriveUntil = (desiredLidarValueToDriveUntil + DISTANCE_BETWEEN_LIDAR_AND_EDGE_OF_BUMPER);
}

// Called just before this Command runs the first time
void DriveBackwardsWithLidar::Initialize() {
	CommandBase::lidars->OpenLidarChannelOnMultiplexer(Lidars::VALUE_TO_OPEN_FRONT_LIDAR_CHANNEL_7);
	initializeI2CMultiplexerChannel = true;
	readLidarValueForFirstTime = false;
	configureLidar = false;

	startUsingLidarValues = false;

	driveAtInitialDriveTrainMotorPower = true;
	driveAtSlowerDriveTrainMotorPowerPart1 = false;
	driveAtSlowerDriveTrainMotorPowerPart2 = false;
	robotAtDesiredLidarValue = false;

	CommandBase::driveTrain->ZeroGyroAngle();
	zeroGyroAngle = true;
}

// Called repeatedly when this Command is scheduled to run
void DriveBackwardsWithLidar::Execute() {
	if (initializeI2CMultiplexerChannel == false) {
		CommandBase::lidars->OpenLidarChannelOnMultiplexer(Lidars::VALUE_TO_OPEN_FRONT_LIDAR_CHANNEL_7);
		initializeI2CMultiplexerChannel = true;
	}

	if (zeroGyroAngle == false) {
		CommandBase::driveTrain->ZeroGyroAngle();
		zeroGyroAngle = true;
	}

	currentGyroAngle = CommandBase::driveTrain->GetGyroAngle();

	if (configureLidar == false) {
		CommandBase::lidars->ConfigureLidar();
		readLidarValueForFirstTime = true;
		configureLidar = true;
	}
	else if (configureLidar) {
		lidarUpperByte = CommandBase::lidars->GetUpperByte();
		frc::Wait(.002);
		lidarLowerByte = CommandBase::lidars->GetLowerByte();
		currentlidarValueIN = CommandBase::lidars->GetLidarValue(lidarLowerByte, lidarUpperByte, Lidars::DISTANCE_UNIT_ARRAY[Lidars::INCHES]);
		configureLidar = false;
	}

	/*if (configureLidar == false) {
		frc::Wait(.002);
		CommandBase::lidars->ConfigureLidar();
		configureLidar = true;
	} */

	if (currentlidarValueIN != 0.0 && startUsingLidarValues == false) {
		startUsingLidarValues = true;
	}

	if (startUsingLidarValues) {
		differenceBetweenDesiredAndCurrentLidarValue = (this->desiredLidarValueToDriveUntil - currentlidarValueIN);
		if (differenceBetweenDesiredAndCurrentLidarValue <= ESTIMATED_COAST_DISTANCE_IN) {
			CommandBase::driveTrain->DriveTank(0.0, 0.0);
			robotAtDesiredLidarValue = true;
		}
		else if (differenceBetweenDesiredAndCurrentLidarValue < DIFFERENCE_BETWEEN_DESIRED_AND_CURRENT_LIDAR_VALUES_TO_DRIVE_SLOWER_PART_2) {
			driveAtSlowerDriveTrainMotorPowerPart2 = true;
			driveAtInitialDriveTrainMotorPower = false;
			driveAtSlowerDriveTrainMotorPowerPart1 = false;
			robotAtDesiredLidarValue = false;
		}
		else if (differenceBetweenDesiredAndCurrentLidarValue < DIFFERENCE_BETWEEN_DESIRED_AND_CURRENT_LIDAR_VALUES_TO_DRIVE_SLOWER_PART_1) {
			driveAtSlowerDriveTrainMotorPowerPart1 = true;
			driveAtInitialDriveTrainMotorPower = false;
			driveAtSlowerDriveTrainMotorPowerPart2 = false;
			robotAtDesiredLidarValue = false;
		}
		else {
			driveAtInitialDriveTrainMotorPower = true;
			driveAtSlowerDriveTrainMotorPowerPart1 = false;
			driveAtSlowerDriveTrainMotorPowerPart2 = false;
			robotAtDesiredLidarValue = false;
		}
	}

	if (driveAtInitialDriveTrainMotorPower) {
		CommandBase::driveTrain->DriveStraightWithGyro(this->initialDriveTrainMotorPower, currentGyroAngle);
	}
	else if (driveAtSlowerDriveTrainMotorPowerPart1) {
		CommandBase::driveTrain->DriveStraightWithGyro(SLOWER_DRIVE_TRAIN_MOTOR_POWER_PART_1, currentGyroAngle);
	}
	else if (driveAtSlowerDriveTrainMotorPowerPart2) {
		CommandBase::driveTrain->DriveStraightWithGyro(SLOWER_DRIVE_TRAIN_MOTOR_POWER_PART_2, currentGyroAngle);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveBackwardsWithLidar::IsFinished() {
	return robotAtDesiredLidarValue;
}

// Called once after isFinished returns true
void DriveBackwardsWithLidar::End() {
	CommandBase::driveTrain->DriveTank(0.0, 0.0);

	initializeI2CMultiplexerChannel = false;
	readLidarValueForFirstTime = false;
	configureLidar = false;

	startUsingLidarValues = false;

	driveAtInitialDriveTrainMotorPower = true;
	driveAtSlowerDriveTrainMotorPowerPart1 = false;
	driveAtSlowerDriveTrainMotorPowerPart2 = false;
	robotAtDesiredLidarValue = false;

	zeroGyroAngle = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveBackwardsWithLidar::Interrupted() {
	End();
}
