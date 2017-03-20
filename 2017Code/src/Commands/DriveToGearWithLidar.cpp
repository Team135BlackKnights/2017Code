#include "DriveToGearWithLidar.h"

DriveToGearWithLidar::DriveToGearWithLidar(double desiredDistanceToTravel, double driveTrainMotorPower) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get());
	Requires(CommandBase::lidar.get());

	this->desiredDistanceToTravel = desiredDistanceToTravel;
	this->driveTrainMotorPower = driveTrainMotorPower;
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

	CommandBase::lidar->SendArduinoDigitalSignal(false);
	initializeLidarSendLowSignal = true;
	startUsingLidar = false;
	lidarAtGearPeg = false;
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

	if (initializeLidarSendLowSignal == false) {
		CommandBase::lidar->SendArduinoDigitalSignal(false);
		initializeLidarSendLowSignal = true;
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

	if (differenceBetweenCurrentAndInitialDistanceTraveled >= DISTANCE_BEFORE_STARTING_USING_LIDAR && startUsingLidar == false) {
		startUsingLidar = true;
	}

	if (startUsingLidar) {
		CommandBase::lidar->SendArduinoDigitalSignal(true);
		lidarAtGearPeg = CommandBase::lidar->GetArduinoAnalogSignal();
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveToGearWithLidar::IsFinished() {
	return (distanceTraveledWithEncoder || lidarAtGearPeg);
}

// Called once after isFinished returns true
void DriveToGearWithLidar::End() {
	CommandBase::driveTrain->DriveTank(0.0, 0.0);
	requiresDriveTrainSubsystem = false;
	initialDistanceConfigured = false;
	CommandBase::driveTrain->is_aiming = false;
	zeroGyro = false;
	distanceTraveledWithEncoder = false;

	CommandBase::lidar->SendArduinoDigitalSignal(false);
	initializeLidarSendLowSignal = false;
	startUsingLidar = false;
	lidarAtGearPeg = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveToGearWithLidar::Interrupted() {
	End();
}
