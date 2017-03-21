#include "DriveToGearWithLidar.h"

DriveToGearWithLidar::DriveToGearWithLidar(double desiredDistanceToTravel, double driveTrainMotorPower, bool rightGear) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get());
	Requires(CommandBase::lidars.get());

	this->desiredDistanceToTravel = desiredDistanceToTravel;
	this->driveTrainMotorPower = driveTrainMotorPower;
	this->rightGear = rightGear;
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

	initializeI2CMultiplxerChannelToOpen = false;
	configureLidar = false;
	startLidarDetectingGearPeg = false;
	waitingForGearPeg = false;
	lidarDetectsGearPeg = false;
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

	if (initializeI2CMultiplxerChannelToOpen == false) {
		if (this->rightGear == RIGHT_GEAR) {
			CommandBase::lidars->OpenLidarChannelOnMultiplexer(Lidars::VALUE_TO_OPEN_LIDAR_CHANNEL_6_LEFT_LIDAR);
		}
		else if (this->rightGear == LEFT_GEAR) {
			CommandBase::lidars->OpenLidarChannelOnMultiplexer(Lidars::VALUE_TO_OPEN_LIDAR_CHANNEL_7_RIGHT_LIDAR);
		}
		initializeI2CMultiplxerChannelToOpen = true;
	}

	if (configureLidar == false) {
		CommandBase::lidars->ConfigureLidar();
		configureLidar = true;
	}
	else if (configureLidar) {
		lidarUpperByte = CommandBase::lidars->GetUpperByte();
		lidarLowerByte = CommandBase::lidars->GetLowerByte();
		currentLidarValueIN = CommandBase::lidars->GetLidarValue(lidarLowerByte, lidarUpperByte, Lidars::DISTANCE_UNIT_ARRAY[Lidars::INCHES]);
		configureLidar = false;
	}

	if (differenceBetweenCurrentAndInitialDistanceTraveled >= DISTANCE_TO_TRAVEL_TO_START_DETECTING_AIRSHIP) {
		startLidarDetectingGearPeg = true;
	}

	if (startLidarDetectingGearPeg && waitingForGearPeg == false) {
		savedLidarTransparentAirshipValue = currentLidarValueIN;
		waitingForGearPeg = true;
	}
	else if (startLidarDetectingGearPeg && waitingForGearPeg) {
		if ((currentLidarValueIN + DISTANCE_DROP_OF_LIDAR_FROM_ARISHIP_TO_GEAR_PEG_IN) <= savedLidarTransparentAirshipValue) {
			lidarDetectsGearPeg = true;
		}
		else {
			lidarDetectsGearPeg = false;
		}
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveToGearWithLidar::IsFinished() {
	return (distanceTraveledWithEncoder || lidarDetectsGearPeg);
}

// Called once after isFinished returns true
void DriveToGearWithLidar::End() {
	CommandBase::driveTrain->DriveTank(0.0, 0.0);
	requiresDriveTrainSubsystem = false;
	initialDistanceConfigured = false;
	CommandBase::driveTrain->is_aiming = false;
	zeroGyro = false;
	distanceTraveledWithEncoder = false;

	initializeI2CMultiplxerChannelToOpen = false;
	configureLidar = false;
	startLidarDetectingGearPeg = false;
	waitingForGearPeg = false;
	lidarDetectsGearPeg = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveToGearWithLidar::Interrupted() {
	End();
}
