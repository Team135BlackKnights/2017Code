#include "DriveToGearWithLidar.h"

DriveToGearWithLidar::DriveToGearWithLidar(double driveTrainMotorPower, bool rightGear) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get());
	Requires(CommandBase::lidars.get());


	this->driveTrainMotorPower = driveTrainMotorPower;
	this->rightGear = rightGear;

	timer = new frc::Timer();
}

// Called just before this Command runs the first time
void DriveToGearWithLidar::Initialize() {
	CommandBase::driveTrain->ZeroGyroAngle();
	zeroGyro = true;
	CommandBase::driveTrain->is_aiming = true;
	requiresDriveTrainSubsystem = true;

	initializeI2CMultiplxerChannelToOpen = false;
	configureLidar = false;
	receiveLidarUpperByte = false;
	startLidarDetectingGearPeg = false;
	initializeGearPegLidarTimer = false;
	waitingForGearPeg = false;
	lidarDetectsGearPeg = false;
	traveledToTurningRadiusOfRobot = false;
	configureInitialDistanceBeforeTravelingExtraDistance = false;
}

// Called repeatedly when this Command is scheduled to run
void DriveToGearWithLidar::Execute() {
	if (requiresDriveTrainSubsystem == false) {
		CommandBase::driveTrain->is_aiming = true;
		requiresDriveTrainSubsystem = true;
	}

	if (zeroGyro == false) {
		CommandBase::driveTrain->ZeroGyroAngle();
		zeroGyro = true;
	}

	gyroAngle = CommandBase::driveTrain->GetGyroAngle();

	if (lidarDetectsGearPeg == false) {
		CommandBase::driveTrain->DriveStraightWithGyro(this->driveTrainMotorPower, gyroAngle);
	}

	if (initializeI2CMultiplxerChannelToOpen == false) {
		if (this->rightGear == RIGHT_GEAR) {
			CommandBase::lidars->OpenLidarChannelOnMultiplexer(Lidars::VALUE_TO_OPEN_LIDAR_CHANNEL_7_RIGHT_LIDAR);
		}
		else if (this->rightGear == LEFT_GEAR) {
			CommandBase::lidars->OpenLidarChannelOnMultiplexer(Lidars::VALUE_TO_OPEN_LIDAR_CHANNEL_6_LEFT_LIDAR);
		}
		initializeI2CMultiplxerChannelToOpen = true;
	}

	if (configureLidar == false) {
		CommandBase::lidars->ConfigureLidar();
		configureLidar = true;
	}
	else if (configureLidar && receiveLidarUpperByte == false) {
		lidarUpperByte = CommandBase::lidars->GetUpperByte();
		receiveLidarUpperByte = true;
	}
	else if  (configureLidar && receiveLidarUpperByte) {
		lidarLowerByte = CommandBase::lidars->GetLowerByte();
		currentLidarValueIN = CommandBase::lidars->GetLidarValue(lidarLowerByte, lidarUpperByte, Lidars::DISTANCE_UNIT_ARRAY[Lidars::INCHES]);
		configureLidar = false;
		receiveLidarUpperByte = false;
		startLidarDetectingGearPeg = true;
	}

	if (startLidarDetectingGearPeg && waitingForGearPeg == false) {
		savedLidarTransparentAirshipValue = currentLidarValueIN;
		waitingForGearPeg = true;
	}
	else if (startLidarDetectingGearPeg && waitingForGearPeg && lidarDetectsGearPeg == false) {
		if ((currentLidarValueIN + DISTANCE_DROP_OF_LIDAR_FROM_ARISHIP_TO_GEAR_PEG_IN) <= savedLidarTransparentAirshipValue) {
			if (initializeGearPegLidarTimer == false) {
				timer->Reset();
				timer->Start();
				initializeGearPegLidarTimer = true;
			}

			currentTimerValue = timer->Get();

			if (initializeGearPegLidarTimer && (currentTimerValue >= TIME_TO_WAIT_TO_REACT_TO_GEAR_PEG)) {
				lidarDetectsGearPeg = true;
			}
		}
		else {
			timer->Stop();
			timer->Reset();
			initializeGearPegLidarTimer = false;
			lidarDetectsGearPeg = false;
		}
	}

	currentDistanceTraveledWhileTravelingExtraDistance = CommandBase::driveTrain->GetDistance(DriveTrain::RIGHT_SIDE_ENCODER);
	if (lidarDetectsGearPeg) {
		if (configureInitialDistanceBeforeTravelingExtraDistance == false) {
			initialDistanceTraveledBeforeTravelingExtraDistance = currentDistanceTraveledWhileTravelingExtraDistance;
			configureInitialDistanceBeforeTravelingExtraDistance = true;
		}

		differenceBetweenCurrentAndInitialDistanceWhileTravelingExtraDistance = (fabs(currentDistanceTraveledWhileTravelingExtraDistance - initialDistanceTraveledBeforeTravelingExtraDistance));

		if (differenceBetweenCurrentAndInitialDistanceWhileTravelingExtraDistance >= DISTANCE_BETWEEN_LIDAR_AND_TURNING_POINT_OF_ROBOT) {
			CommandBase::driveTrain->DriveTank(0.0, 0.0);
			traveledToTurningRadiusOfRobot = true;
		}
		else {
			CommandBase::driveTrain->DriveStraightWithGyro(DRIVE_TRAIN_MOTOR_POWER_FOR_EXTRA_DISTANCE, gyroAngle);
			traveledToTurningRadiusOfRobot = false;
		}
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveToGearWithLidar::IsFinished() {
	return traveledToTurningRadiusOfRobot;
}

// Called once after isFinished returns true
void DriveToGearWithLidar::End() {
	CommandBase::driveTrain->DriveTank(0.0, 0.0);
	requiresDriveTrainSubsystem = false;
	CommandBase::driveTrain->is_aiming = false;
	zeroGyro = false;

	initializeI2CMultiplxerChannelToOpen = false;
	configureLidar = false;
	receiveLidarUpperByte = false;
	startLidarDetectingGearPeg = false;
	initializeGearPegLidarTimer = false;
	waitingForGearPeg = false;
	lidarDetectsGearPeg = false;
	traveledToTurningRadiusOfRobot = false;
	configureInitialDistanceBeforeTravelingExtraDistance = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveToGearWithLidar::Interrupted() {
	End();
}
