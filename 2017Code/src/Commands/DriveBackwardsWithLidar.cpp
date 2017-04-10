#include "DriveBackwardsWithLidar.h"

DriveBackwardsWithLidar::DriveBackwardsWithLidar(double initialDriveTrainMotorPower, double desiredLidarValueToDriveUntil, double desiredInitialDistanceToTravel) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get());
	Requires(CommandBase::lidars.get());

	this->initialDriveTrainMotorPower = initialDriveTrainMotorPower;
	this->desiredLidarValueToDriveUntil = (desiredLidarValueToDriveUntil + DISTANCE_BETWEEN_LIDAR_AND_EDGE_OF_BUMPER);
	this->desiredInitialDistanceToTravel = desiredInitialDistanceToTravel;

	timer = new frc::Timer();
}

// Called just before this Command runs the first time
void DriveBackwardsWithLidar::Initialize() {
	CommandBase::lidars->TurnLidarOnOff(Lidars::TURN_LIDAR_ON);
	turnOnLidar = true;

	CommandBase::lidars->OpenLidarChannelOnMultiplexer(Lidars::VALUE_TO_OPEN_FRONT_LIDAR_CHANNEL_7);
	initializeI2CMultiplexerChannel = true;

	CommandBase::lidars->TurnOffDetectorBiasBetweenLidarAcquisitions();
	turnOffDetectorBiasBetweenAcquisitions = true;

	configureLidar = false;

	startUsingLidarValues = false;

	robotAtDesiredLidarValue = false;

	CommandBase::driveTrain->ZeroGyroAngle();
	zeroGyroAngle = true;

	initialDistanceTraveled = CommandBase::driveTrain->GetDistance(DriveTrain::RIGHT_SIDE_ENCODER);
	initializedEncoderDistanceTravel = true;

	startUsingEncoderValues = false;
	startEncoderBasedSlowerMotorPower = false;
	robotTraveledDistanceWithEncoders = false;

	initializeTimer = false;
}

// Called repeatedly when this Command is scheduled to run
void DriveBackwardsWithLidar::Execute() {
	if (turnOnLidar == false) {
		CommandBase::lidars->TurnLidarOnOff(Lidars::TURN_LIDAR_ON);
		turnOnLidar = true;
	}

	if (initializeI2CMultiplexerChannel == false) {
		CommandBase::lidars->OpenLidarChannelOnMultiplexer(Lidars::VALUE_TO_OPEN_FRONT_LIDAR_CHANNEL_7);
		initializeI2CMultiplexerChannel = true;
	}

	if (turnOffDetectorBiasBetweenAcquisitions == false) {
		CommandBase::lidars->TurnOffDetectorBiasBetweenLidarAcquisitions();
		turnOffDetectorBiasBetweenAcquisitions = true;
	}

	if (zeroGyroAngle == false) {
		CommandBase::driveTrain->ZeroGyroAngle();
		zeroGyroAngle = true;
	}

	if (initializedEncoderDistanceTravel == false) {
		initialDistanceTraveled = CommandBase::driveTrain->GetDistance(DriveTrain::RIGHT_SIDE_ENCODER);
		initializedEncoderDistanceTravel = true;
	}

	currentGyroAngle = CommandBase::driveTrain->GetGyroAngle();

	if (configureLidar == false) {
		CommandBase::lidars->ConfigureLidar();
		configureLidar = true;
	}
	else if (configureLidar) {
		lidarUpperByte = CommandBase::lidars->GetUpperByte();
		frc::Wait(.002);
		lidarLowerByte = CommandBase::lidars->GetLowerByte();
		currentlidarValueIN = CommandBase::lidars->GetLidarValue(lidarLowerByte, lidarUpperByte, Lidars::DISTANCE_UNIT_ARRAY[Lidars::INCHES]);
		configureLidar = false;
	}

	frc::SmartDashboard::PutNumber("Autonomous Lidar Value:", currentlidarValueIN);

	if (currentlidarValueIN != 0.0 && startUsingLidarValues == false && startUsingEncoderValues == false) {
		if (configureLidar == false) {
			if (savedLidarValueIN != currentlidarValueIN) {
				savedLidarValueIN = currentlidarValueIN;
				goodLidarCounter++;
			}
			else if (savedLidarValueIN == currentlidarValueIN) {
				badLidarCounter++;
			}

			if (badLidarCounter >= 3) {
				startUsingEncoderValues = true;
				startUsingLidarValues = false;
			}
			else if (goodLidarCounter >= 3) {
				startUsingLidarValues = true;
				startUsingEncoderValues = false;
			}
		}
	}
	else if (currentlidarValueIN == 0.0 && startUsingLidarValues == false && startUsingEncoderValues == false) {
		if (initializeTimer == false) {
			timer->Reset();
			timer->Start();
			initializeTimer = true;
		}

		timerValue = timer->Get();

		if (timerValue >= TIME_TO_WAIT_UNTIL_USING_ENCODER_VALUES) {
			startUsingEncoderValues = true;
		}
	}

	if (startUsingLidarValues) {
		std::cout << "Lidar" << std::endl;
		differenceBetweenDesiredAndCurrentLidarValue = (this->desiredLidarValueToDriveUntil - currentlidarValueIN);
		if (differenceBetweenDesiredAndCurrentLidarValue <= ESTIMATED_COAST_DISTANCE_IN) {
			CommandBase::driveTrain->DriveTank(0.0, 0.0);
			robotAtDesiredLidarValue = true;
		}
		else if (differenceBetweenDesiredAndCurrentLidarValue < DIFFERENCE_BETWEEN_DESIRED_AND_CURRENT_LIDAR_VALUES_TO_DRIVE_SLOWER_PART_2) {
			CommandBase::driveTrain->DriveStraightWithGyro(SLOWER_DRIVE_TRAIN_MOTOR_POWER_PART_2, currentGyroAngle);
			robotAtDesiredLidarValue = false;
		}
		else if (differenceBetweenDesiredAndCurrentLidarValue < DIFFERENCE_BETWEEN_DESIRED_AND_CURRENT_LIDAR_VALUES_TO_DRIVE_SLOWER_PART_1) {
			CommandBase::driveTrain->DriveStraightWithGyro(SLOWER_DRIVE_TRAIN_MOTOR_POWER_PART_1, currentGyroAngle);
			robotAtDesiredLidarValue = false;
		}
		else {
			CommandBase::driveTrain->DriveStraightWithGyro(this->initialDriveTrainMotorPower, currentGyroAngle);
			robotAtDesiredLidarValue = false;
		}
	}
	else if (startUsingEncoderValues) {
		std::cout << "Encoder" << std::endl;
		currentDistanceTraveled = CommandBase::driveTrain->GetDistance(DriveTrain::RIGHT_SIDE_ENCODER);
		if (startEncoderBasedSlowerMotorPower == false) {
			differenceBetweenInitialAndCurrentDistanceTraveled = (fabs(currentDistanceTraveled - initialDistanceTraveled));
			if (differenceBetweenInitialAndCurrentDistanceTraveled >= this->desiredInitialDistanceToTravel) {
				initialDistanceTraveledForSlowerMotorPower = currentDistanceTraveled;
				startEncoderBasedSlowerMotorPower = true;
			}
			else {
				CommandBase::driveTrain->DriveStraightWithGyro(this->initialDriveTrainMotorPower, currentGyroAngle);
			}
		}

		if (startEncoderBasedSlowerMotorPower) {
			differenceBetweenCurrentAndInitialEncoderValuesSlowerMotorPower = (currentDistanceTraveled - initialDistanceTraveledForSlowerMotorPower);
			if (differenceBetweenCurrentAndInitialEncoderValuesSlowerMotorPower >= DISTANCE_TO_TRAVEL_SLOW_SPEED) {
				CommandBase::driveTrain->DriveStraightWithGyro(0.0, currentGyroAngle);
				robotTraveledDistanceWithEncoders = true;
			}
			else {
				CommandBase::driveTrain->DriveStraightWithGyro(ENCODER_BASED_SLOW_MOTOR_POWER, currentGyroAngle);
			}
		}
	}
	else {
		CommandBase::driveTrain->DriveStraightWithGyro(this->initialDriveTrainMotorPower, currentGyroAngle);
	}

	currentDistanceTraveled = CommandBase::driveTrain->GetDistance(DriveTrain::RIGHT_SIDE_ENCODER);
	std::cout << "Current Distance Traveled: " << currentDistanceTraveled << std::endl;
}

// Make this return true when this Command no longer needs to run execute()
bool DriveBackwardsWithLidar::IsFinished() {
	return robotAtDesiredLidarValue || robotTraveledDistanceWithEncoders;
}

// Called once after isFinished returns true
void DriveBackwardsWithLidar::End() {
	CommandBase::lidars->TurnLidarOnOff(Lidars::TURN_LIDAR_OFF);
	turnOnLidar = false;

	CommandBase::driveTrain->DriveTank(0.0, 0.0);

	initializeI2CMultiplexerChannel = false;
	turnOffDetectorBiasBetweenAcquisitions = false;
	configureLidar = false;

	startUsingLidarValues = false;

	robotAtDesiredLidarValue = false;

	zeroGyroAngle = false;

	initializedEncoderDistanceTravel = false;
	startUsingEncoderValues = false;
	startEncoderBasedSlowerMotorPower = false;
	robotTraveledDistanceWithEncoders = false;

	initializeTimer = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveBackwardsWithLidar::Interrupted() {
	End();
}
