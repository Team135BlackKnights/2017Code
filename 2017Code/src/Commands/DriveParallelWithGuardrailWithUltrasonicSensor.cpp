#include "DriveParallelWithGuardrailWithUltrasonicSensor.h"

DriveParallelWithGuardrailWithUltrasonicSensor::DriveParallelWithGuardrailWithUltrasonicSensor(double desiredDistanceToTravel, double distanceAwayFromGuardrailToDrive, double driveTrainMotorPower, bool rightHopperAndShoot) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get());
	Requires(CommandBase::ultrasonicSensor.get());
	Requires(CommandBase::lidars.get());

	this->desiredDistanceToTravel = desiredDistanceToTravel;
	this->distanceAwayFromGuardrailToDrive = distanceAwayFromGuardrailToDrive;
	this->driveTrainMotorPower = driveTrainMotorPower;
	this->rightHopperAndShoot = rightHopperAndShoot;
}

// Called just before this Command runs the first time
void DriveParallelWithGuardrailWithUltrasonicSensor::Initialize() {
	initialDistanceTraveled = CommandBase::driveTrain->GetDistance(DriveTrain::RIGHT_SIDE_ENCODER);
	initializeEncoderDriveDistance = true;

	doneWithDrivingWithUltrasonicSensor = false;

	CommandBase::driveTrain->is_aiming = true;
	CommandBase::ultrasonicSensor->usingUltrasonicSensor = true;
	initializeIsAimingBoolean = true;

	CommandBase::driveTrain->ZeroGyroAngle();
	zeroGyro = true;

	initializeInitialGyroAngle = false;

	adjustRobotWithGyro = false;

	initialUltrasonicSensorValue = CommandBase::ultrasonicSensor->GetSideUltrasonicSensorValueInches(UltrasonicSensor::LEFT_SIDE_ULTRASONIC_SENSOR);
	getInitialUltrasonicSensorValue = true;

	initializeDistanceToTravel = false;
}

// Called repeatedly when this Command is scheduled to run
void DriveParallelWithGuardrailWithUltrasonicSensor::Execute() {
	if (initializeDistanceToTravel == false) {
		frontUltrasonicSensorValue = CommandBase::ultrasonicSensor->GetGearUltrasonicSensorValueInches();
		desiredDistanceToTravelFromLidar = CommandBase::lidars->GetDistanceToTravelToHopper(frontUltrasonicSensorValue);
		initializeDistanceToTravel = true;
	}

	if (getInitialUltrasonicSensorValue == false) {
		initialUltrasonicSensorValue = CommandBase::ultrasonicSensor->GetSideUltrasonicSensorValueInches(UltrasonicSensor::LEFT_SIDE_ULTRASONIC_SENSOR);
		getInitialUltrasonicSensorValue = true;
	}

	if (initializeEncoderDriveDistance == false) {
		initialDistanceTraveled = CommandBase::driveTrain->GetDistance(DriveTrain::RIGHT_SIDE_ENCODER);
		initializeEncoderDriveDistance = true;
	}

	if (initializeIsAimingBoolean == false) {
		CommandBase::driveTrain->is_aiming = true;
		CommandBase::ultrasonicSensor->usingUltrasonicSensor = true;
		initializeIsAimingBoolean = true;
	}

	if (zeroGyro == false) {
		CommandBase::driveTrain->ZeroGyroAngle();
		zeroGyro = true;
	}

	if (this->rightHopperAndShoot) {
		sideUltrasonicSensorValue = CommandBase::ultrasonicSensor->GetSideUltrasonicSensorValueInches(UltrasonicSensor::LEFT_SIDE_ULTRASONIC_SENSOR);
	}
	else if (this->rightHopperAndShoot == false) {
		sideUltrasonicSensorValue = CommandBase::ultrasonicSensor->GetSideUltrasonicSensorValueInches(UltrasonicSensor::RIGHT_SIDE_ULTRASONIC_SENSOR);
	}

	gyroAngle = CommandBase::driveTrain->GetGyroAngle();
	acutalDistanceRobotIsFromWall = CommandBase::ultrasonicSensor->GetActualDistanceFromGuardrail(sideUltrasonicSensorValue, gyroAngle);

	currentDistanceTraveled = CommandBase::driveTrain->GetDistance(DriveTrain::RIGHT_SIDE_ENCODER);
	differenceBetweenInitialAndCurrentDistance = fabs((currentDistanceTraveled - initialDistanceTraveled));
	if (differenceBetweenInitialAndCurrentDistance >= desiredDistanceToTravelFromLidar) {
		CommandBase::driveTrain->DriveTank(0.0, 0.0);
		doneWithDrivingWithUltrasonicSensor = true;
	}
	else if (differenceBetweenInitialAndCurrentDistance < desiredDistanceToTravelFromLidar) {
		if ((((fabs(acutalDistanceRobotIsFromWall - initialUltrasonicSensorValue)) < THRESHOLD_FOR_ADJUSTING_DRIVE_STRAIGHT_WITH_ULTRSONIC_SENSOR) && ((fabs(gyroAngle)) > THRESHOLD_GYRO_ANGLE_FOR_ADJUSTING)) || adjustRobotWithGyro) {
			std::cout << "Adjusting" << std::endl;
			if (initializeInitialGyroAngle == false) {
				initialGyroAngleForAdjusting = gyroAngle;
				initializeInitialGyroAngle = true;
				adjustRobotWithGyro = true;
			}

			gyroAngle = CommandBase::driveTrain->GetGyroAngle();

			if (initialGyroAngleForAdjusting < 0) {
				//  Move Right Side Backwards
				if (gyroAngle >= 0) {
					adjustRobotWithGyro = false;
				}
				else {
					CommandBase::driveTrain->DriveTank(0.0, (this->driveTrainMotorPower - .25));
				}
			}
			else if (initialGyroAngleForAdjusting > 0) {
				//  Move Left Side Backwards
				if (gyroAngle <= 0) {
					adjustRobotWithGyro = false;
				}
				else {
					CommandBase::driveTrain->DriveTank((this->driveTrainMotorPower - .25), 0.0);
				}
			}
		}
		else {
			std::cout << "Tracking" << std::endl;
			CommandBase::driveTrain->DriveStraightWithUltrasonicSensor(acutalDistanceRobotIsFromWall, initialUltrasonicSensorValue, this->driveTrainMotorPower, this->rightHopperAndShoot);
		}
	}

	std::cout << "Gyro ANgle: " << gyroAngle << std::endl;
	std::cout << "Ultrasonic Sensor Cos Value: " << acutalDistanceRobotIsFromWall << std::endl;
}

// Make this return true when this Command no longer needs to run execute()
bool DriveParallelWithGuardrailWithUltrasonicSensor::IsFinished() {
	return doneWithDrivingWithUltrasonicSensor;
}

// Called once after isFinished returns true
void DriveParallelWithGuardrailWithUltrasonicSensor::End() {
	CommandBase::driveTrain->DriveTank(0.0, 0.0);
	CommandBase::driveTrain->is_aiming = false;
	CommandBase::ultrasonicSensor->usingUltrasonicSensor = false;
	zeroGyro = false;
	initializeIsAimingBoolean = false;
	initializeEncoderDriveDistance = false;
	getInitialUltrasonicSensorValue = false;
	initializeDistanceToTravel = false;
	doneWithDrivingWithUltrasonicSensor = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveParallelWithGuardrailWithUltrasonicSensor::Interrupted() {

}
