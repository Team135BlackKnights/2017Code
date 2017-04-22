#include "DriveParallelWithGuardrailWithUltrasonicSensor.h"

DriveParallelWithGuardrailWithUltrasonicSensor::DriveParallelWithGuardrailWithUltrasonicSensor(double desiredDistanceToTravel, double distanceAwayFromGuardrailToDrive, double driveTrainMotorPower, bool rightHopperAndShoot) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get());
	Requires(CommandBase::ultrasonicSensor.get());

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
	initializeIsAimingBoolean = true;

	CommandBase::ultrasonicSensor->usingUltrasonicSensor = true;

	CommandBase::driveTrain->ZeroGyroAngle();
	zeroGyro = true;
}

// Called repeatedly when this Command is scheduled to run
void DriveParallelWithGuardrailWithUltrasonicSensor::Execute() {
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

	if (DRIVE_ENDING == DRIVE_UNTIL_ULTRASONIC_SENSOR_IS_HIGH_VALUE) {
		if (acutalDistanceRobotIsFromWall >= THRESHOLD_ULTRASONIC_SENSOR_VALUE) {
			CommandBase::driveTrain->DriveTank(0.0, 0.0);
			doneWithDrivingWithUltrasonicSensor = true;
		}
		else if (acutalDistanceRobotIsFromWall <= THRESHOLD_ULTRASONIC_SENSOR_VALUE) {
			CommandBase::driveTrain->DriveStraightWithUltrasonicSensor(acutalDistanceRobotIsFromWall, this->distanceAwayFromGuardrailToDrive, this->driveTrainMotorPower, this->rightHopperAndShoot);
		}
	}
	else if (DRIVE_ENDING == DRIVE_WITH_DISTANCE) {
		if (initializeEncoderDriveDistance == false) {
			currentDistanceTraveled = CommandBase::driveTrain->GetStraightDistanceTraveled(DriveTrain::RIGHT_SIDE_ENCODER, gyroAngle, CONFIGURE_INITIAL_DISTANCE, DRIVING_BACKWARDS);
			initializeEncoderDriveDistance = true;
		}
		else if (initializeEncoderDriveDistance) {
			currentDistanceTraveled = CommandBase::driveTrain->GetStraightDistanceTraveled(DriveTrain::RIGHT_SIDE_ENCODER, gyroAngle, DONT_RECONFIGURE_INITIAL_DISTANCE, DRIVING_BACKWARDS);
			if (currentDistanceTraveled >= this->desiredDistanceToTravel) {
				CommandBase::driveTrain->DriveTank(0.0, 0.0);
				doneWithDrivingWithUltrasonicSensor = true;
			}
			else if (currentDistanceTraveled < this->desiredDistanceToTravel) {
				CommandBase::driveTrain->DriveStraightWithUltrasonicSensor(acutalDistanceRobotIsFromWall, this->distanceAwayFromGuardrailToDrive, this->driveTrainMotorPower, this->rightHopperAndShoot);
			}
		}
	}

	std::cout << "Ultrasonic Sensor Value: " << sideUltrasonicSensorValue << std::endl;
	std::cout << "Actual Distance From Guardrail: " << acutalDistanceRobotIsFromWall << std::endl;
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
	doneWithDrivingWithUltrasonicSensor = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveParallelWithGuardrailWithUltrasonicSensor::Interrupted() {

}
