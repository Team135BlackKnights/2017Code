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
}

// Called repeatedly when this Command is scheduled to run
void DriveParallelWithGuardrailWithUltrasonicSensor::Execute() {
	if (initializeEncoderDriveDistance == false) {
		initialDistanceTraveled = CommandBase::driveTrain->GetDistance(DriveTrain::RIGHT_SIDE_ENCODER);
		initializeEncoderDriveDistance = true;
	}

	if (this->rightHopperAndShoot) {
		sideUltrasonicSensorValue = CommandBase::ultrasonicSensor->GetSideUltrasonicSensorValueInches(UltrasonicSensor::LEFT_SIDE_ULTRASONIC_SENSOR);
	}
	else if (this->rightHopperAndShoot == false) {
		sideUltrasonicSensorValue = CommandBase::ultrasonicSensor->GetSideUltrasonicSensorValueInches(UltrasonicSensor::RIGHT_SIDE_ULTRASONIC_SENSOR);
	}

	if (DRIVE_ENDING == DRIVE_UNTIL_ULTRASONIC_SENSOR_IS_HIGH_VALUE) {
		if (sideUltrasonicSensorValue >= THRESHOLD_ULTRASONIC_SENSOR_VALUE) {
			CommandBase::driveTrain->DriveTank(0.0, 0.0);
			doneWithDrivingWithUltrasonicSensor = true;
		}
		else if (sideUltrasonicSensorValue <= THRESHOLD_ULTRASONIC_SENSOR_VALUE) {
			CommandBase::driveTrain->DriveStraightWithUltrasonicSensor(sideUltrasonicSensorValue, this->distanceAwayFromGuardrailToDrive, this->driveTrainMotorPower, this->rightHopperAndShoot);
		}
	}
	else if (DRIVE_ENDING == DRIVE_WITH_DISTANCE) {
		currentDistanceTraveled = CommandBase::driveTrain->GetDistance(DriveTrain::RIGHT_SIDE_ENCODER);
		differenceBetweenCurrentAndDesiredDistance = (fabs(currentDistanceTraveled - initializeEncoderDriveDistance));

		if (differenceBetweenCurrentAndDesiredDistance >= this->desiredDistanceToTravel) {
			CommandBase::driveTrain->DriveTank(0.0, 0.0);
			doneWithDrivingWithUltrasonicSensor = true;
		}
		else if (differenceBetweenCurrentAndDesiredDistance < this->desiredDistanceToTravel) {
			CommandBase::driveTrain->DriveStraightWithUltrasonicSensor(sideUltrasonicSensorValue, this->distanceAwayFromGuardrailToDrive, this->driveTrainMotorPower, this->rightHopperAndShoot);
		}
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveParallelWithGuardrailWithUltrasonicSensor::IsFinished() {
	return doneWithDrivingWithUltrasonicSensor;
}

// Called once after isFinished returns true
void DriveParallelWithGuardrailWithUltrasonicSensor::End() {
	CommandBase::driveTrain->DriveTank(0.0, 0.0);
	initializeEncoderDriveDistance = false;
	doneWithDrivingWithUltrasonicSensor = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveParallelWithGuardrailWithUltrasonicSensor::Interrupted() {

}
