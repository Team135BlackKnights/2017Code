#include "AlignRobotWithGuardrail.h"

AlignRobotWithGuardrail::AlignRobotWithGuardrail(double driveTrainMotorPower, bool rightHopper) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(ultrasonicSensor.get());
	Requires(driveTrain.get());

	this->driveTrainMotorPower = driveTrainMotorPower;
	this->rightHopper = rightHopper;
}

// Called just before this Command runs the first time
void AlignRobotWithGuardrail::Initialize() {
	pingedFrontUltrasonicSensor = false;
	pingedBackUltrasonicSensor = true;

	getDesiredAngleToTurn = false;

	CommandBase::driveTrain->ZeroGyroAngle();
	zeroGyro = true;

	alignedRobotWithGuardRail = false;

	CommandBase::ultrasonicSensor->usingUltrasonicSensor = true;
}

// Called repeatedly when this Command is scheduled to run
void AlignRobotWithGuardrail::Execute() {
	if (zeroGyro == false) {
		CommandBase::driveTrain->ZeroGyroAngle();
		zeroGyro = true;
	}

	if (pingedFrontUltrasonicSensor == false) {
		CommandBase::ultrasonicSensor->PingGearUltrasonicSensor();
		pingedFrontUltrasonicSensor = true;
	}
	else if (pingedFrontUltrasonicSensor) {
		initialFrontUltrasonicSensorValue = CommandBase::ultrasonicSensor->GetGearUltrasonicSensorValueInches();
		pingedFrontUltrasonicSensor = false;
	}

	if (pingedBackUltrasonicSensor == false) {
		CommandBase::ultrasonicSensor->PingSideUltrasonicSensor(UltrasonicSensor::LEFT_SIDE_ULTRASONIC_SENSOR);
		pingedBackUltrasonicSensor = true;
	}
	else if (pingedBackUltrasonicSensor) {
		initialBackUltrasonicSensorValue = CommandBase::ultrasonicSensor->GetSideUltrasonicSensorValueInches(UltrasonicSensor::LEFT_SIDE_ULTRASONIC_SENSOR);
		pingedBackUltrasonicSensor = false;
	}

	if (getDesiredAngleToTurn == false) {
		if (initialFrontUltrasonicSensorValue < 5000.0 && initialFrontUltrasonicSensorValue > 0.0 && initialBackUltrasonicSensorValue < 5000.0 && initialBackUltrasonicSensorValue > 0.0) {
			desiredAngleToTurn = CommandBase::ultrasonicSensor->GetAngleToTurnToAlignWithGuardRail(initialFrontUltrasonicSensorValue, initialBackUltrasonicSensorValue, this->rightHopper);
			getDesiredAngleToTurn = true;
		}
	}
	else if (getDesiredAngleToTurn) {
		currentGyroAngle = CommandBase::driveTrain->GetGyroAngle();

		if (desiredAngleToTurn > 0.0) {
			//std::cout << "Turning Right" << std::endl;
			if (currentGyroAngle >= desiredAngleToTurn) {
				CommandBase::driveTrain->DriveTank(0.0, 0.0);
				alignedRobotWithGuardRail = true;
			}
			else {
				//std::cout << "Moving Right" << std::endl;
				CommandBase::driveTrain->DriveTank(0.0, (-1 * this->driveTrainMotorPower));
			}
		}
		else if (desiredAngleToTurn < 0.0) {
			//std::cout << "Turning Left" << std::endl;
			if (currentGyroAngle <= desiredAngleToTurn) {
				CommandBase::driveTrain->DriveTank(0.0, 0.0);
				alignedRobotWithGuardRail = true;
			}
			else {
				//std::cout << "Moving Left" << std::endl;
				CommandBase::driveTrain->DriveTank((-1 * this->driveTrainMotorPower), 0.0);
			}
		}
		else if (desiredAngleToTurn == 0.0) {
			alignedRobotWithGuardRail = true;
		}
	}
}

// Make this return true when this Command no longer needs to run execute()
bool AlignRobotWithGuardrail::IsFinished() {
	return alignedRobotWithGuardRail;
}

// Called once after isFinished returns true
void AlignRobotWithGuardrail::End() {
	CommandBase::driveTrain->DriveTank(0.0, 0.0);
	getDesiredAngleToTurn = false;
	alignedRobotWithGuardRail = false;
	CommandBase::ultrasonicSensor->usingUltrasonicSensor = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AlignRobotWithGuardrail::Interrupted() {
	End();
}
