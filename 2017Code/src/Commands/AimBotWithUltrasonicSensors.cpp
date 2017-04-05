#include "AimBotWithUltrasonicSensors.h"

AimBotWithUltrasonicSensors::AimBotWithUltrasonicSensors(bool* sonarGood) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::ultrasonicSensor.get());
	Requires(CommandBase::driveTrain.get());
	sonBool = sonarGood;
	timer = new frc::Timer();
}

// Called just before this Command runs the first time
void AimBotWithUltrasonicSensors::Initialize() {
	leftUltrasonicSensorValue = ultrasonicSensor->GetUltrasonicSensorValueInches(UltrasonicSensor::LEFT_ULTRASONIC_SENSOR);
	rightUltrasonicSensorValue = ultrasonicSensor->GetUltrasonicSensorValueInches(UltrasonicSensor::RIGHT_ULTRASONIC_SENSOR);
	if(leftUltrasonicSensorValue > 100.0) {
		*sonBool = false;
		CommandBase::ultrasonicSensor->usingRightUltrasonicSensorForGearCamera = true;
	}
	else if(rightUltrasonicSensorValue > 100.0){
		*sonBool = false;
		CommandBase::ultrasonicSensor->usingRightUltrasonicSensorForGearCamera = false;
	}
	else
	{
		*sonBool = true;
	}
	desiredAngleToTurnDriveTrain = ultrasonicSensor->GetAngleToTurnForGear(rightUltrasonicSensorValue, leftUltrasonicSensorValue);

	CommandBase::driveTrain->TurnPIDEnable(desiredAngleToTurnDriveTrain);

	CommandBase::driveTrain->is_aiming = true;

	timer->Reset();
	timer->Start();
	initializeAimBotWithUltrasonicSensors = true;
}

// Called repeatedly when this Command is scheduled to run
void AimBotWithUltrasonicSensors::Execute() {
	if (initializeAimBotWithUltrasonicSensors == false) {
		leftUltrasonicSensorValue = ultrasonicSensor->GetUltrasonicSensorValueInches(UltrasonicSensor::LEFT_ULTRASONIC_SENSOR);
		rightUltrasonicSensorValue = ultrasonicSensor->GetUltrasonicSensorValueInches(UltrasonicSensor::RIGHT_ULTRASONIC_SENSOR);

		desiredAngleToTurnDriveTrain = ultrasonicSensor->GetAngleToTurnForGear(rightUltrasonicSensorValue, leftUltrasonicSensorValue);

		CommandBase::driveTrain->TurnPIDEnable(desiredAngleToTurnDriveTrain);

		CommandBase::driveTrain->is_aiming = true;

		timer->Reset();
		timer->Start();
		initializeAimBotWithUltrasonicSensors = true;
	}
	timerValue = timer->Get();
	CommandBase::driveTrain->PIDTurning();

	frc::SmartDashboard::PutNumber("Desired Angle To Turn With Ultrasonic Sensors", desiredAngleToTurnDriveTrain);
}

// Make this return true when this Command no longer needs to run execute()
bool AimBotWithUltrasonicSensors::IsFinished() {
	return (timerValue >= MAX_TIME_FOR_EXECUTING_DRIVE_TRAIN_PID) || (!CommandBase::driveTrain->turnController->IsEnabled()) || sonBool == false;
}

// Called once after isFinished returns true
void AimBotWithUltrasonicSensors::End() {
	CommandBase::driveTrain->is_aiming = false;
	initializeAimBotWithUltrasonicSensors = false;
	timer->Stop();
	timer->Reset();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AimBotWithUltrasonicSensors::Interrupted() {
	End();
}
