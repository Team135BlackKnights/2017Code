#include "AutoGearOnPeg.h"

AutoGearOnPeg::AutoGearOnPeg() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get());
	Requires(CommandBase::ultrasonicSensor.get());
	Requires(CommandBase::gearHolder.get());
}

// Called just before this Command runs the first time
void AutoGearOnPeg::Initialize() {
	startMovingTowardsGear = true;
	startPuttingGearOnPeg = false;
	gearOnPeg = false;
	timer = new frc::Timer();
	timer->Reset();
	timer->Start();
}

// Called repeatedly when this Command is scheduled to run
void AutoGearOnPeg::Execute() {
	ultrasonicValue = CommandBase::ultrasonicSensor->GetUltrasonicSensorValueInches();
	//frc::SmartDashboard::PutNumber("Ultrasonic Sensor Value", ultrasonicValue);
	timerValue = timer->Get();

	if (startMovingTowardsGear) {
		if (ultrasonicValue > DISTANCE_AWAY_FROM_AIRSHIP_TO_DROP_GEAR_IN) {
			rightDriveTrainEncoderRPM = (fabs(CommandBase::driveTrain->GetEncoderRPM(DriveTrain::RIGHT_SIDE_ENCODER)));
			if ((rightDriveTrainEncoderRPM < DRIVE_TRAIN_ENCODER_RPM_THRESHOLD && startMovingRobot == false) || retryGearLineUp) {
				if (retryGearLineUp == false) {
					initialRightEncoderDistance = CommandBase::driveTrain->GetDistance(DriveTrain::RIGHT_SIDE_ENCODER);
					desiredRightEncoderDistance = (initialRightEncoderDistance + DISTANCE_TO_MOVE_AWAY_FROM_GEAR);
					CommandBase::driveTrain->DriveTank(DRIVE_TRAIN_MOTOR_POWER, DRIVE_TRAIN_MOTOR_POWER);
					retryGearLineUp = true;
				}
				else if (retryGearLineUp) {
					currentRightEncoderDistance = CommandBase::driveTrain->GetDistance(DriveTrain::RIGHT_SIDE_ENCODER);
					if (currentRightEncoderDistance >= desiredRightEncoderDistance) {
						CommandBase::driveTrain->DriveTank(0.0, 0.0);
						moveGearHolderDown = true;
						retryGearLineUp = false;
						startMovingRobot = true;
					}
					else {
						CommandBase::driveTrain->DriveTank(DRIVE_TRAIN_MOTOR_POWER, DRIVE_TRAIN_MOTOR_POWER);
					}
				}
			}
			else {
				if (moveGearHolderDown) {
					if (stopGearHolderDown == false) {
						timer->Reset();
						timer->Start();
						CommandBase::gearHolder->DriveGearHolderMotor(-GEAR_HOLDER_MOTOR_POWER);
						stopGearHolderDown = true;
					}
					else if (stopGearHolderDown && timerValue < TIME_TO_LOWER_GEAR_HOLDER) {
						CommandBase::gearHolder->DriveGearHolderMotor(-GEAR_HOLDER_MOTOR_POWER);
					}
					else if (stopGearHolderDown && timerValue >= TIME_TO_LOWER_GEAR_HOLDER) {
						CommandBase::gearHolder->DriveGearHolderMotor(0.0);
						timer->Reset();
						timer->Stop();
						stopGearHolderDown = false;
						moveGearHolderDown = false;
					}
				}
				else {
					if (rightDriveTrainEncoderRPM >= DRIVE_TRAIN_ENCODER_RPM_THRESHOLD) {
						startMovingRobot = false;
					}
					else {
						startMovingRobot = true;
					}
					CommandBase::driveTrain->DriveTank(-DRIVE_TRAIN_MOTOR_POWER, -DRIVE_TRAIN_MOTOR_POWER);
				}
			}
		}
		else if (ultrasonicValue <= DISTANCE_AWAY_FROM_AIRSHIP_TO_DROP_GEAR_IN) {
			CommandBase::driveTrain->DriveTank(0.0, 0.0);
			startMovingTowardsGear = false;
			startPuttingGearOnPeg = true;
		}
	}

	if (startPuttingGearOnPeg) {
		lowerLimitSwitchValue = CommandBase::gearHolder->GetLimitSwitchValue(GearHolder::LOWER_LIMIT_SWITCH_PORT);
		if (lowerLimitSwitchValue) {
			CommandBase::gearHolder->DriveGearHolderMotor(0.0);
			startPuttingGearOnPeg = false;
			gearOnPeg = true;
		}
		else {
			CommandBase::gearHolder->DriveGearHolderMotor(-GEAR_HOLDER_MOTOR_POWER);
		}
	}
}

// Make this return true when this Command no longer needs to run execute()
bool AutoGearOnPeg::IsFinished() {
	return gearOnPeg;
}

// Called once after isFinished returns true
void AutoGearOnPeg::End() {
	startMovingTowardsGear = true;
	startPuttingGearOnPeg = false;
	gearOnPeg = false;
	CommandBase::driveTrain->DriveTank(0.0, 0.0);
	CommandBase::gearHolder->DriveGearHolderMotor(0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutoGearOnPeg::Interrupted() {
	End();
}
