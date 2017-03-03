#include "ShooterHood.h"
#include "../RobotMap.h"
#include "Commands/ReadHoodEncoderValue.h"
#include <iostream>

ShooterHood::ShooterHood() : Subsystem("ShooterHood") {

}

void ShooterHood::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
	SetDefaultCommand(new ReadHoodEncoderValue());
}

void ShooterHood::InitializeShooterHoodMotor(bool competitionBot) {
	if (competitionBot) {
		shooterHoodMotor = new CANTalon(CB_HOOD_MOTOR_TALON_ID);
		shooterHoodMotor->SetInverted(CB_SHOOTER_HOOD_MOTOR_INVERTED);
	}
	else if (competitionBot == false) {
		shooterHoodMotor = new CANTalon(PB_HOOD_MOTOR_TALON_ID);
		shooterHoodMotor->SetInverted(PB_SHOOTER_HOOD_MOTOR_INVERTED);
	}
}

void ShooterHood::DriveShooterHoodMotor(double motorPower) {
	if (this->GetMinAngleLimitSwitch() == 1) {
		this->SetShooterHoodEncoder(MAX_ENCODER_VALUE);
		std::cout << "Min Angle Limit Switch Pressed" << std::endl;
	}
	else if (this->GetMaxAngleLimitSwitch() == 1) {
		this->ZeroShooterHoodEncoder();
		std::cout << "Max Angle Limit Switch Pressed" << std::endl;
	}

	shooterHoodMotor->Set(motorPower);
}

void ShooterHood::ConfigureShooterHoodEncoder() {
	shooterHoodMotor->ConfigEncoderCodesPerRev(SHOOTER_HOOD_ENCODER_COUNTS);
	shooterHoodMotor->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
	shooterHoodMotor->SetStatusFrameRateMs(CANTalon::StatusFrameRate::StatusFrameRateQuadEncoder, 10);
	shooterHoodMotor->SetSensorDirection(REVERSE_SHOOTER_HOOD_ENCODER_DIRECTION);
}

int ShooterHood::GetShooterHoodEncoderPosition() {
	return shooterHoodMotor->GetEncPosition();
}

void ShooterHood::SetShooterHoodEncoder(int encoderPosition) {
	shooterHoodMotor->SetEncPosition(encoderPosition);
}

void ShooterHood::ZeroShooterHoodEncoder() {
	shooterHoodMotor->SetEncPosition(0);
}

double ShooterHood::GetAngleOfShooterHoodGivenEncoderPosition(int encoderPosition) {
	angleAboveHood = (((double)encoderPosition) * ENCODER_POSITION_TO_ANGLE_OF_SHOOTER_HOOD);
	actualAngle = (angleAboveHood + MIN_ANGLE_VALUE);
	return actualAngle;
}

bool ShooterHood::DriveShooterHoodMotorToDesiredAngle(double desiredAngle, double motorPower) {
	currentEncoderPosition = this->GetShooterHoodEncoderPosition();
	currentAngle = this->GetAngleOfShooterHoodGivenEncoderPosition(currentEncoderPosition);

	if (initializeDirectionOfHoodToMove == false) {
		if (desiredAngle >= currentAngle) {
			driveHoodToIncreaseAngle = true;
		}
		else if (currentAngle > desiredAngle) {
			driveHoodToIncreaseAngle = false;
		}
		drivenToAngle = false;
		initializeDirectionOfHoodToMove = true;
	}
	else if (initializeDirectionOfHoodToMove) {
		if (driveHoodToIncreaseAngle) {
			if (currentAngle >= desiredAngle) {
				this->DriveShooterHoodMotor(0.0);
				initializeDirectionOfHoodToMove = false;
				drivenToAngle = true;
			}
			else {
				this->DriveShooterHoodMotor(motorPower);
			}
		}
		else if (driveHoodToIncreaseAngle == false) {
			if (currentAngle <= desiredAngle) {
				this->DriveShooterHoodMotor(0.0);
				initializeDirectionOfHoodToMove = false;
				drivenToAngle = true;
			}
			else {
				this->DriveShooterHoodMotor(-motorPower);
			}
		}
	}
	return drivenToAngle;
}

double ShooterHood::GetDesiredAngleOfShooterHood(double xDistanceFromLidar_CM, bool closeShot, int shooterAngledPosition) {
	if (shooterAngledPosition == SHOOTER_ANGLED_POSITION_ARRAY[FURTHEST_POINT_FROM_STRAIGHT_ON]) {
		xDistanceFromLidar_M = (xDistanceFromLidar_CM/100.0);
		totalXDistance_M = (xDistanceFromLidar_M + ADDED_X_DISTANCE_MAX_INSIDE_BOILER_M);
	}
	else if (shooterAngledPosition == SHOOTER_ANGLED_POSITION_ARRAY[STRAIGHT_ON]) {
		xDistanceFromLidar_M = (xDistanceFromLidar_CM/100.0);
		totalXDistance_M = (xDistanceFromLidar_M + ADDED_X_DISTANCE_MIN_INSIDE_BOILER_M);
	}

	if (closeShot) {
		chosenVelocityOfShooter = SHOOTER_CLOSE_SHOT_M_PER_SEC;
	}
	else {
		chosenVelocityOfShooter = SHOOTER_FAR_SHOT_M_PER_SEC;
	}

	//valueAngleForLoopHasToEqual = ((((-1) * ACCELERATION_OF_GRAVITY * totalXDistance_M)/((chosenVelocityOfShooter)^2)) - (Y_DISTANCE_M/totalXDistance_M));

	return 0.0;
}

int ShooterHood::GetMaxAngleLimitSwitch() {
	return shooterHoodMotor->IsFwdLimitSwitchClosed();
}

int ShooterHood::GetMinAngleLimitSwitch() {
	return shooterHoodMotor->IsRevLimitSwitchClosed();
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
