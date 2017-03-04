#include "ShooterHood.h"
#include "../RobotMap.h"
#include "Commands/ReadHoodEncoderValue.h"
#include <iostream>
#include <math.h>

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
	angleBelowMaxHoodAngle = (((double)encoderPosition) * ENCODER_POSITION_TO_ANGLE_OF_SHOOTER_HOOD);
	actualAngle = (MAX_ANGLE_VALUE - angleBelowMaxHoodAngle);
	return actualAngle;
}

int ShooterHood::GetEncoderPositionOfShooterHoodGivenAngle(double angle) {
	calculatedAngleBelowMaxHoodAngle = (MAX_ANGLE_VALUE - angle);
	doubleEncoderPosition = (calculatedAngleBelowMaxHoodAngle * ANGLE_TO_ENCODER_POSITION_OF_SHOOTER_HOOD);
	intEncoderPosition = ((int)(rint(doubleEncoderPosition)));
	return intEncoderPosition;
}

bool ShooterHood::DriveShooterHoodMotorToDesiredAngle(double desiredAngle, double motorPower) {
	currentEncoderPosition = this->GetShooterHoodEncoderPosition();
	currentAngle = this->GetAngleOfShooterHoodGivenEncoderPosition(currentEncoderPosition);
	desiredEncoderPosition = this->GetEncoderPositionOfShooterHoodGivenAngle(desiredAngle);
	differenceBetweenCurrentAndDesiredEncoderPositions = (abs(currentEncoderPosition - desiredEncoderPosition));

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
				if (differenceBetweenCurrentAndDesiredEncoderPositions > THRESHOLD_ENCODER_RANGE_TO_START_SLOWING_MOTOR_POWER) {
					desiredMotorPowerToRunAt = motorPower;
				}
				else if (differenceBetweenCurrentAndDesiredEncoderPositions <= THRESHOLD_ENCODER_RANGE_TO_START_SLOWING_MOTOR_POWER) {
					desiredMotorPowerToRunAt = APPROACHING_DESIRED_ENCODER_POSITION_MOTOR_POWER;
				}
				this->DriveShooterHoodMotor(desiredMotorPowerToRunAt);
			}
		}
		else if (driveHoodToIncreaseAngle == false) {
			if (currentAngle <= desiredAngle) {
				this->DriveShooterHoodMotor(0.0);
				initializeDirectionOfHoodToMove = false;
				drivenToAngle = true;
			}
			else {
				if (differenceBetweenCurrentAndDesiredEncoderPositions > THRESHOLD_ENCODER_RANGE_TO_START_SLOWING_MOTOR_POWER) {
					desiredMotorPowerToRunAt = ((-1) * motorPower);
				}
				else if (differenceBetweenCurrentAndDesiredEncoderPositions <= THRESHOLD_ENCODER_RANGE_TO_START_SLOWING_MOTOR_POWER) {
					desiredMotorPowerToRunAt = ((-1) * APPROACHING_DESIRED_ENCODER_POSITION_MOTOR_POWER);
				}
				this->DriveShooterHoodMotor(-desiredMotorPowerToRunAt);
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
	else if (closeShot == false) {
		chosenVelocityOfShooter = SHOOTER_FAR_SHOT_M_PER_SEC;
	}

	valueAngleForLoopHasToEqual = ((((-1) * ACCELERATION_OF_GRAVITY * totalXDistance_M)/((chosenVelocityOfShooter)^2)) - (Y_DISTANCE_M/totalXDistance_M));

	for (int i = 0; i <= 90; i++) {
		radiansAngleValue = ConvertDegreesToRadians(((double)i));
		valueWithInputtedAngle = (((Y_DISTANCE_M/totalXDistance_M) * cos((2.0 * radiansAngleValue))) - (sin((2.0 * radiansAngleValue))));
		currentDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue = (fabs(valueAngleForLoopHasToEqual - valueWithInputtedAngle));
		if (firstAngleValueReceived == false) {
			pastDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue = currentDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue;
			calculatedAngleFirstRound = ((double)i);
			firstAngleValueReceived = true;
		}
		else if (firstAngleValueReceived && currentDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue < pastDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue) {
			pastDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue = currentDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue;
			calculatedAngleFirstRound = ((double)i);
		}
	}

	maxPossibleAngleFirstRound = (calculatedAngleFirstRound + 1.0);
	minPossibleAngleFirstRound = (calculatedAngleFirstRound - 1.0);

	return 0.0;
}

void ShooterHood::CheckIfHoodHitsLimitSwitch() {
	if (this->GetMaxAngleLimitSwitch() == 1) {
		this->ZeroShooterHoodEncoder();
	}
	else if (this->GetMinAngleLimitSwitch() == 1) {
		this->SetShooterHoodEncoder(MAX_ENCODER_VALUE);
	}
}

int ShooterHood::GetMaxAngleLimitSwitch() {
	return shooterHoodMotor->IsFwdLimitSwitchClosed();
}

int ShooterHood::GetMinAngleLimitSwitch() {
	return shooterHoodMotor->IsRevLimitSwitchClosed();
}

double ShooterHood::ConvertDegreesToRadians(double degrees) {
	return (degrees * DEGREES_TO_RADIANS_CONSTANT);
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
