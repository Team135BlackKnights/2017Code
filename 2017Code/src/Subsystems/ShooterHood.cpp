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
	}
	else if (this->GetMaxAngleLimitSwitch() == 1) {
		this->ZeroShooterHoodEncoder();
	}

	shooterHoodMotor->Set(motorPower);
}

void ShooterHood::ConfigureShooterHoodEncoder() {
	shooterHoodMotor->ConfigEncoderCodesPerRev(SHOOTER_HOOD_ENCODER_COUNTS);
	shooterHoodMotor->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
	shooterHoodMotor->SetStatusFrameRateMs(CANTalon::StatusFrameRate::StatusFrameRateQuadEncoder, 10);
	shooterHoodMotor->SetSensorDirection(REVERSE_SHOOTER_HOOD_ENCODER_DIRECTION);
}


bool ShooterHood::HoodEncoderPluggedIn() {
	hoodEncoderPresent = shooterHoodMotor->IsSensorPresent(CANTalon::FeedbackDevice::QuadEncoder);
	if (hoodEncoderPresent == DISCONNECTED) {
		hoodEncoderPluggedIn = false;
		//std::cout << "Disconnected" << std::endl;
	}
	else if (hoodEncoderPresent == UNKNOWN_CONNECTED || hoodEncoderPresent == RECOGNIZED_CONNECTED) {
		/*if (hoodEncoderPresent == UNKNOWN_CONNECTED) {
			std::cout << "Unknown" << std::endl;
		}
		else if (hoodEncoderPresent == RECOGNIZED_CONNECTED) {
			std::cout << "Present" << std::endl;
		} */
		hoodEncoderPluggedIn = true;
	}
	return hoodEncoderPluggedIn;
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

bool ShooterHood::DriveShooterHoodToDesiredEncoderValue(int desiredShooterHoodEncoderValue) {
	currentShooterHoodEncoderValue = this->GetShooterHoodEncoderPosition();
	differenceBetweenDesiredAndCurrentShooterHoodEncoderValue = abs(desiredShooterHoodEncoderValue - currentShooterHoodEncoderValue);

	if (initializeDirectionOfShooterHood == false) {
		if (desiredShooterHoodEncoderValue > currentShooterHoodEncoderValue) {
			this->DriveShooterHoodMotor(-SHOOTER_HOOD_MOTOR_POWER);
			drivingShooterHoodForward = false;
		}
		else if (currentShooterHoodEncoderValue > desiredShooterHoodEncoderValue) {
			this->DriveShooterHoodMotor(SHOOTER_HOOD_MOTOR_POWER);
			drivingShooterHoodForward = true;
		}
		hoodAtDesiredEncoderValue = false;
		initializeDirectionOfShooterHood = true;
	}
	else if (initializeDirectionOfShooterHood) {
		if (drivingShooterHoodForward) {
			if (desiredShooterHoodEncoderValue == 0) {
				if (GetMaxAngleLimitSwitch() == 1) {
					this->DriveShooterHoodMotor(0.0);
					initializeDirectionOfShooterHood = false;
					hoodAtDesiredEncoderValue = true;
				}
				else {
					this->DriveShooterHoodMotor(SHOOTER_HOOD_MOTOR_POWER);
				}
			}
			else if (currentShooterHoodEncoderValue <= desiredShooterHoodEncoderValue) {
				this->DriveShooterHoodMotor(0.0);
				initializeDirectionOfShooterHood = false;
				hoodAtDesiredEncoderValue = true;
			}
			else {
				if (differenceBetweenDesiredAndCurrentShooterHoodEncoderValue <= THRESHOLD_SHOOTER_HOOD_ENCODER_VALUE_TO_DECREASE_MOTOR_POWER) {
					this->DriveShooterHoodMotor(SLOWER_SHOOTER_HOOD_MOTOR_POWER);
				}
				else {
					this->DriveShooterHoodMotor(SHOOTER_HOOD_MOTOR_POWER);
				}
			}
		}
		else if (drivingShooterHoodForward == false) {
			if (currentShooterHoodEncoderValue >= desiredShooterHoodEncoderValue) {
				this->DriveShooterHoodMotor(0.0);
				initializeDirectionOfShooterHood = false;
				hoodAtDesiredEncoderValue = true;
			}
			else {
				if (differenceBetweenDesiredAndCurrentShooterHoodEncoderValue <= THRESHOLD_SHOOTER_HOOD_ENCODER_VALUE_TO_DECREASE_MOTOR_POWER) {
					this->DriveShooterHoodMotor(-SLOWER_SHOOTER_HOOD_MOTOR_POWER);
				}
				else {
					this->DriveShooterHoodMotor(-SHOOTER_HOOD_MOTOR_POWER);
				}
			}
		}
	}
	return hoodAtDesiredEncoderValue;
}

void ShooterHood::ResetDesiredAngleOfShooterHoodFunctionVariables() {
	completeFirstForLoop = false;
	incrementCounterWhenAngleIsClose = false;
	firstAngleValueReceived = false;
	secondAngleCounter = 0;
	pastDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue = 0.0;
	currentDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue = 0.0;
	calculatedAngle = 0.0;
	startSecondForLoop = false;
	secondLoopCompleted = false;
	maxPossibleAngleFirstRound = 0.0;
	minPossibleAngleFirstRound = 0.0;
	desiredAngleValueFromSecondForLoop = 0.0;
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

	valueAngleForLoopHasToEqual = ((((-1.0) * ACCELERATION_OF_GRAVITY * totalXDistance_M)/(pow(chosenVelocityOfShooter, 2.0))) - (Y_DISTANCE_M/totalXDistance_M));


	if (completeFirstForLoop == false) {
		for (int i = 0; i <= 90; i++) {
			radiansAngleValue = ConvertDegreesToRadians(((double)i));
			valueWithInputtedAngle = (((Y_DISTANCE_M/totalXDistance_M) * cos((2.0 * radiansAngleValue))) - (sin((2.0 * radiansAngleValue))));
			currentDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue = (fabs(valueAngleForLoopHasToEqual - valueWithInputtedAngle));
			if (currentDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue < .2) {
				if (incrementCounterWhenAngleIsClose == false) {
					secondAngleCounter++;
					incrementCounterWhenAngleIsClose = true;
				}

				if (secondAngleCounter == 2) {
					firstAngleValueReceived = false;
					secondAngleCounter++;
				}

				if (firstAngleValueReceived == false) {
					pastDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue = currentDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue;
					calculatedAngle = ((double)i);
					firstAngleValueReceived = true;
				}
				else if (firstAngleValueReceived && currentDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue < pastDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue) {
					pastDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue = currentDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue;
					calculatedAngle = ((double)i);
				}
			}
			else {
				incrementCounterWhenAngleIsClose = false;
			}

			if (i == 90) {
				completeFirstForLoop = true;
			}
		}
	}
	else if (completeFirstForLoop && startSecondForLoop == false) {
		maxPossibleAngleFirstRound = (calculatedAngle + 1.0);
		minPossibleAngleFirstRound = (calculatedAngle - 1.0);

		currentDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue = 0.0;
		pastDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue = 0.0;
		calculatedAngle = 0.0;
		firstAngleValueReceived = false;
		startSecondForLoop = true;
	}
	else if (completeFirstForLoop && startSecondForLoop) {
		for (double i = minPossibleAngleFirstRound; i <= maxPossibleAngleFirstRound; i = i + .1) {
			radiansAngleValue = ConvertDegreesToRadians(((double)i));
			valueWithInputtedAngle = (((Y_DISTANCE_M/totalXDistance_M) * cos((2.0 * radiansAngleValue))) - (sin((2.0 * radiansAngleValue))));
			currentDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue = (fabs(valueAngleForLoopHasToEqual - valueWithInputtedAngle));
			if (firstAngleValueReceived == false) {
				pastDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue = currentDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue;
				calculatedAngle = ((double)i);
				firstAngleValueReceived = true;
			}
			else if (firstAngleValueReceived && currentDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue < pastDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue) {
				pastDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue = currentDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue;
				calculatedAngle = ((double)i);
			}

			if (i == maxPossibleAngleFirstRound) {
				secondLoopCompleted = true;
			}
		}
	}
	else if (completeFirstForLoop && startSecondForLoop && secondLoopCompleted) {
		desiredAngleValueFromSecondForLoop = calculatedAngle;
	}

	return desiredAngleValueFromSecondForLoop;
}

bool ShooterHood::TimeOfShotGreaterThanTimeOfMaxHeight(double desiredAngle, bool closeShot, double lidarValue_CM, int shooterAngledPosition) {
	if (shooterAngledPosition == SHOOTER_ANGLED_POSITION_ARRAY[FURTHEST_POINT_FROM_STRAIGHT_ON]) {
		lidarValue_Meters = (lidarValue_CM/100.0);
		totalXDistance_M = (lidarValue_Meters + ADDED_X_DISTANCE_MAX_INSIDE_BOILER_M);
	}
	else if (shooterAngledPosition == SHOOTER_ANGLED_POSITION_ARRAY[STRAIGHT_ON]) {
		lidarValue_Meters = (lidarValue_CM/100.0);
		totalXDistance_M = (lidarValue_Meters + ADDED_X_DISTANCE_MIN_INSIDE_BOILER_M);
	}

	radiansDesiredAngleValue = this->ConvertDegreesToRadians(desiredAngle);
	if (closeShot) {
		initialYVelocity = (SHOOTER_CLOSE_SHOT_M_PER_SEC * (sin(radiansDesiredAngleValue)));
		initialXVelocity = (SHOOTER_CLOSE_SHOT_M_PER_SEC * (cos(radiansDesiredAngleValue)));
	}
	else if (closeShot == false) {
		initialYVelocity = (SHOOTER_FAR_SHOT_M_PER_SEC * (sin(radiansDesiredAngleValue)));
		initialXVelocity = (SHOOTER_FAR_SHOT_M_PER_SEC * (cos(radiansDesiredAngleValue)));
	}

	timeOfMaxHeight = (initialYVelocity/ACCELERATION_OF_GRAVITY);
	timeOfShot = (totalXDistance_M/initialXVelocity);

	if (timeOfShot > timeOfMaxHeight) {
		timeOfShotGreaterThanTimeOfMaxHeight = true;
	}
	else if (timeOfMaxHeight >= timeOfShot) {
		timeOfShotGreaterThanTimeOfMaxHeight = false;
	}

	return timeOfShotGreaterThanTimeOfMaxHeight;
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
	if (COMPETITION_BOT) {
		maxLimitSwitchValue = shooterHoodMotor->IsRevLimitSwitchClosed();
	}
	else if (COMPETITION_BOT == false) {
		maxLimitSwitchValue = shooterHoodMotor->IsFwdLimitSwitchClosed();
	}
	return maxLimitSwitchValue;
}

int ShooterHood::GetMinAngleLimitSwitch() {
	if (COMPETITION_BOT) {
		minLimitSwitchValue = shooterHoodMotor->IsFwdLimitSwitchClosed();
	}
	else if (COMPETITION_BOT == false) {
		minLimitSwitchValue = shooterHoodMotor->IsRevLimitSwitchClosed();
	}
	return minLimitSwitchValue;
}

double ShooterHood::ConvertDegreesToRadians(double degrees) {
	return (degrees * DEGREES_TO_RADIANS_CONSTANT);
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
