#include "ShooterHood.h"
#include "../RobotMap.h"

ShooterHood::ShooterHood() : Subsystem("ShooterHood") {

}

void ShooterHood::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
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
	/*if (this->GetMinAngleLimitSwitch() == 1) {
		this->ZeroShooterHoodEncoder();
		shooterHoodMotorPower = fmin(0.0, motorPower);
	}
	else if (this->GetMaxAngleLimitSwitch() == 1) {
		this->SetShooterHoodEncoder(MAX_ENCODER_VALUE);
		shooterHoodMotorPower = fmax(0.0, motorPower);
	}
	else {
		shooterHoodMotorPower = motorPower;
	} */

	shooterHoodMotorPower = motorPower;
	shooterHoodMotor->Set(shooterHoodMotorPower);
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

int ShooterHood::GetMaxAngleLimitSwitch() {
	return shooterHoodMotor->IsRevLimitSwitchClosed();
}

int ShooterHood::GetMinAngleLimitSwitch() {
	return shooterHoodMotor->IsFwdLimitSwitchClosed();
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
