#include "GetReadyForLiftHang.h"

GetReadyForLiftHang::GetReadyForLiftHang() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::liftHang.get());
}

// Called just before this Command runs the first time
void GetReadyForLiftHang::Initialize() {
	calculateDesiredEncoderPosition = false;
	initializeDirectionOfLiftHang = false;
	liftHangAtDesiredPosition = false;
}

// Called repeatedly when this Command is scheduled to run
void GetReadyForLiftHang::Execute() {
	liftHangCurrentEncoderPosition = CommandBase::liftHang->GetLiftHangEncoderValue();
	currentLiftHangNumberOfRotations = CommandBase::liftHang->GetNumberOfRotationsOfLiftHangEncoder();

	if (calculateDesiredEncoderPosition == false) {
		intCurrentLiftHangNumberOfRotations = ((int)(round(currentLiftHangNumberOfRotations)));
		desiredEncoderPosition = ((intCurrentLiftHangNumberOfRotations * LiftHang::QUADRATURE_ENCODER_COUNT) + LiftHang::LIFT_HANG_READY_ENCODER_POSITION);
		calculateDesiredEncoderPosition = true;
	}

	if (initializeDirectionOfLiftHang == false) {
		differenceBetweenCurrentAndDesiredLiftHangEncoderValue = (abs(desiredEncoderPosition - liftHangCurrentEncoderPosition));
		if (differenceBetweenCurrentAndDesiredLiftHangEncoderValue < VARIABILITY_IN_DESIRED_LIFT_HANG_ENCODER_POSITION) {
			liftHangAtDesiredPosition = true;
		}
		else if (liftHangCurrentEncoderPosition > desiredEncoderPosition) {
			driveLiftHangForward = false;
		}
		else if (liftHangCurrentEncoderPosition < desiredEncoderPosition) {
			driveLiftHangForward = true;
		}
		initializeDirectionOfLiftHang = true;
	}

	if (initializeDirectionOfLiftHang) {
		if (driveLiftHangForward) {
			if (liftHangCurrentEncoderPosition >= desiredEncoderPosition) {
				CommandBase::liftHang->DriveLiftHang(0.0);
				liftHangAtDesiredPosition = true;
			}
			else {
				CommandBase::liftHang->DriveLiftHang(LIFT_HANG_MOTOR_POWER);
			}
		}
		else if (driveLiftHangForward == false) {
			if (liftHangCurrentEncoderPosition <= desiredEncoderPosition) {
				CommandBase::liftHang->DriveLiftHang(0.0);
				liftHangAtDesiredPosition = true;
			}
			else {
				CommandBase::liftHang->DriveLiftHang(-LIFT_HANG_MOTOR_POWER);
			}
		}
	}
}

// Make this return true when this Command no longer needs to run execute()
bool GetReadyForLiftHang::IsFinished() {
	return liftHangAtDesiredPosition;
}

// Called once after isFinished returns true
void GetReadyForLiftHang::End() {
	calculateDesiredEncoderPosition = false;
	initializeDirectionOfLiftHang = false;
	liftHangAtDesiredPosition = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void GetReadyForLiftHang::Interrupted() {
	End();
}
