#include "GetReadyForLiftHang.h"

GetReadyForLiftHang::GetReadyForLiftHang() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::liftHang.get());
}

// Called just before this Command runs the first time
void GetReadyForLiftHang::Initialize() {
	calculateDesiredEncoderPosition = false;
}

// Called repeatedly when this Command is scheduled to run
void GetReadyForLiftHang::Execute() {
	liftHangCurrentEncoderPosition = CommandBase::liftHang->GetLiftHangEncoderValue();
	currentLiftHangNumberOfRotations = CommandBase::liftHang->GetNumberOfRotationsOfLiftHang();
	lowNumberOfCurrentRotations = floor(currentLiftHangNumberOfRotations);
	liftHangDesiredEncoderValueForLowNumberOfRotations = ((((int)lowNumberOfCurrentRotations) * LiftHang::QUADRATURE_ENCODER_COUNT) + LiftHang::LIFT_HANG_READY_ENCODER_POSITION);

	if (calculateDesiredEncoderPosition == false) {
		if (liftHangCurrentEncoderPosition < liftHangDesiredEncoderValueForLowNumberOfRotations) {
			desiredEncoderPosition = liftHangDesiredEncoderValueForLowNumberOfRotations;
		}
		else if (liftHangCurrentEncoderPosition > liftHangDesiredEncoderValueForLowNumberOfRotations) {
			highNumberOfCurrentRotations = (ceil(currentLiftHangNumberOfRotations));
			desiredEncoderPosition = ((((int)highNumberOfCurrentRotations) * LiftHang::QUADRATURE_ENCODER_COUNT) + LiftHang::LIFT_HANG_READY_ENCODER_POSITION);
		}
		else if (liftHangCurrentEncoderPosition == liftHangDesiredEncoderValueForLowNumberOfRotations) {
			liftHangInDesiredPosition = true;
		}

		differenceBetweenCurrentAndDesiredLiftHangEncoderValue = (desiredEncoderPosition - currentLiftHangNumberOfRotations);

		if (differenceBetweenCurrentAndDesiredLiftHangEncoderValue < VARIABILITY_IN_DESIRED_LIFT_HANG_ENCODER_POSITION) {
			liftHangInDesiredPosition = true;
		}

		calculateDesiredEncoderPosition = true;
	}

	if (calculateDesiredEncoderPosition) {
		if (liftHangCurrentEncoderPosition > desiredEncoderPosition) {
			CommandBase::liftHang->DriveLiftHang(0.0);
			liftHangInDesiredPosition = true;
		}
		else {
			CommandBase::liftHang->DriveLiftHang(LIFT_HANG_MOTOR_POWER);
		}
	}
}

// Make this return true when this Command no longer needs to run execute()
bool GetReadyForLiftHang::IsFinished() {
	return liftHangInDesiredPosition;
}

// Called once after isFinished returns true
void GetReadyForLiftHang::End() {
	calculateDesiredEncoderPosition = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void GetReadyForLiftHang::Interrupted() {
	End();
}
