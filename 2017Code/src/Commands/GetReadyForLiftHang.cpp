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
	liftHangCurrentEncoderPosition = CommandBase::liftHang->GetLiftHangEncoderRawValue();
	currentLiftHangNumberOfRotations = CommandBase::liftHang->GetNumberOfRotationsOfLiftHang();
	lowNumberOfCurrentRotations = floor(currentLiftHangNumberOfRotations);
	liftHangDesiredEncoderValueForLowNumberOfRotations = ((((int)lowNumberOfCurrentRotations) * LiftHang::QUADRATURE_ENCODER_COUNT) + LiftHang::LIFT_HANG_READY_ENCODER_POSITION);

	differenceBetweenCurrentAndDesiredLiftHangEncoderValue = (fabs(liftHangDesiredEncoderValueForLowNumberOfRotations - liftHangCurrentEncoderPosition));
	//std::cout << "Difference Between CUrrent and Deisred: " << differenceBetweenCurrentAndDesiredLiftHangEncoderValue << std::endl;

	if (calculateDesiredEncoderPosition == false) {
		if (differenceBetweenCurrentAndDesiredLiftHangEncoderValue < VARIABILITY_IN_DESIRED_LIFT_HANG_ENCODER_POSITION) {
			liftHangInDesiredPosition = true;
		}
		else if (liftHangCurrentEncoderPosition < liftHangDesiredEncoderValueForLowNumberOfRotations) {
			desiredEncoderPosition = liftHangDesiredEncoderValueForLowNumberOfRotations;
		}
		else if (liftHangCurrentEncoderPosition > liftHangDesiredEncoderValueForLowNumberOfRotations) {
			highNumberOfCurrentRotations = (ceil(currentLiftHangNumberOfRotations));
			desiredEncoderPosition = ((((int)highNumberOfCurrentRotations) * LiftHang::QUADRATURE_ENCODER_COUNT) + LiftHang::LIFT_HANG_READY_ENCODER_POSITION);
		}

		//std::cout << "Desired Lift Hang Encoder Position: " << desiredEncoderPosition << std::endl;

		calculateDesiredEncoderPosition = true;
	}

	if (calculateDesiredEncoderPosition) {
		if (liftHangCurrentEncoderPosition > desiredEncoderPosition) {
			CommandBase::liftHang->DriveLiftHang(0.0);
			liftHangInDesiredPosition = true;
			calculateDesiredEncoderPosition = false;
		}
		else {
			CommandBase::liftHang->DriveLiftHang(LIFT_HANG_MOTOR_POWER);
			liftHangInDesiredPosition= false;
		}
	}
}

// Make this return true when this Command no longer needs to run execute()
bool GetReadyForLiftHang::IsFinished() {
	return liftHangInDesiredPosition;
}

// Called once after isFinished returns true
void GetReadyForLiftHang::End() {
	CommandBase::liftHang->DriveLiftHang(0.0);
	calculateDesiredEncoderPosition = false;
	liftHangInDesiredPosition= false;
	//std::cout << "COmmand Ended" << std::endl;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void GetReadyForLiftHang::Interrupted() {
	End();
}
