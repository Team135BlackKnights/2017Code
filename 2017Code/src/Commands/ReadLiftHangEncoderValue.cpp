#include "ReadLiftHangEncoderValue.h"

ReadLiftHangEncoderValue::ReadLiftHangEncoderValue() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::liftHang.get());
}

// Called just before this Command runs the first time
void ReadLiftHangEncoderValue::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void ReadLiftHangEncoderValue::Execute() {
	liftHangRawEncoderValue = CommandBase::liftHang->GetLiftHangEncoderRawValue();

	frc::SmartDashboard::PutNumber("Lift Hang Encoder Valueee", liftHangRawEncoderValue);
}

// Make this return true when this Command no longer needs to run execute()
bool ReadLiftHangEncoderValue::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ReadLiftHangEncoderValue::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ReadLiftHangEncoderValue::Interrupted() {

}
