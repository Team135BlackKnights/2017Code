#include "AutoLiftHang.h"

AutoLiftHang::AutoLiftHang() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::liftHang.get());
	Requires(CommandBase::pdp.get());
}

// Called just before this Command runs the first time
void AutoLiftHang::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void AutoLiftHang::Execute() {
	liftHangCurrentValue = CommandBase::pdp->GetCurrentOfPDPPort(PDP::HANGING_MOTOR_PDP_PORT);
}

// Make this return true when this Command no longer needs to run execute()
bool AutoLiftHang::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void AutoLiftHang::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutoLiftHang::Interrupted() {

}
