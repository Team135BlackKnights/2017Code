#include "DriveLiftHang.h"

DriveLiftHang::DriveLiftHang() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::liftHang.get());
	Requires(CommandBase::pdp.get());
}

// Called just before this Command runs the first time
void DriveLiftHang::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void DriveLiftHang::Execute() {
	CommandBase::liftHang->DriveLiftHang(LIFT_HANG_MOTOR_SPEED);

	liftHangCurrentValue = CommandBase::pdp->GetCurrentOfPDPPort(PDP::HANGING_MOTOR_PDP_PORT);
	frc::SmartDashboard::PutNumber("Lift Hang Motor Current", liftHangCurrentValue);
}

// Make this return true when this Command no longer needs to run execute()
bool DriveLiftHang::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void DriveLiftHang::End() {
	CommandBase::liftHang->DriveLiftHang(0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveLiftHang::Interrupted() {
	End();
}
