#include "ReadHoodEncoderValue.h"

ReadHoodEncoderValue::ReadHoodEncoderValue() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::shooterHood.get());
}

// Called just before this Command runs the first time
void ReadHoodEncoderValue::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void ReadHoodEncoderValue::Execute() {
	shooterHoodEncoderValue = CommandBase::shooterHood->GetShooterHoodEncoderPosition();
	//std::cout << "Shooter Hood Encoder Value: " << shooterHoodEncoderValue << std::endl;

	frc::SmartDashboard::PutNumber("Shooter Hood Encoder Value", shooterHoodEncoderValue);

	maxAngleLimitSwitchValue = CommandBase::shooterHood->GetMaxAngleLimitSwitch();
	minAngleLimitSwitchValue = CommandBase::shooterHood->GetMinAngleLimitSwitch();

	frc::SmartDashboard::PutBoolean("Shooter Hood MAX Angle Limit Switch Pressedd", maxAngleLimitSwitchValue);
	frc::SmartDashboard::PutBoolean("Shooter Hood MIN Angle Limit Switch Pressedd", minAngleLimitSwitchValue);

	//std::cout << "Max Angle Limit Switch Value: " << maxAngleLimitSwitchValue << std::endl;
	//std::cout << "Min Angle Limit Switch Value: " << minAngleLimitSwitchValue << std::endl;

	/*hoodEncoderPluggedIn = CommandBase::shooterHood->HoodEncoderPluggedIn();
	if (hoodEncoderPluggedIn == false) {
		std::cout << "Shooter Hood Encoder Disconnected" << std::endl;
	} */
}

// Make this return true when this Command no longer needs to run execute()
bool ReadHoodEncoderValue::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ReadHoodEncoderValue::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ReadHoodEncoderValue::Interrupted() {

}
