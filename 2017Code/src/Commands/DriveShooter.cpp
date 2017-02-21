#include "DriveShooter.h"

DriveShooter::DriveShooter() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::shooter.get());
}

// Called just before this Command runs the first time
void DriveShooter::Initialize() {
	setpointRPM = Preferences::GetInstance()->GetDouble("Shooter PID Setpoint", 2400.0);
	desiredShooterVoltage = Preferences::GetInstance()->GetDouble("Shooter Voltage", 8.0);
	CommandBase::shooter->ConfigureShooterVoltageMode();
	//CommandBase::shooter->ConfigureShooterPID();
	initializeVoltageMode = false;
	initializePID = false;
}

// Called repeatedly when this Command is scheduled to run
void DriveShooter::Execute() {
	shooterForwardsButtonPressed = oi->GetButtonPressed(OI::MANIPULATOR_JOYSTICK, OI::TRIGGER_BUTTON);
	shooterBackwardsButtonPressed = oi->GetButtonPressed(OI::MANIPULATOR_JOYSTICK, OI::THUMB_BUTTON);

	shooterMotorRPM = CommandBase::shooter->GetShooterWheelRPM();
	shooterMotorNUPer100Ms = CommandBase::shooter->GetShooterWheelNUPer100Ms();

	std::cout << "Shooter RPM: " << shooterMotorRPM << std::endl;
	std::cout << "Shooter NU Per 100ms: " << shooterMotorNUPer100Ms << std::endl;
	frc::SmartDashboard::PutNumber("Shooter Motor RPM", shooterMotorRPM);

	if (shooterForwardsButtonPressed) {
		CommandBase::shooter->DriveShooterMotor(desiredShooterVoltage);
	}
	else if (shooterBackwardsButtonPressed) {
		CommandBase::shooter->DriveShooterMotor(-desiredShooterVoltage);
	}
	else {
		CommandBase::shooter->DriveShooterMotor(0.0);
	}

	shooterOutputCurrent = CommandBase::shooter->GetShooterMotorOutputCurrent();

	frc::SmartDashboard::PutNumber("Shooter Output Current", shooterOutputCurrent);

	/*if (shooterForwardsButtonPressed) {
		if (initializePID == false) {
			CommandBase::shooter->ConfigureShooterPID();
			CommandBase::shooter->DriveShooterMotor(setpointRPM);
			initializeVoltageMode = false;
			initializePID = true;
		}
		else {
			CommandBase::shooter->DriveShooterMotor(setpointRPM);
		}
	}
	else if (shooterBackwardsButtonPressed) {
		if (initializeVoltageMode == false) {
			CommandBase::shooter->ConfigureShooterVoltageMode();
			CommandBase::shooter->DriveShooterMotor(SHOOTER_BACKWARDS_VOLTAGE);
			initializePID = false;
			initializeVoltageMode = true;
		}
		else {
			CommandBase::shooter->DriveShooterMotor(SHOOTER_BACKWARDS_VOLTAGE);
		}
	}
	else {
		if (initializeVoltageMode == false) {
			CommandBase::shooter->ConfigureShooterVoltageMode();
			CommandBase::shooter->DriveShooterMotor(0.0);
			initializePID = false;
			initializeVoltageMode = true;
		}
		else {
			CommandBase::shooter->DriveShooterMotor(0.0);
		}
	} */
}

// Make this return true when this Command no longer needs to run execute()
bool DriveShooter::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void DriveShooter::End() {
	CommandBase::shooter->ConfigureShooterVoltageMode();
	CommandBase::shooter->DriveShooterMotor(0.0);
	initializeVoltageMode = false;
	initializePID = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveShooter::Interrupted() {
	End();
}
