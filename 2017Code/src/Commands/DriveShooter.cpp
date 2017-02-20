#include "DriveShooter.h"

DriveShooter::DriveShooter() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::shooter.get());
}

// Called just before this Command runs the first time
void DriveShooter::Initialize() {
	setpointRPM = Preferences::GetInstance()->GetDouble("Shooter PID Setpoint", 2400.0);
	CommandBase::shooter->ConfigureShooterVoltageMode();
	CommandBase::shooter->ConfigureShooterMotorEncoder();
}

// Called repeatedly when this Command is scheduled to run
void DriveShooter::Execute() {
	shooterForwardsButtonPressed = oi->GetButtonPressed(OI::MANIPULATOR_JOYSTICK, OI::TRIGGER_BUTTON);
	shooterBackwardsButtonPressed = oi->GetButtonPressed(OI::MANIPULATOR_JOYSTICK, OI::THUMB_BUTTON);

	shooterMotorRPM = CommandBase::shooter->GetShooterWheelRPM();
	shooterMotorNUPer100Ms = CommandBase::shooter->GetShooterWheelNUPer100Ms();

	if (shooterForwardsButtonPressed) {
		if (shooterMotorRPM < (MAX_PERCENT_OF_SETPOINT_TO_RAMP_VOLTAGE * setpointRPM)) {
			if (initializeVoltageMode == false) {
				CommandBase::shooter->ConfigureShooterVoltageMode();
				CommandBase::shooter->DriveShooterMotor(RAMP_UP_OUTPUT_VOLTAGE);
				initializeVoltageMode = true;
				initializePID = false;
			}
			else if (initializeVoltageMode) {
				CommandBase::shooter->DriveShooterMotor(RAMP_UP_OUTPUT_VOLTAGE);
			}
		}
		else {
			if (initializePID == false) {
				CommandBase::shooter->ConfigureShooterPID();
				CommandBase::shooter->DriveShooterMotor(setpointRPM);
				initializePID = true;
				initializeVoltageMode = false;
			}
			else if (initializePID) {
				CommandBase::shooter->DriveShooterMotor(setpointRPM);
			}
		}
	}
	else if (shooterBackwardsButtonPressed) {
		if (initializeVoltageMode == false) {
			CommandBase::shooter->ConfigureShooterVoltageMode();
			CommandBase::shooter->DriveShooterMotor(SHOOTER_BACKWARDS_MOTOR_VOLTAGE);
			initializeVoltageMode = true;
			initializePID = false;
		}
		else if (initializeVoltageMode) {
			CommandBase::shooter->DriveShooterMotor(SHOOTER_BACKWARDS_MOTOR_VOLTAGE);
		}
	}
	else {
		CommandBase::shooter->ConfigureShooterVoltageMode();
		CommandBase::shooter->DriveShooterMotor(0.0);
		initializeVoltageMode = true;
		initializePID = false;
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveShooter::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void DriveShooter::End() {
	CommandBase::shooter->ConfigureShooterVoltageMode();
	CommandBase::shooter->DriveShooterMotor(0.0);
	initializeVoltageMode = true;
	initializePID = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveShooter::Interrupted() {
	End();
}
