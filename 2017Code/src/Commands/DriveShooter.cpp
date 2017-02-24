#include "DriveShooter.h"

DriveShooter::DriveShooter(ShooterPIDSelection shooterMode) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::shooter.get());

	this->shooterMode = shooterMode;
}

// Called just before this Command runs the first time
void DriveShooter::Initialize() {
	setpointRPM = Preferences::GetInstance()->GetDouble("Shooter PID Setpoint", 2400.0);
	desiredShooterVoltage = Preferences::GetInstance()->GetDouble("Shooter Voltage", 8.0);
	CommandBase::shooter->ConfigureShooterVoltageMode();
	//CommandBase::shooter->ConfigureShooterPID();
	initializeVoltageMode = false;
	initializePID = true;
	zeroAccumulatedError = false;
}

// Called repeatedly when this Command is scheduled to run
void DriveShooter::Execute() {
	shooterMotorRPM = CommandBase::shooter->GetShooterWheelRPM();
	shooterMotorNUPer100Ms = CommandBase::shooter->GetShooterWheelNUPer100Ms();
	shooterOutputCurrent = CommandBase::shooter->GetShooterMotorOutputCurrent();

	std::cout << "Shooter RPM: " << shooterMotorRPM << std::endl;
	std::cout << "Shooter NU Per 100ms: " << shooterMotorNUPer100Ms << std::endl;

	frc::SmartDashboard::PutNumber("Shooter Motor RPM", shooterMotorRPM);
	frc::SmartDashboard::PutNumber("Shooter Output Current", shooterOutputCurrent);

	shooterForwardsButtonPressed = oi->GetButtonPressed(OI::MANIPULATOR_JOYSTICK, OI::TRIGGER_BUTTON);
	shooterBackwardsButtonPressed = oi->GetButtonPressed(OI::MANIPULATOR_JOYSTICK, OI::THUMB_BUTTON);

	if (this->shooterMode == ShooterPIDSelection::PID_CompetitionBot) {
		if (shooterForwardsButtonPressed) {
			if (initializePID == false) {
				CommandBase::shooter->ConfigureShooterPID();
				initializeVoltageMode = false;
				initializePID = true;
			}
			CommandBase::shooter->DriveShooterMotor(setpointRPM);
		}
		else if (shooterBackwardsButtonPressed) {
			if (initializeVoltageMode == false) {
				CommandBase::shooter->ConfigureShooterVoltageMode();
				initializePID = false;
				initializeVoltageMode = true;
				zeroAccumulatedError = false;
			}
			CommandBase::shooter->DriveShooterMotor(SHOOTER_BACKWARDS_VOLTAGE);
		}
		else {
			if (initializeVoltageMode == false) {
				CommandBase::shooter->ConfigureShooterVoltageMode();
				initializePID = false;
				initializeVoltageMode = true;
				zeroAccumulatedError = false;
			}
			CommandBase::shooter->DriveShooterMotor(0.0);
		}

		/*if (shooterMotorRPM >= setpointRPM) {
			if (zeroAccumulatedError == false) {
				CommandBase::shooter->ZeroAccumulatedError();
				zeroAccumulatedError = true;
			}
		} */
	}
	else if (this->shooterMode == ShooterPIDSelection::PID_PracticeBot) {
		if (shooterForwardsButtonPressed) {
			if (shooterMotorRPM < (.4 * setpointRPM)) {
				if (initializeVoltageMode == false) {
					CommandBase::shooter->ConfigureShooterVoltageMode();
					initializePID = false;
					initializeVoltageMode = true;
				}
				CommandBase::shooter->DriveShooterMotor(.65 * desiredShooterVoltage);
			}
			else if (shooterMotorRPM < (.65 * setpointRPM)) {
				if (initializeVoltageMode == false) {
					CommandBase::shooter->ConfigureShooterVoltageMode();
					initializePID = false;
					initializeVoltageMode = true;
				}
				CommandBase::shooter->DriveShooterMotor(desiredShooterVoltage);
			}
			else {
				if (initializePID == false) {
					CommandBase::shooter->ConfigureShooterPID();
					initializeVoltageMode = false;
					initializePID = true;
				}
				CommandBase::shooter->DriveShooterMotor(setpointRPM);
			}
		}
		else if (shooterBackwardsButtonPressed) {
			if (initializeVoltageMode == false) {
				CommandBase::shooter->ConfigureShooterVoltageMode();
				initializePID = false;
				initializeVoltageMode = true;
			}
			CommandBase::shooter->DriveShooterMotor(-desiredShooterVoltage);
		}
		else {
			if (initializeVoltageMode == false) {
				CommandBase::shooter->ConfigureShooterVoltageMode();
				initializePID = false;
				initializeVoltageMode = true;
			}
			CommandBase::shooter->DriveShooterMotor(0.0);
		}
	}
	else if (this->shooterMode == ShooterPIDSelection::Voltage) {
		if (initializeVoltageMode == false) {
			CommandBase::shooter->ConfigureShooterVoltageMode();
			initializePID = false;
			initializeVoltageMode = true;
		}
		CommandBase::shooter->DriveShooterMotor(desiredShooterVoltage);
	}

	std::cout << "Shooter RPM: " << shooterMotorRPM << std::endl;
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
	zeroAccumulatedError = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveShooter::Interrupted() {
	End();
}
