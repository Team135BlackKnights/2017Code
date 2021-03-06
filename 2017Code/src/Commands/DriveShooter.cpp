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
	initializeVoltageMode = true;
	initializePID = false;

	initializedCloseShotPIDSlot = false;
	initializedFarShotPIDSlot = false;
}

// Called repeatedly when this Command is scheduled to run
void DriveShooter::Execute() {
	/*if (CommandBase::shooter->usingShooterSubsystem) {
		resetPIDSelection = true;
		return;
	} */

	closeShotMode = CommandBase::shooter->GetSwitchBetweenFarAndCloseShotShooterRPM();

	if (closeShotMode) {
		if (initializedCloseShotPIDSlot == false) {
			CommandBase::shooter->SelectPIDProfileSlot(Shooter::CLOSE_SHOT_PID_VALUES);
			initializedCloseShotPIDSlot = true;
			initializedFarShotPIDSlot = false;
		}
		throttleValue = CommandBase::oi->GetThrottleValue(OI::MANIPULATOR_JOYSTICK);
		chosenSetpoint = CommandBase::shooter->GetCloseShotShooterRPMGivenThrottleValue(throttleValue, Shooter::SHOOTER_RANGE_OF_RPM_PART_1, Shooter::SHOOTER_SETPOINT_MINIMUM_PART_1);
		//chosenSetpoint = 2850.0;
		chosenVoltage = Shooter::DESIRED_VOLTAGE_CLOSE_SHOT;
	}
	else if (closeShotMode == false) {
		if (initializedFarShotPIDSlot == false) {
			CommandBase::shooter->SelectPIDProfileSlot(Shooter::FAR_SHOT_PID_VALUES);
			initializedFarShotPIDSlot = true;
			initializedCloseShotPIDSlot = false;
		}
		throttleValue = CommandBase::oi->GetThrottleValue(OI::MANIPULATOR_JOYSTICK);
		chosenSetpoint = CommandBase::shooter->GetCloseShotShooterRPMGivenThrottleValue(throttleValue, Shooter::SHOOTER_RANGE_OF_RPM_PART_2, Shooter::SHOOTER_SETPOINT_MINIMUM_PART_2);
		//chosenSetpoint = 3150;
		chosenVoltage = Shooter::DESIRED_VOLTAGE_FAR_SHOT;
	}

	frc::SmartDashboard::PutNumber("Desired Shooter RPM", chosenSetpoint);

	shooterMotorRPM = CommandBase::shooter->GetShooterWheelRPM();
	//shooterOutputCurrent = CommandBase::shooter->GetShooterMotorOutputCurrent();

	frc::SmartDashboard::PutNumber("Shooter Motor RPM", shooterMotorRPM);
	//frc::SmartDashboard::PutNumber("Shooter Output Current", shooterOutputCurrent);

	shooterForwardsButtonPressed = oi->GetButtonPressed(OI::MANIPULATOR_JOYSTICK, OI::TRIGGER_BUTTON);
	shooterBackwardsButtonPressed = oi->GetButtonPressed(OI::MANIPULATOR_JOYSTICK, OI::THUMB_BUTTON);

	if (this->shooterMode == ShooterPIDSelection::PID_CompetitionBot) {
		if (shooterForwardsButtonPressed) {
			if (initializePID == false) {
				CommandBase::shooter->ConfigureShooterPID();
				initializeVoltageMode = false;
				initializePID = true;
			}
			CommandBase::shooter->DriveShooterMotor(chosenSetpoint);
		}
		else if (shooterBackwardsButtonPressed) {
			if (initializeVoltageMode == false) {
				CommandBase::shooter->ConfigureShooterVoltageMode();
				initializePID = false;
				initializeVoltageMode = true;
			}
			CommandBase::shooter->DriveShooterMotor(chosenVoltage);
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
	else if (this->shooterMode == ShooterPIDSelection::PID_Ramp_Up) {
		if (shooterForwardsButtonPressed) {
			if (shooterMotorRPM < (.45 * chosenSetpoint)) {
				if (initializeVoltageMode == false) {
					CommandBase::shooter->ConfigureShooterVoltageMode();
					initializePID = false;
					initializeVoltageMode = true;
				}
				CommandBase::shooter->DriveShooterMotor(.7 * chosenVoltage);
			}
			else if (shooterMotorRPM < (.7 * chosenSetpoint)) {
				if (initializeVoltageMode == false) {
					CommandBase::shooter->ConfigureShooterVoltageMode();
					initializePID = false;
					initializeVoltageMode = true;
				}
				CommandBase::shooter->DriveShooterMotor(chosenVoltage);
			}
			else {
				if (initializePID == false) {
					CommandBase::shooter->ConfigureShooterPID();
					initializeVoltageMode = false;
					initializePID = true;
				}
				CommandBase::shooter->DriveShooterMotor(chosenSetpoint);
			}
		}
		else if (shooterBackwardsButtonPressed) {
			if (initializeVoltageMode == false) {
				CommandBase::shooter->ConfigureShooterVoltageMode();
				initializePID = false;
				initializeVoltageMode = true;
			}
			CommandBase::shooter->DriveShooterMotor(-chosenVoltage);
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

		if (shooterForwardsButtonPressed) {
			CommandBase::shooter->DriveShooterMotor(chosenVoltage);
		}
		else if (shooterBackwardsButtonPressed) {
			CommandBase::shooter->DriveShooterMotor(-chosenVoltage);
		}
		else {
			CommandBase::shooter->DriveShooterMotor(0.0);
		}
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
	initializeVoltageMode = false;
	initializePID = false;

	initializedCloseShotPIDSlot = false;
	initializedFarShotPIDSlot = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveShooter::Interrupted() {
	End();
}
