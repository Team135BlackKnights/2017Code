#include "AutoGetShooterUpToSpeed.h"

AutoGetShooterUpToSpeed::AutoGetShooterUpToSpeed(double desiredShooterRPM, bool closeShotPID) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::shooter.get());

	this->desiredShooterRPM = desiredShooterRPM;
	this->closeShotPID = closeShotPID;

	timer = new frc::Timer();
}

// Called just before this Command runs the first time
void AutoGetShooterUpToSpeed::Initialize() {
	CommandBase::shooter->ConfigureShooterPID();
	initializeVoltageMode = false;
	initializePID = true;
	maintainShooterRPM = false;
	shooterUpToSpeed = false;
	timer->Reset();
	timer->Start();
	startTimer = true;

	initializePIDSlot = false;
}

// Called repeatedly when this Command is scheduled to run
void AutoGetShooterUpToSpeed::Execute() {
	if (initializePID == false) {
		CommandBase::shooter->ConfigureShooterPID();
		initializeVoltageMode = false;
		initializePID = true;
	}

	if (initializePIDSlot == false) {
		if (this->closeShotPID) {
			CommandBase::shooter->SelectPIDProfileSlot(Shooter::CLOSE_SHOT_PID_VALUES);
		}
		else if (this->closeShotPID == false) {
			CommandBase::shooter->SelectPIDProfileSlot(Shooter::FAR_SHOT_PID_VALUES);
		}
		initializePIDSlot = true;
	}

	timerValue = timer->Get();

	CommandBase::shooter->DriveShooterMotor(this->desiredShooterRPM);

	currentShooterRPMValue = CommandBase::shooter->GetShooterWheelRPM();
	if (((currentShooterRPMValue >= this->desiredShooterRPM) && shooterUpToSpeed == false) || maintainShooterRPM) {
		if (startTimer == false) {
			timer->Reset();
			timer->Start();
			startTimer = true;
			maintainShooterRPM = true;
		}

		if (startTimer && (timerValue < TIME_TO_WAIT_FOR_SHOOTER_TO_MAINTAIN_VELOCITY)) {
			shooterUpToSpeed = false;
			maintainShooterRPM = true;
		}
		else if (startTimer && (timerValue >= TIME_TO_WAIT_FOR_SHOOTER_TO_MAINTAIN_VELOCITY)) {
			maintainShooterRPM = false;
			shooterUpToSpeed = true;
		}
		CommandBase::shooter->ShooterUpToSpeed(shooterUpToSpeed);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool AutoGetShooterUpToSpeed::IsFinished() {
	return CommandBase::shooter->stopShooterFromDriving;
}

// Called once after isFinished returns true
void AutoGetShooterUpToSpeed::End() {
	CommandBase::shooter->ConfigureShooterVoltageMode();
	initializePID = false;
	initializePIDSlot = true;
	initializeVoltageMode = true;
	CommandBase::shooter->DriveShooterMotor(0.0);
	maintainShooterRPM = false;
	shooterUpToSpeed = false;
	CommandBase::shooter->ShooterUpToSpeed(shooterUpToSpeed);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutoGetShooterUpToSpeed::Interrupted() {
	End();
}
