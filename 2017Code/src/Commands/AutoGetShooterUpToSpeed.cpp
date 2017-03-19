#include "AutoGetShooterUpToSpeed.h"

AutoGetShooterUpToSpeed::AutoGetShooterUpToSpeed(double desiredShooterRPM) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::shooter.get());

	this->desiredShooterRPM = desiredShooterRPM;

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
}

// Called repeatedly when this Command is scheduled to run
void AutoGetShooterUpToSpeed::Execute() {
	if (initializePID == false) {
		CommandBase::shooter->ConfigureShooterPID();
		initializeVoltageMode = false;
		initializePID = true;
	}

	timerValue = timer->Get();

	CommandBase::shooter->DriveShooterMotor(this->desiredShooterRPM);

	currentShooterRPMValue = CommandBase::shooter->GetShooterWheelRPM();
	//std::cout << "Shooter RPM: " << currentShooterRPMValue << std::endl;
	if (((currentShooterRPMValue >= this->desiredShooterRPM) && shooterUpToSpeed == false) || maintainShooterRPM) {
		if (startTimer == false) {
			timer->Reset();
			timer->Start();
			startTimer = true;
			maintainShooterRPM = true;
		}

		if (startTimer && (timerValue < TIME_TO_WAIT_FOR_SHOOTER_TO_MAINTAIN_VELOCITY)) {
			shooterUpToSpeed = false;
		}
		else if (startTimer && (timerValue >= TIME_TO_WAIT_FOR_SHOOTER_TO_MAINTAIN_VELOCITY)) {
			maintainShooterRPM = false;
			shooterUpToSpeed = true;
		}
		CommandBase::shooter->ShooterUpToSpeed(shooterUpToSpeed);
	}

	frc::SmartDashboard::PutNumber("Shooter Autonomous RPM", currentShooterRPMValue);
}

// Make this return true when this Command no longer needs to run execute()
bool AutoGetShooterUpToSpeed::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void AutoGetShooterUpToSpeed::End() {
	CommandBase::shooter->ConfigureShooterVoltageMode();
	initializePID = false;
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
