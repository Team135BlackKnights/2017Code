#include "Shooter.h"
#include "../RobotMap.h"
#include "Commands/DriveShooter.h"

Shooter::Shooter() : Subsystem("Shooter") {

}

void Shooter::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
	SetDefaultCommand(new DriveShooter(SHOOTER_PID_SELECTION));
}

void Shooter::InitializeShooterMotor(bool competitionBot) {
	if (competitionBot) {
		shooterMotor = new CANTalon(CB_SHOOTER_MOTOR_TALON_ID);
		shooterMotor->SetInverted(CB_SHOOTER_MOTOR_INVERTED);
	}
	else if (competitionBot == false) {
		shooterMotor = new CANTalon(PB_SHOOTER_MOTOR_TALON_ID);
		shooterMotor->SetInverted(PB_SHOOTER_MOTOR_INVERTED);
	}
}

void Shooter::DriveShooterMotor(double motorPower) {
	shooterMotor->Set(motorPower);
}

void Shooter::ConfigureShooterMotorEncoder() {
	shooterMotor->ConfigEncoderCodesPerRev(SHOOTER_ENCODER_COUNT);
	shooterMotor->SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Relative);
	shooterMotor->SetSensorDirection(REVERSE_SHOOTER_ENCODER_DIRECTION);
	shooterMotor->SetPosition(0.0);
	shooterMotor->SetNominalClosedLoopVoltage(SHOOTER_MAX_VOLTAGE);

	this->SelectPIDProfileSlot(CLOSE_SHOT_PID_VALUES);
	shooterMotor->ConfigPeakOutputVoltage(9.0, -6.0);
	shooterMotor->ConfigNominalOutputVoltage(0.0, -0.0);
	shooterMotor->SetP(1.0);
	shooterMotor->SetI(0);
	shooterMotor->SetD(9.5);
	shooterMotor->SetF(.037);

	this->SelectPIDProfileSlot(FAR_SHOT_PID_VALUES);
	shooterMotor->ConfigPeakOutputVoltage(10.0, -6.0);
	shooterMotor->ConfigNominalOutputVoltage(0.0, -0.0);
	shooterMotor->SetP(1.0);
	shooterMotor->SetI(0.0);
	shooterMotor->SetD(0.0);
	shooterMotor->SetF(.0359);
}

void Shooter::ConfigureShooterVoltageMode() {
	shooterMotor->SetControlMode(CANTalon::ControlMode::kVoltage);
	shooterMotor->Set(0.0);
}

void Shooter::ConfigureShooterPID() {
	shooterMotor->SetControlMode(CANTalon::ControlMode::kSpeed);
	shooterMotor->Set(0.0);
}

int Shooter::GetShooterWheelRPM() {
	return shooterMotor->GetSpeed();
}

int Shooter::GetShooterWheelNUPer100Ms() {
	return shooterMotor->GetEncVel();
}

double Shooter::GetShooterVoltage() {
	return shooterMotor->GetOutputVoltage();
}

double Shooter::GetShooterMotorOutputCurrent() {
	return shooterMotor->GetOutputCurrent();
}

void Shooter::ZeroAccumulatedError() {
	shooterMotor->ClearIaccum();
}

void Shooter::SelectPIDProfileSlot(int profileSlot) {
	shooterMotor->SelectProfileSlot(profileSlot);
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
