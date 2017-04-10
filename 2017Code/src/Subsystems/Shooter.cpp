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

void Shooter::ConfigureShooterMotorEncoder(bool competitionBot) {
	shooterMotor->ConfigEncoderCodesPerRev(SHOOTER_ENCODER_COUNT);
	shooterMotor->SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Relative);
	shooterMotor->SetSensorDirection(REVERSE_SHOOTER_ENCODER_DIRECTION);
	shooterMotor->SetPosition(0.0);
	shooterMotor->SetNominalClosedLoopVoltage(SHOOTER_MAX_VOLTAGE);

	if (competitionBot) {
		closeShotKp = CB_CLOSE_SHOT_Kp;
		closeShotKi = CB_CLOSE_SHOT_Ki;
		closeShotKd = CB_CLOSE_SHOT_Kd;
		closeShotKf = CB_CLOSE_SHOT_Kf;
		closeShotPositivePeakVoltage = CB_CLOSE_SHOT_POSITIVE_PEAK_VOLTAGE;
		closeShotNegativePeakVoltage = CB_CLOSE_SHOT_NEGATIVE_PEAK_VOLTAGE;

		farShotKp = CB_FAR_SHOT_Kp;
		farShotKi = CB_FAR_SHOT_Ki;
		farShotKd = CB_FAR_SHOT_Kd;
		farShotKf = CB_FAR_SHOT_Kf;
		farShotPositivePeakVoltage = CB_FAR_SHOT_POSITIVE_PEAK_VOLTAGE;
		farShotNegativePeakVoltage = CB_FAR_SHOT_NEGATIVE_PEAK_VOLTAGE;
	}
	else if (competitionBot == false) {
		closeShotKp = PB_CLOSE_SHOT_Kp;
		closeShotKi = PB_CLOSE_SHOT_Ki;
		closeShotKd = PB_CLOSE_SHOT_Kd;
		closeShotKf = PB_CLOSE_SHOT_Kf;
		closeShotPositivePeakVoltage = PB_CLOSE_SHOT_POSITIVE_PEAK_VOLTAGE;
		closeShotNegativePeakVoltage = PB_CLOSE_SHOT_NEGATIVE_PEAK_VOLTAGE;

		farShotKp = PB_FAR_SHOT_Kp;
		farShotKi = PB_FAR_SHOT_Ki;
		farShotKd = PB_FAR_SHOT_Kd;
		farShotKf = PB_FAR_SHOT_Kf;
		farShotPositivePeakVoltage = PB_FAR_SHOT_POSITIVE_PEAK_VOLTAGE;
		farShotNegativePeakVoltage = PB_FAR_SHOT_NEGATIVE_PEAK_VOLTAGE;
	}

	this->SelectPIDProfileSlot(CLOSE_SHOT_PID_VALUES);
	shooterMotor->ConfigPeakOutputVoltage(closeShotPositivePeakVoltage, closeShotNegativePeakVoltage);
	shooterMotor->ConfigNominalOutputVoltage(0.0, -0.0);
	shooterMotor->SetP(closeShotKp);
	shooterMotor->SetI(closeShotKi);
	shooterMotor->SetD(closeShotKd);
	shooterMotor->SetF(closeShotKf);

	this->SelectPIDProfileSlot(FAR_SHOT_PID_VALUES);
	shooterMotor->ConfigPeakOutputVoltage(farShotPositivePeakVoltage, farShotNegativePeakVoltage);
	shooterMotor->ConfigNominalOutputVoltage(0.0, -0.0);
	shooterMotor->SetP(farShotKp);
	shooterMotor->SetI(farShotKi);
	shooterMotor->SetD(farShotKd);
	shooterMotor->SetF(farShotKf);
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

void Shooter::ShooterUpToSpeed(bool shooterUpToSpeed) {
	this->shooterUpToSpeed = shooterUpToSpeed;
}

bool Shooter::GetShooterUpToSpeed() {
	return (this->shooterUpToSpeed);
}

double Shooter::GetCloseShotShooterRPMGivenThrottleValue(double throttleValue) {
	convertedThrottleValue = (throttleValue + 1.0);
	desiredCloseShotShooterSetpoint = (((convertedThrottleValue / 2.0) * RANGE_OF_CLOSE_SHOT_SHOOTER_RPM) + SHOOTER_SETPOINT_MIN_RPM_CLOSE_SHOT);
	return desiredCloseShotShooterSetpoint;
}

void Shooter::SetSwitchFarAndCloseShotShooterRPM(bool closeShot) {
	this->closeShot = closeShot;
}

bool Shooter::GetSwitchBetweenFarAndCloseShotShooterRPM() {
	return this->closeShot;
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
