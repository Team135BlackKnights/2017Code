#include "DriveTrain.h"
#include "../RobotMap.h"
#include "Commands/DriveWithJoysticks.h"

DriveTrain::DriveTrain() : Subsystem("DriveTrain") {

}

void DriveTrain::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
	SetDefaultCommand(new DriveWithJoysticks());
}

void DriveTrain::InitializeDriveTrainMotors(bool competitionBot) {
	if (competitionBot) {
		driveTrainMotors[FRONT_LEFT] = new CANTalon(CB_FRONT_LEFT_TALON_ID);
		driveTrainMotors[REAR_LEFT] = new CANTalon(CB_REAR_LEFT_TALON_ID);
		driveTrainMotors[FRONT_RIGHT] = new CANTalon(CB_FRONT_RIGHT_TALON_ID);
		driveTrainMotors[REAR_RIGHT] = new CANTalon(CB_REAR_RIGHT_TALON_ID);
	}
	else if (competitionBot == false) {
		driveTrainMotors[FRONT_LEFT] = new CANTalon(PB_FRONT_LEFT_TALON_ID);
		driveTrainMotors[REAR_LEFT] = new CANTalon(PB_REAR_LEFT_TALON_ID);
		driveTrainMotors[FRONT_RIGHT] = new CANTalon(PB_FRONT_RIGHT_TALON_ID);
		driveTrainMotors[REAR_RIGHT] = new CANTalon(PB_REAR_RIGHT_TALON_ID);
	}
	chassis = new frc::RobotDrive(driveTrainMotors[FRONT_LEFT], driveTrainMotors[REAR_LEFT], driveTrainMotors[FRONT_RIGHT], driveTrainMotors[REAR_RIGHT]);
	chassis->SetSafetyEnabled(false);
	//navX = new AHRS(SerialPort::Port::kUSB);
}

void DriveTrain::DriveTank(double leftMotorPower, double rightMotorPower) {
	chassis->TankDrive(leftMotorPower, rightMotorPower);
}

void DriveTrain::RotateTank(double motorPower, bool turnRight) {
	if (turnRight) {
		this->DriveTank(motorPower, -motorPower);
	}
	else if (turnRight == false) {
		this->DriveTank(-motorPower, motorPower);
	}
}

void DriveTrain::ConfigureDriveTrainEncoders() {
	driveTrainMotors[LEFT_SIDE_ENCODER]->ConfigEncoderCodesPerRev(ENCODER_COUNTS);
	driveTrainMotors[RIGHT_SIDE_ENCODER]->ConfigEncoderCodesPerRev(ENCODER_COUNTS);

	driveTrainMotors[LEFT_SIDE_ENCODER]->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
	driveTrainMotors[RIGHT_SIDE_ENCODER]->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);

	driveTrainMotors[LEFT_SIDE_ENCODER]->SetStatusFrameRateMs(CANTalon::StatusFrameRate::StatusFrameRateQuadEncoder, 10);
	driveTrainMotors[RIGHT_SIDE_ENCODER]->SetStatusFrameRateMs(CANTalon::StatusFrameRate::StatusFrameRateQuadEncoder, 10);
}

void DriveTrain::ZeroDriveTrainEncoder(int motorEncoderPort) {
	driveTrainMotors[motorEncoderPort]->SetEncPosition(0);
}

int DriveTrain::GetEncoderPosition(int motorEncoderPort) {
	return driveTrainMotors[motorEncoderPort]->GetEncPosition();
}

double DriveTrain::GetDistance(int motorEncoderPort) {
	encoderValue = this->GetEncoderPosition(motorEncoderPort);
	distanceTraveled = (encoderValue * ENCODER_COUNT_TO_DISTANCE_CONSTANT);
	return distanceTraveled;
}

void DriveTrain::InitializeDriveTrainPID() {
	navX = new AHRS(SerialPort::Port::kUSB);
	turnController = new PIDController(kP, kI, kD, kF, navX, this);
	turnController->SetInputRange(-90.0f,  90.0f);
	turnController->SetOutputRange(-1.0, 1.0);
	turnController->SetAbsoluteTolerance(kToleranceDegrees);
	turnController->SetContinuous(true);
	turnController->Disable();
}

double DriveTrain::GetNavXAngle() {
	return navX->GetAngle();
}

void DriveTrain::ZeroNavXAngle() {
	navX->ZeroYaw();
}


void DriveTrain::TurnPIDEnable(double angleToTurn)
{
	this->ZeroNavXAngle();
	turnController->SetSetpoint(angleToTurn);
	turnController->Enable();
}

void DriveTrain::TurnPIDDisable()
{
	turnController->Disable();
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
