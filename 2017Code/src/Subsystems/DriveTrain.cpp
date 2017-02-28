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
	navX = new AHRS(frc::SerialPort::Port::kUSB);
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

void DriveTrain::ConfigureDriveTrainEncoders(bool competitionBot) {
	if (competitionBot) {
		driveTrainMotors[LEFT_SIDE_ENCODER]->ConfigEncoderCodesPerRev(CB_ENCODER_COUNTS);
		driveTrainMotors[RIGHT_SIDE_ENCODER]->ConfigEncoderCodesPerRev(CB_ENCODER_COUNTS);

		driveTrainMotors[LEFT_SIDE_ENCODER]->SetSensorDirection(CB_LEFT_ENCODER_SENSOR_DIRECTION);
		driveTrainMotors[RIGHT_SIDE_ENCODER]->SetSensorDirection(CB_RIGHT_ENCODER_SENSOR_DIRECTION);

		quadratureCountOfEncoder = (CB_ENCODER_COUNTS * 4);
	}
	else if (competitionBot == false) {
		driveTrainMotors[LEFT_SIDE_ENCODER]->ConfigEncoderCodesPerRev(PB_ENCODER_COUNTS);
		driveTrainMotors[RIGHT_SIDE_ENCODER]->ConfigEncoderCodesPerRev(PB_ENCODER_COUNTS);

		driveTrainMotors[LEFT_SIDE_ENCODER]->SetSensorDirection(PB_LEFT_ENCODER_SENSOR_DIRECTION);
		driveTrainMotors[RIGHT_SIDE_ENCODER]->SetSensorDirection(PB_RIGHT_ENCODER_SENSOR_DIRECTION);

		quadratureCountOfEncoder = (PB_ENCODER_COUNTS * 4);
	}

	encoderCountToDistanceConstant = (CIRCUMFERENCE_OF_WHEEL/((double)quadratureCountOfEncoder));

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
	distanceTraveled = (encoderValue * encoderCountToDistanceConstant);
	return distanceTraveled;
}

double DriveTrain::GetNavXAngle() {
	return navX->GetAngle();
}

void DriveTrain::ZeroNavXAngle() {
	navX->ZeroYaw();
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
