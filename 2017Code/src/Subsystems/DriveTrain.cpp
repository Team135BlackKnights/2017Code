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

		chassis = new frc::RobotDrive(driveTrainMotors[FRONT_LEFT], driveTrainMotors[REAR_LEFT], driveTrainMotors[FRONT_RIGHT], driveTrainMotors[REAR_RIGHT]);

		chassis->SetInvertedMotor(frc::RobotDrive::MotorType::kFrontLeftMotor, CB_DRIVE_TRAIN_FRONT_LEFT_INVERTED);
		chassis->SetInvertedMotor(frc::RobotDrive::MotorType::kRearLeftMotor, CB_DRIVE_TRAIN_REAR_LEFT_INVERTED);
		chassis->SetInvertedMotor(frc::RobotDrive::MotorType::kFrontRightMotor, CB_DRIVE_TRAIN_FRONT_RIGHT_INVERTED);
		chassis->SetInvertedMotor(frc::RobotDrive::MotorType::kRearRightMotor, CB_DRIVE_TRAIN_REAR_RIGHT_INVERTED);
	}
	else if (competitionBot == false) {
		driveTrainMotors[FRONT_LEFT] = new CANTalon(PB_FRONT_LEFT_TALON_ID);
		driveTrainMotors[REAR_LEFT] = new CANTalon(PB_REAR_LEFT_TALON_ID);
		driveTrainMotors[FRONT_RIGHT] = new CANTalon(PB_FRONT_RIGHT_TALON_ID);
		driveTrainMotors[REAR_RIGHT] = new CANTalon(PB_REAR_RIGHT_TALON_ID);

		chassis = new frc::RobotDrive(driveTrainMotors[FRONT_LEFT], driveTrainMotors[REAR_LEFT], driveTrainMotors[FRONT_RIGHT], driveTrainMotors[REAR_RIGHT]);

		chassis->SetInvertedMotor(frc::RobotDrive::MotorType::kFrontLeftMotor, PB_DRIVE_TRAIN_FRONT_LEFT_INVERTED);
		chassis->SetInvertedMotor(frc::RobotDrive::MotorType::kRearLeftMotor, PB_DRIVE_TRAIN_REAR_LEFT_INVERTED);
		chassis->SetInvertedMotor(frc::RobotDrive::MotorType::kFrontRightMotor, PB_DRIVE_TRAIN_FRONT_RIGHT_INVERTED);
		chassis->SetInvertedMotor(frc::RobotDrive::MotorType::kRearRightMotor, PB_DRIVE_TRAIN_REAR_RIGHT_INVERTED);
	}
}

void DriveTrain::DriveTank(double leftMotorPower, double rightMotorPower) {
	chassis->TankDrive(leftMotorPower, rightMotorPower);
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

// Put methods for controlling this subsystem
// here. Call these from Commands.
