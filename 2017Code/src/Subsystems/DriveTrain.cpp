#include "DriveTrain.h"
#include "../RobotMap.h"
#include "Commands/DriveWithJoysticks.h"

DriveTrain::DriveTrain() : frc::PIDOutput(), Subsystem("DriveTrain") {
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

int DriveTrain::GetEncoderRPM(int motorEncoderPort) {
	return driveTrainMotors[motorEncoderPort]->GetSpeed();
}

void DriveTrain::DriveStraightWithGyro(double motorPower, double gyroAngle) {
	curveValue = ((-1 * gyroAngle) * STRAIGHT_DRIVE_TRAIN_PROPORTIONAL_CONSTANT);
	chassis->Drive(motorPower, curveValue);
}

void DriveTrain::InitializeDriveTrainPID() {
	gyro = new ADXRS450_Gyro(); //maybe?
	gyro->Calibrate();
	//for testingturnController = new frc::PIDController(Preferences::GetInstance()->GetDouble("kP",0.0625), Preferences::GetInstance()->GetDouble("kI",0.0),Preferences::GetInstance()->GetDouble("kD",0.0) , kF, gyro, this);
	turnController = new frc::PIDController(kP,kI,kD,kF, gyro, this);

	turnController->SetInputRange(-90.0f,  90.0f);
	turnController->SetOutputRange(-1.0, 1.0);
	turnController->SetAbsoluteTolerance(kToleranceDegrees);
	turnController->SetContinuous(true);
	turnController->Disable();

	chassis->SetSensitivity(STRAIGHT_DRIVE_TRAIN_SENSITIVITY);
}

double DriveTrain::GetGyroAngle() {
	return gyro->GetAngle();
}

void DriveTrain::ZeroGyroAngle() {
	gyro->Reset();
}

void DriveTrain::TurnPIDEnable(double angleToTurn)
{
	ZeroGyroAngle();
	std::cout <<"navx angle: " << gyro->GetAngle() << "\n";
	turnController->SetSetpoint(angleToTurn);
	turnController->Enable();
}

void DriveTrain::TurnPIDDisable()
{
	turnController->Disable();
}

void DriveTrain::PIDTurning()
{
	std::cout << "navx angle: " << gyro->GetAngle();
	std::cout << "rot rate: " << rotateToAngleRate << "\n";
	this->RotateTank(rotateToAngleRate, 1);
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
