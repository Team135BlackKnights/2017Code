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

	for (int i = 0; i < NUM_OF_DRIVE_TRAIN_MOTORS; i++) {
		driveTrainMotors[i]->ConfigNeutralMode(CANTalon::NeutralMode::kNeutralMode_Brake);
	}

	chassis = new frc::RobotDrive(driveTrainMotors[FRONT_LEFT], driveTrainMotors[REAR_LEFT], driveTrainMotors[FRONT_RIGHT], driveTrainMotors[REAR_RIGHT]);
	chassis->SetSafetyEnabled(false);

	chassis->SetInvertedMotor(frc::RobotDrive::MotorType::kFrontLeftMotor, true);
	chassis->SetInvertedMotor(frc::RobotDrive::MotorType::kRearLeftMotor, true);
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

void DriveTrain::SwitchBetweenFastAndSlowDriveTrainMotorPower(bool fastDriveTrainMotorPower) {
	this->fastDriveTrainMotorPower = fastDriveTrainMotorPower;
}

double DriveTrain::GetDesiredDriveTrainMaxMotorPower() {
	if (this->fastDriveTrainMotorPower) {
		desiredMaxMotorPower = FAST_MAX_DRIVE_TRAIN_MOTOR_POWER;
	}
	else if (this->fastDriveTrainMotorPower == false) {
		desiredMaxMotorPower = SLOW_MAX_DRIVE_TRAIN_MOTOR_POWER;
	}
	return desiredMaxMotorPower;
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

bool DriveTrain::MakeSureDriveTrainEncoderIsPluggedIn(bool rightLeftSideEncoder) {
	if (RIGHT_SIDE_ENCODER_BOOLEAN) {
		rightEncoderPluggedIn = driveTrainMotors[RIGHT_SIDE_ENCODER]->IsSensorPresent(CANTalon::FeedbackDevice::QuadEncoder);
		if (rightEncoderPluggedIn == UNKNOWN_CONNECTED || rightEncoderPluggedIn == RECOGNIZED_CONNECTED) {
			rightDriveTrainEncoderPluggedIn = true;
		}
		else if (rightEncoderPluggedIn == DISCONNECTED) {
			rightDriveTrainEncoderPluggedIn = false;
		}
		return rightDriveTrainEncoderPluggedIn;
	}
	else if (LEFT_SIDE_ENCODER_BOOLEAN) {
		leftEncoderPluggedIn = driveTrainMotors[LEFT_SIDE_ENCODER]->IsSensorPresent(CANTalon::FeedbackDevice::QuadEncoder);
		if (leftEncoderPluggedIn == UNKNOWN_CONNECTED || leftEncoderPluggedIn == RECOGNIZED_CONNECTED) {
			leftDriveTrainEncoderPluggedIn = true;
		}
		else if (leftEncoderPluggedIn == DISCONNECTED) {
			leftDriveTrainEncoderPluggedIn = false;
		}
		return leftDriveTrainEncoderPluggedIn;
	}
}

void DriveTrain::ZeroDriveTrainEncoder(int motorEncoderPort) {
	driveTrainMotors[motorEncoderPort]->SetEncPosition(0);
}

int DriveTrain::GetEncoderPosition(int motorEncoderPort) {
	if (motorEncoderPort == LEFT_SIDE_ENCODER) {
		return (-1 * driveTrainMotors[motorEncoderPort]->GetEncPosition());
	}
	else {
		return driveTrainMotors[motorEncoderPort]->GetEncPosition();
	}
}

double DriveTrain::GetDistance(int motorEncoderPort) {
	encoderValue = this->GetEncoderPosition(motorEncoderPort);
	distanceTraveled = (encoderValue * encoderCountToDistanceConstant);
	return (distanceTraveled * DRIVE_TRAIN_SPROCKET_RATIO);
}

double DriveTrain::GetStraightDistanceTraveled(int motorEncoderPort, double gyroAngleDegrees, bool configureInitialDistanceTraveled, bool drivingForwards) {
	if (configureInitialDistanceTraveled) {
		savedDistanceTraveled = this->GetDistance(motorEncoderPort);
		distanceTraveledStraight = 0.0;
	}
	else if (configureInitialDistanceTraveled == false) {
		currentDistanceTraveled = this->GetDistance(motorEncoderPort);
		if (drivingForwards) {
			differenceBetweenCurrentAndSavedDistanceTraveled = (currentDistanceTraveled - savedDistanceTraveled);
		}
		else if (drivingForwards == false) {
			differenceBetweenCurrentAndSavedDistanceTraveled = (savedDistanceTraveled - currentDistanceTraveled);
		}
		gyroAngleRadians = (gyroAngleDegrees * DEGREES_TO_RADIANS_CONSTANT);
		smallPortionOfStraightDistanceTraveled = (differenceBetweenCurrentAndSavedDistanceTraveled * (cos(gyroAngleRadians)));
		distanceTraveledStraight += smallPortionOfStraightDistanceTraveled;
		savedDistanceTraveled = currentDistanceTraveled;
	}
	return distanceTraveledStraight;
}

int DriveTrain::GetEncoderRPM(int motorEncoderPort) {
	return driveTrainMotors[motorEncoderPort]->GetSpeed();
}

double DriveTrain::GetTalonOutputCurrent(int motorArray) {
	return driveTrainMotors[motorArray]->GetOutputCurrent();
}

void DriveTrain::InitializeDriveStraightWithGyro(bool competitionBot) {
	if (competitionBot) {
		straightDriveTrainSensitivity = CB_STRAIGHT_DRIVE_TRAIN_SENSITIVITY;
		straightDriveTrainProportionalConstant = CB_STRAIGHT_DRIVE_TRAIN_PROPORTIONAL_CONSTANT;
		ultrasonicSensorDriveStraightProportionalConstant = CB_DRIVE_STRAIGHT_WITH_ULTRASONIC_SENSOR_PROPORTIONAL_CONSTANT;
		ultrasonicSensorSensitivityValue = CB_DRIVE_STRAIGHT_WITH_ULTRASONIC_SENSOR_SENSITIVITY_CONSTANT;
	}
	else if (competitionBot == false) {
		straightDriveTrainSensitivity = .05;  //. 12  Low Battery
		straightDriveTrainProportionalConstant = .125;  //  .04  Low Battery
		ultrasonicSensorDriveStraightProportionalConstant = .1;  //  .12 ///////////////////
		ultrasonicSensorSensitivityValue = .5;  //  .17 /////////////////////////
	}
	chassis->SetSensitivity(straightDriveTrainSensitivity);
	initializeGyroDriveStraight = true;
	initializeUltrasonicSensorDriveStraight = false;
}

void DriveTrain::ChangeDriveStraightSensitivity(double sensitivity) {
	chassis->SetSensitivity(sensitivity);
}

void DriveTrain::DriveStraightWithGyro(double motorPower, double gyroAngle) {
	if (initializeGyroDriveStraight == false) {
		this->ChangeDriveStraightSensitivity(straightDriveTrainSensitivity);
		initializeGyroDriveStraight = true;
		initializeUltrasonicSensorDriveStraight = false;
	}
	if (motorPower > 0.0) {
		curveValue = (-1 * gyroAngle * straightDriveTrainProportionalConstant);
	}
	else if (motorPower < 0.0) {
		curveValue = (gyroAngle * straightDriveTrainProportionalConstant);
	}

	if (curveValue > 1.0) {
		curveValue = 1.0;
	}
	else if (curveValue < -1.0) {
		curveValue = -1.0;
	}

	chassis->Drive(motorPower, curveValue);
}

void DriveTrain::DriveStraightWithUltrasonicSensor(double currentUltrasonicSensorValue, double desiredDistanceFromGuardrail, double motorPower, bool rightSideHopperAndShoot) {
	if (initializeUltrasonicSensorDriveStraight == false) {
		this->ChangeDriveStraightSensitivity(ultrasonicSensorSensitivityValue);
		initializeUltrasonicSensorDriveStraight = true;
		initializeGyroDriveStraight = false;
	}
	differenceBetweenCurrentAndDesiredUltrasonicSensorValue = (currentUltrasonicSensorValue - desiredDistanceFromGuardrail);
	if (rightSideHopperAndShoot) {
		ultrasonicSensorDriveStraightCurveValue = (-1.0 * ultrasonicSensorDriveStraightProportionalConstant * differenceBetweenCurrentAndDesiredUltrasonicSensorValue);
	}
	else if (rightSideHopperAndShoot == false) {
		ultrasonicSensorDriveStraightCurveValue = (ultrasonicSensorDriveStraightProportionalConstant * differenceBetweenCurrentAndDesiredUltrasonicSensorValue);
	}

	if (ultrasonicSensorDriveStraightCurveValue > 1.0) {
		ultrasonicSensorDriveStraightCurveValue = 1.0;
	}
	else if (ultrasonicSensorDriveStraightCurveValue < -1.0) {
		ultrasonicSensorDriveStraightCurveValue = -1.0;
	}

	chassis->Drive(motorPower, ultrasonicSensorDriveStraightCurveValue);

	std::cout << "Curve Value: " << ultrasonicSensorDriveStraightCurveValue << std::endl;
}

void DriveTrain::InitializeDriveTrainPID() {
	gyro = new ADXRS450_Gyro(); //maybe?
	gyro->Calibrate();
	turnController = new frc::PIDController(Preferences::GetInstance()->GetDouble("kP",0.0625), Preferences::GetInstance()->GetDouble("kI",0.0),Preferences::GetInstance()->GetDouble("kD",0.0) , kF, gyro, this);
	//turnController = new frc::PIDController(kP,kI,kD,kF, gyro, this);
	turnController->SetInputRange(-90.0f,  90.0f);
	turnController->SetOutputRange(-1.0, 1.0);
	turnController->SetAbsoluteTolerance(kToleranceDegrees);
	turnController->SetContinuous(true);
	turnController->Disable();
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
	//std::cout <<"navx angle: " << gyro->GetAngle() << "\n";
	turnController->SetSetpoint(angleToTurn);
	turnController->Enable();
}

void DriveTrain::TurnPIDDisable()
{
	turnController->Disable();
}

void DriveTrain::PIDTurning()
{
	//std::cout << "navx angle: " << gyro->GetAngle();
	//std::cout << "rot rate: " << rotateToAngleRate << "\n";
	this->RotateTank(rotateToAngleRate, 1);
}

bool DriveTrain::AutoRotateRobot(double motorPower, double desiredGyroAngleToTurn, bool turnRight, bool initializeAutoRotateRobot) {
	if (initializeAutoRotateRobot == false) {
		initialGyroAngle = this->GetGyroAngle();
		std::cout << "Initial Gyro Angle: " << initialGyroAngle << std::endl;
		doneAutoRotateRobot = false;
		initializeAutoRotateRobot = true;
	}

	currentGyroAngle = this->GetGyroAngle();
	std::cout << "Current Gyro Angle" << currentGyroAngle << std::endl;
	differenceBetweenCurrentAndInitialGyroAngle = (fabs(currentGyroAngle - initialGyroAngle));
	std::cout << "Difference: " << differenceBetweenCurrentAndInitialGyroAngle << std::endl;
	std::cout << "Desired Gyro ANgleee: " << desiredGyroAngleToTurn << std::endl;

	if (initializeAutoRotateRobot) {
		if (differenceBetweenCurrentAndInitialGyroAngle >= desiredGyroAngleToTurn) {
			this->RotateTank(0.0, turnRight);
			initializeAutoRotateRobot = false;
			doneAutoRotateRobot = true;
			std::cout << "Done" << std::endl;
		}
		else {
			this->RotateTank(motorPower, turnRight);
		}
	}
	return doneAutoRotateRobot;
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
