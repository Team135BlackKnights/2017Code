#ifndef DriveTrain_H
#define DriveTrain_H

#include <Commands/Subsystem.h>
#include <CANTalon.h>
#include <RobotDrive.h>
#include <math.h>
#include <PIDController.h>
#include <Preferences.h>
#include <ADXRS450_Gyro.h>

class DriveTrain : public frc::PIDOutput, public frc::Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

	static const int NUM_OF_DRIVE_TRAIN_MOTORS = 4;
	CANTalon* driveTrainMotors[NUM_OF_DRIVE_TRAIN_MOTORS];


	//  Drive Train Encoder Variables
	static const bool CB_LEFT_ENCODER_SENSOR_DIRECTION = true;
	static const bool CB_RIGHT_ENCODER_SENSOR_DIRECTION = false;
	static const bool PB_LEFT_ENCODER_SENSOR_DIRECTION = false;
	static const bool PB_RIGHT_ENCODER_SENSOR_DIRECTION = false;

	static const int CB_ENCODER_COUNTS = 64;
	static const int CB_QUADRATURE_ENCODER_COUNTS = (CB_ENCODER_COUNTS * 4);
	static const int PB_ENCODER_COUNTS = 256;
	static const int PB_QUADRATURE_ENCODER_COUNTS = (PB_ENCODER_COUNTS * 4);

	int quadratureCountOfEncoder = 0;
	double encoderCountToDistanceConstant = 0;

	static constexpr double DIAMETER_OF_WHEEL_IN = 4.1;
	static constexpr double CIRCUMFERENCE_OF_WHEEL = (DIAMETER_OF_WHEEL_IN * M_PI);

	ADXRS450_Gyro* gyro;

	int encoderValue = 0;
	double distanceTraveled = 0.0;

	double kToleranceDegrees = .5f;
	double kP = .144f;
	double kI = 0;
	double kD = 0.1f;

	double kF = 0.0f;

	//  Practice Bot Straight Drive Train
	//  Sensitivity .02
	//  Kp .2

	double straightDriveTrainSensitivity = 0.0;
	double straightDriveTrainProportionalConstant = 0.0;

	static constexpr double PB_STRAIGHT_DRIVE_TRAIN_SENSITIVITY = .05;
	static constexpr double PB_STRAIGHT_DRIVE_TRAIN_PROPORTIONAL_CONSTANT = .125;
	static constexpr double CB_STRAIGHT_DRIVE_TRAIN_SENSITIVITY = .07;
	static constexpr double CB_STRAIGHT_DRIVE_TRAIN_PROPORTIONAL_CONSTANT = .12;

	double curveValue = 0.0;

	const CANTalon::FeedbackDeviceStatus UNKNOWN_CONNECTED = CANTalon::FeedbackDeviceStatus::FeedbackStatusUnknown;
	const CANTalon::FeedbackDeviceStatus RECOGNIZED_CONNECTED = CANTalon::FeedbackDeviceStatus::FeedbackStatusPresent;
	const CANTalon::FeedbackDeviceStatus DISCONNECTED = CANTalon::FeedbackDeviceStatus::FeedbackStatusNotPresent;

	CANTalon::FeedbackDeviceStatus rightEncoderPluggedIn;
	CANTalon::FeedbackDeviceStatus leftEncoderPluggedIn;

	bool rightDriveTrainEncoderPluggedIn = false;
	bool leftDriveTrainEncoderPluggedIn = false;

	static constexpr double DRIVE_TRAIN_SPROCKET_RATIO = (15.0/12.0);
	//static constexpr double DRIVE_TRAIN_SPROCKET_RATIO = 1.0;

	//  Variables for Switching the MAX Drive Train Motor Power
	bool fastDriveTrainMotorPower = true;
	double desiredMaxMotorPower = 0.0;

	//  Variables for AutoRotateRobot()
	double currentGyroAngle = 0.0;
	double initialGyroAngle = 0.0;
	double differenceBetweenCurrentAndInitialGyroAngle = 0.0;
	bool doneAutoRotateRobot = false;

	//  Variables for DriveStraightWithUltrasonicSensor()
	double differenceBetweenCurrentAndDesiredUltrasonicSensorValue = 0.0;
	double ultrasonicSensorDriveStraightProportionalConstant = 0.0;
	double ultrasonicSensorDriveStraightCurveValue = 0.0;
	double ultrasonicSensorSensitivityValue = 0.0;

	bool initializeUltrasonicSensorDriveStraight = false;
	bool initializeGyroDriveStraight = false;

	static constexpr double PB_DRIVE_STRAIGHT_WITH_ULTRASONIC_SENSOR_PROPORTIONAL_CONSTANT = .12;
	static constexpr double PB_DRIVE_STRAIGHT_WITH_ULTRASONIC_SENSOR_SENSITIVITY_CONSTANT = .17;
	static constexpr double CB_DRIVE_STRAIGHT_WITH_ULTRASONIC_SENSOR_PROPORTIONAL_CONSTANT = .05;  //  TBD
	static constexpr double CB_DRIVE_STRAIGHT_WITH_ULTRASONIC_SENSOR_SENSITIVITY_CONSTANT = .17;  //  TBD

public:
	DriveTrain();
	void InitDefaultCommand();

	void InitializeDriveTrainMotors(bool);
	void DriveTank(double, double);
	void RotateTank(double, bool);

	void SwitchBetweenFastAndSlowDriveTrainMotorPower(bool);
	double GetDesiredDriveTrainMaxMotorPower();

	void ConfigureDriveTrainEncoders(bool);
	bool MakeSureDriveTrainEncoderIsPluggedIn(bool);
	void ZeroDriveTrainEncoder(int);
	int GetEncoderPosition(int);
	double GetDistance(int);
	double GetStraightDistanceTraveled(int, double, bool, bool);

	double initialDistanceTraveled = 0.0;
	double savedDistanceTraveled = 0.0;
	double currentDistanceTraveled = 0.0;
	double differenceBetweenCurrentAndSavedDistanceTraveled = 0.0;
	double smallPortionOfStraightDistanceTraveled = 0.0;
	double distanceTraveledStraight = 0.0;
	double gyroAngleRadians = 0.0;
	static constexpr double DEGREES_TO_RADIANS_CONSTANT = (M_PI/180.0);

	int GetEncoderRPM(int);

	void InitializeDriveStraightWithGyro(bool);
	void ChangeDriveStraightSensitivity(double);
	void DriveStraightWithGyro(double, double);

	void DriveStraightWithUltrasonicSensor(double, double, double, bool);

	double GetTalonOutputCurrent(int);


	frc::RobotDrive* chassis;

	static const int FRONT_LEFT = 0;
	static const int REAR_LEFT = 1;
	static const int FRONT_RIGHT = 2;
	static const int REAR_RIGHT = 3;

	//  For Competition Bot
	static const int LEFT_SIDE_ENCODER = REAR_LEFT;
	static const int RIGHT_SIDE_ENCODER = REAR_RIGHT;

	static const bool RIGHT_SIDE_ENCODER_BOOLEAN = true;
	static const bool LEFT_SIDE_ENCODER_BOOLEAN = !RIGHT_SIDE_ENCODER_BOOLEAN;

	static constexpr double FAST_MAX_DRIVE_TRAIN_MOTOR_POWER = 1.0;
	static constexpr double SLOW_MAX_DRIVE_TRAIN_MOTOR_POWER = .75;

	void InitializeDriveTrainPID();
	double GetGyroAngle();
	void ZeroGyroAngle();
	void TurnPIDEnable(double angleToTurn);
	void PIDTurning();
	void TurnPIDDisable();

	bool is_aiming = false;
	double rotateToAngleRate = 0;
	virtual void PIDWrite(double output) {
	        this->rotateToAngleRate = output;
	    }
	PIDController* turnController;

	bool AutoRotateRobot(double, double, bool, bool);
};

#endif  // DriveTrain_H
