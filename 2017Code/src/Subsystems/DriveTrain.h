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

	static const int FRONT_LEFT = 0;
	static const int REAR_LEFT = 1;
	static const int FRONT_RIGHT = 2;
	static const int REAR_RIGHT = 3;

	frc::RobotDrive* chassis;

	//  Drive Train Encoder Variables
	static const bool CB_LEFT_ENCODER_SENSOR_DIRECTION = true;
	static const bool CB_RIGHT_ENCODER_SENSOR_DIRECTION = false;
	static const bool PB_LEFT_ENCODER_SENSOR_DIRECTION = true;
	static const bool PB_RIGHT_ENCODER_SENSOR_DIRECTION = false;

	static const int CB_ENCODER_COUNTS = 256;
	static const int CB_QUADRATURE_ENCODER_COUNTS = (CB_ENCODER_COUNTS * 4);
	static const int PB_ENCODER_COUNTS = 64;
	static const int PB_QUADRATURE_ENCODER_COUNTS = (PB_ENCODER_COUNTS * 4);

	int quadratureCountOfEncoder = 0;
	double encoderCountToDistanceConstant = 0;

	static constexpr double DIAMETER_OF_WHEEL_IN = 4.1;
	static constexpr double CIRCUMFERENCE_OF_WHEEL = (DIAMETER_OF_WHEEL_IN * M_PI);

	ADXRS450_Gyro* gyro;

	int encoderValue = 0;
	double distanceTraveled = 0.0;

	double kToleranceDegrees = .5f;
	double kP = .09f;
	double kI = 0;
	double kD = .2f;

	double kF = 0.0f;

	static constexpr double STRAIGHT_DRIVE_TRAIN_SENSITIVITY = .01;
	static constexpr double STRAIGHT_DRIVE_TRAIN_PROPORTIONAL_CONSTANT = .01;
	double curveValue = 0.0;

public:
	DriveTrain();
	void InitDefaultCommand();

	void InitializeDriveTrainMotors(bool);
	void DriveTank(double, double);
	void RotateTank(double, bool);

	void ConfigureDriveTrainEncoders(bool);
	void ZeroDriveTrainEncoder(int);
	int GetEncoderPosition(int);
	double GetDistance(int);

	void DriveStraightWithGyro(double, double);

	//  For Competition Bot
	static const int LEFT_SIDE_ENCODER = REAR_LEFT;
	static const int RIGHT_SIDE_ENCODER = REAR_RIGHT;

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
};

#endif  // DriveTrain_H
