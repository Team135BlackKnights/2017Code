#ifndef DriveTrain_H
#define DriveTrain_H

#include <Commands/Subsystem.h>
#include <CANTalon.h>
#include <RobotDrive.h>
#include <math.h>
#include <AHRS.h>

class DriveTrain : public Subsystem {
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

	static const int ENCODER_COUNTS = 256;
	static const int QUADRATURE_ENCODER_COUNTS = (ENCODER_COUNTS * 4);

	static constexpr double DIAMETER_OF_WHEEL_IN = 4.1;
	static constexpr double CIRCUMFERENCE_OF_WHEEL = (DIAMETER_OF_WHEEL_IN * M_PI);

	static constexpr double ENCODER_COUNT_TO_DISTANCE_CONSTANT = (CIRCUMFERENCE_OF_WHEEL/((double)QUADRATURE_ENCODER_COUNTS));

	int encoderValue = 0;
	double distanceTraveled = 0.0;

	AHRS* navX;

public:
	DriveTrain();
	void InitDefaultCommand();

	void InitializeDriveTrainMotors(bool);
	void DriveTank(double, double);
	void RotateTank(double, bool);

	void ConfigureDriveTrainEncoders();
	void ZeroDriveTrainEncoder(int);
	int GetEncoderPosition(int);
	double GetDistance(int);

	double GetNavXAngle();
	void ZeroNavXAngle();

	static const int LEFT_SIDE_ENCODER = REAR_LEFT;
	static const int RIGHT_SIDE_ENCODER = REAR_RIGHT;
};

#endif  // DriveTrain_H
