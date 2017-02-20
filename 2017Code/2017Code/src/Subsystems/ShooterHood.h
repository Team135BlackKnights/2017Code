#ifndef ShooterHood_H
#define ShooterHood_H

#include <Commands/Subsystem.h>
#include <CANTalon.h>

class ShooterHood : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

	CANTalon* shooterHoodMotor;

	//  Shooter Hood Encoder Count To Be Determined
	static const int SHOOTER_HOOD_ENCODER_COUNTS = 497;
	static const int SHOOTER_HOOD_QUADRATURE_ENCODER_COUNTS = (SHOOTER_HOOD_ENCODER_COUNTS * 4);

	static const bool REVERSE_SHOOTER_HOOD_ENCODER_DIRECTION = false;

	static const int MIN_ENCODER_VALUE = 0;
	static const int MAX_ENCODER_VALUE = 80;
	static const int RANGE_OF_ENCODER_VALUES = (MAX_ENCODER_VALUE - MIN_ENCODER_VALUE);

	static const int MIN_ANGLE_VALUE = 50;
	static const int MAX_ANGLE_VALUE = 75;
	static const int RANGE_OF_ANGLE_VALUES = (MAX_ANGLE_VALUE - MIN_ANGLE_VALUE);

	static constexpr double ENCODER_POSITION_TO_ANGLE_OF_SHOOTER_HOOD = (((double)RANGE_OF_ANGLE_VALUES)/((double)RANGE_OF_ENCODER_VALUES));

	double angleAboveHood = 0.0;
	double actualAngle = 0.0;

	double currentEncoderPosition = 0.0;
	double currentAngle = 0.0;

	bool initializeDirectionOfHoodToMove = false;
	bool driveHoodToIncreaseAngle = false;
	bool drivenToAngle = false;
public:
	ShooterHood();
	void InitDefaultCommand();

	void InitializeShooterHoodMotor(bool);

	void DriveShooterHoodMotor(double);

	void ConfigureShooterHoodEncoder();
	int GetShooterHoodEncoderPosition();
	void ZeroShooterHoodEncoder();
	double GetAngleOfShooterHoodGivenEncoderPosition(int);
	bool DriveShooterHoodMotorToDesiredAngle(double, double);
};

#endif  // ShooterHood_H
