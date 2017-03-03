#ifndef ShooterHood_H
#define ShooterHood_H

#include <Commands/Subsystem.h>
#include <CANTalon.h>

class ShooterHood : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

	CANTalon* shooterHoodMotor;

	double shooterHoodMotorPower = 0.0;

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

	//  Variables for Calculating Angle for Shooter Hood Given Lidar Value
	static constexpr double ACCELERATION_OF_GRAVITY = 9.81;
	static constexpr double ADDED_X_DISTANCE_MIN_INSIDE_BOILER_IN = 17.5;  //  Directly in front of boiler
	static constexpr double ADDED_X_DISTANCE_MIN_INSIDE_BOILER_CM = (ADDED_X_DISTANCE_MIN_INSIDE_BOILER_IN * 2.54);
	static constexpr double ADDED_X_DISTANCE_MIN_INSIDE_BOILER_M = (ADDED_X_DISTANCE_MIN_INSIDE_BOILER_CM/100.0);
	static constexpr double ADDED_X_DISTANCE_MAX_INSIDE_BOILER_IN = 25.0;  //  Shooting from the side of the field //  Hypotenuse of Triangle is 27.3in.
	static constexpr double ADDED_X_DISTANCE_MAX_INSIDE_BOILER_CM = (ADDED_X_DISTANCE_MAX_INSIDE_BOILER_IN * 2.54);
	static constexpr double ADDED_X_DISTANCE_MAX_INSIDE_BOILER_M = (ADDED_X_DISTANCE_MAX_INSIDE_BOILER_CM/100.0);
	static constexpr double SHOOTER_CLOSE_SHOT_M_PER_SEC = 5.0;  //  To Be Determined
	static constexpr double SHOOTER_FAR_SHOT_M_PER_SEC = 6.5;  //  To Be Determined
	static constexpr double BOILER_HEIGHT_IN = 97.0;
	static constexpr double BOILER_HEIGHT_CM = (BOILER_HEIGHT_IN * 2.54);
	static constexpr double BOILER_HEIGHT_M = (BOILER_HEIGHT_CM/100.0);
	static constexpr double SHOOTER_HEIGHT_OFF_GROUND_IN = 10.0;
	static constexpr double SHOOTER_HEIGHT_OFF_GROUND_CM = (SHOOTER_HEIGHT_OFF_GROUND_IN * 2.54);
	static constexpr double SHOOTER_HEIGHT_OFF_GROUND_M = (SHOOTER_HEIGHT_OFF_GROUND_CM/100.0);
	static constexpr double Y_DISTANCE_M = (BOILER_HEIGHT_M - SHOOTER_HEIGHT_OFF_GROUND_M);

	double valueAngleForLoopHasToEqual = 0.0;
	double xDistanceFromLidar_M = 0.0;
	double totalXDistance_M = 0.0;
	double chosenVelocityOfShooter = 0.0;
public:
	ShooterHood();
	void InitDefaultCommand();

	void InitializeShooterHoodMotor(bool);

	void DriveShooterHoodMotor(double);

	void ConfigureShooterHoodEncoder();
	int GetShooterHoodEncoderPosition();
	void SetShooterHoodEncoder(int);
	void ZeroShooterHoodEncoder();
	double GetAngleOfShooterHoodGivenEncoderPosition(int);
	bool DriveShooterHoodMotorToDesiredAngle(double, double);
	double GetDesiredAngleOfShooterHood(double, bool, int);

	int GetMaxAngleLimitSwitch();
	int GetMinAngleLimitSwitch();

	static const int NUM_OF_SHOOTER_ANGLED_POSITIONS = 2;
	static const int STRAIGHT_ON = 0;
	static const int FURTHEST_POINT_FROM_STRAIGHT_ON = 1;

	static constexpr int SHOOTER_ANGLED_POSITION_ARRAY[NUM_OF_SHOOTER_ANGLED_POSITIONS] = {STRAIGHT_ON, FURTHEST_POINT_FROM_STRAIGHT_ON};
};

#endif  // ShooterHood_H
