#ifndef ShooterHood_H
#define ShooterHood_H

#include <Commands/Subsystem.h>
#include <CANTalon.h>
#include <math.h>

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
	//  15000 for Practice Bot
	static const int MAX_ENCODER_VALUE = 15000;
	static const int RANGE_OF_ENCODER_VALUES = (MAX_ENCODER_VALUE - MIN_ENCODER_VALUE);

	static constexpr double MIN_ANGLE_VALUE = 50;
	static constexpr double MAX_ANGLE_VALUE = 75;
	static const int RANGE_OF_ANGLE_VALUES = (MAX_ANGLE_VALUE - MIN_ANGLE_VALUE);

	static constexpr double ENCODER_POSITION_TO_ANGLE_OF_SHOOTER_HOOD = (((double)RANGE_OF_ANGLE_VALUES)/((double)RANGE_OF_ENCODER_VALUES));
	static constexpr double ANGLE_TO_ENCODER_POSITION_OF_SHOOTER_HOOD = (((double)RANGE_OF_ENCODER_VALUES)/((double)RANGE_OF_ANGLE_VALUES));

	//  Variables for GetAngleOfShooterHoodGivenEncoderPosition()
	double angleBelowMaxHoodAngle = 0.0;
	double actualAngle = 0.0;

	//  Variables for GetEncoderPositionOfShooterHoodGivenAngle()
	double calculatedAngleBelowMaxHoodAngle = 0.0;
	double doubleEncoderPosition = 0.0;
	int intEncoderPosition = 0;

	//  Variables for DriveShooterHoodMotorToDesiredAngle()
	int currentEncoderPosition = 0.0;
	double currentAngle = 0.0;
	int desiredEncoderPosition = 0.0;
	int differenceBetweenCurrentAndDesiredEncoderPositions = 0;
	static const int THRESHOLD_ENCODER_RANGE_TO_START_SLOWING_MOTOR_POWER = 1500;
	static constexpr double APPROACHING_DESIRED_ENCODER_POSITION_MOTOR_POWER = .5;
	double desiredMotorPowerToRunAt = 0.0;

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

	double xDistanceFromLidar_M = 0.0;
	double totalXDistance_M = 0.0;
	double chosenVelocityOfShooter = 0.0;
	double valueAngleForLoopHasToEqual = 0.0;
	double valueWithInputtedAngle = 0.0;
	double radiansAngleValue = 0.0;
	bool completeFirstForLoop = false;
	bool firstAngleValueReceived = false;
	double currentDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue = 0.0;
	double pastDifferenceBetweenCalculatedAngleValueAndDesiredAngleValue = 0.0;
	double calculatedAngle = 0.0;
	double maxPossibleAngleFirstRound = 0.0;
	double minPossibleAngleFirstRound = 0.0;
	int secondAngleCounter = 0;
	bool incrementCounterWhenAngleIsClose = false;
	bool startSecondForLoop = false;
	bool secondLoopCompleted = false;
	double desiredAngleValueFromSecondForLoop = 0.0;

	//  Variables for TimeOfShotGreaterThanTimeOfMaxHeight()
	double timeOfMaxHeight = 0.0;
	double timeOfShot = 0.0;
	double initialYVelocity = 0.0;
	double initialXVelocity = 0.0;
	double radiansDesiredAngleValue = 0.0;
	double lidarValue_Meters = 0.0;
	bool timeOfShotGreaterThanTimeOfMaxHeight = false;

	//  Constant for ConvertDegreesToRadians()
	static constexpr double DEGREES_TO_RADIANS_CONSTANT = (M_PI/180.0);

	int currentShooterHoodEncoderValue = 0;
	int differenceBetweenDesiredAndCurrentShooterHoodEncoderValue = 0;
	static constexpr double SHOOTER_HOOD_MOTOR_POWER = 1.0;
	static constexpr double SLOWER_SHOOTER_HOOD_MOTOR_POWER = .55;
	bool initializeDirectionOfShooterHood = false;
	bool drivingShooterHoodForward = false;
	bool hoodAtDesiredEncoderValue = false;
	static const int THRESHOLD_SHOOTER_HOOD_ENCODER_VALUE_TO_DECREASE_MOTOR_POWER = 1800;

	int maxLimitSwitchValue = 0;
	int minLimitSwitchValue = 0;

	const CANTalon::FeedbackDeviceStatus UNKNOWN_CONNECTED = CANTalon::FeedbackDeviceStatus::FeedbackStatusUnknown;
	const CANTalon::FeedbackDeviceStatus RECOGNIZED_CONNECTED = CANTalon::FeedbackDeviceStatus::FeedbackStatusPresent;
	const CANTalon::FeedbackDeviceStatus DISCONNECTED = CANTalon::FeedbackDeviceStatus::FeedbackStatusNotPresent;

	CANTalon::FeedbackDeviceStatus hoodEncoderPresent;

	bool hoodEncoderPluggedIn = false;

public:
	ShooterHood();
	void InitDefaultCommand();

	void InitializeShooterHoodMotor(bool);

	void DriveShooterHoodMotor(double);

	void ConfigureShooterHoodEncoder();
	bool HoodEncoderPluggedIn();

	int GetShooterHoodEncoderPosition();
	void SetShooterHoodEncoder(int);
	void ZeroShooterHoodEncoder();

	double GetAngleOfShooterHoodGivenEncoderPosition(int);
	int GetEncoderPositionOfShooterHoodGivenAngle(double);
	bool DriveShooterHoodMotorToDesiredAngle(double, double);
	bool DriveShooterHoodToDesiredEncoderValue(int);

	void ResetDesiredAngleOfShooterHoodFunctionVariables();
	double GetDesiredAngleOfShooterHood(double, bool, int);
	bool TimeOfShotGreaterThanTimeOfMaxHeight(double, bool, double, int);

	void CheckIfHoodHitsLimitSwitch();
	int GetMaxAngleLimitSwitch();
	int GetMinAngleLimitSwitch();

	double ConvertDegreesToRadians(double);

	static const int NUM_OF_SHOOTER_ANGLED_POSITIONS = 2;
	static const int STRAIGHT_ON = 0;
	static const int FURTHEST_POINT_FROM_STRAIGHT_ON = 1;

	static constexpr int SHOOTER_ANGLED_POSITION_ARRAY[NUM_OF_SHOOTER_ANGLED_POSITIONS] = {STRAIGHT_ON, FURTHEST_POINT_FROM_STRAIGHT_ON};
};

#endif  // ShooterHood_H
