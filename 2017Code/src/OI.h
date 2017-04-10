#ifndef OI_H
#define OI_H

#include <WPILib.h>

class OI {
private:
	static const int NUM_OF_JOYSTICKS = 3;
	frc::Joystick* joystick[NUM_OF_JOYSTICKS];

	static const int NUM_OF_BUTTONS = 12;
	frc::JoystickButton* joystickButton[NUM_OF_JOYSTICKS][NUM_OF_BUTTONS];

	double joystickYAxisValue = 0.0;
	double deadbandJoystickValue = 0.0;

	static constexpr double JOYSTICK_DEADBAND_VALUE = .15;

	static const int AGITATOR_FORWARD_BUTTON = 8;
	static const int AGITATOR_BACKWARDS_BUTTON = 7;

	static const int COLLECTION_FORWARD_BUTTON = 10;
	static const int COLLECTION_BACKWARDS_BUTTON = 12;

	static const int SHOOTER_HOOD_INCREASE_ANGLE_BUTTON = 5;
	static const int SHOOTER_HOOD_DECREASE_ANGLE_BUTTON = 3;

	static const int SHOOTER_HOOD_SIDE_GEAR_SHOT_BUTTON = 12;  //  Right Drive Joystick
	static const int SHOOTER_HOOD_MIDDLE_GEAR_BUTTON = 11;  //  Right Drive Joystick
	static const int SHOOTER_HOOD_40_KPA_AUTONOMOUS_BUTTON = 10;  //  Right Drive Joystick
	static const int SHOOTER_HOOD_LEFT_CLOSE_SHOT_BUTTON = 9;

	static const int SHOOTER_HOOD_CLOSE_SHOT_BUTTON = 11;  //  MANIPULATOR JOYSTICK

	static const int GET_LIFT_HANG_INTO_CORRECT_POSITION_BUTTON = 5;  //  Right Drive Joystick

	static const int SWITCH_BETWEEN_MAX_DRIVE_TRAIN_MOTOR_POWER_BUTTON = 6;  //  Right Drive Joystick

	static const int SWITCH_BETWEEN_FAR_AND_CLOSE_SHOT_RPM_BUTTON = 9;  //  Manipulator Joystick

	static const int READ_LIDAR_VALUE_BUTTON = 8;  //  Left Drive Joystick

	static const int CLOSE_SHOT_HOOD_ENCODER_VALUE = 0;
	static const int RIGHT_CLOSE_SHOT_SHOOTER_AUTONOMOUS_HOOD_ENCODER_VALUE = 0;
	static const int LEFT_CLOSE_SHOT_SHOOTER_AUTONOMOUS_HOOD_ENCODER_VALUE = 1250;
	static const int MIDDLE_GEAR_SHOOT_AUTONOMUS_HOOD_ENCODER_VALUE = 6750;  //  15000 Tested on Practice Bot
	static const int SIDE_GEAR_SHOOT_AUTONOMOUS_HOOD_ENCODER_VALUE = 3750;
	static const int KPA_AUTONOMOUS_HOOD_ENCODER_VALUE = 5400;

	static const int POV_NUMBER = 0;

	double povValue = 0.0;
	bool povDirectionPressed = false;

	double throttleValue = 0.0;
	bool throttleUp = false;

	static constexpr double LIFT_HANG_FULL_POWER = 1.0;
	static constexpr double LIFT_HANG_HALF_POWER = .45;
public:
	OI();

	double GetYAxis(int);
	double DeadbandJoystickValue(double);

	bool GetButtonPressed(int, int);
	double GetThrottleValue(int);
	bool GetThrottleUp(int);
	int GetAngleOfPOV(int);
	bool POVDirectionPressed(int, int);

	void ConfigureButtonMapping();

	bool GetAction(int joyException, int ButtonException);

	static const int LEFT_DRIVE_JOYSTICK = 0;
	static const int RIGHT_DRIVE_JOYSTICK = 1;
	static const int MANIPULATOR_JOYSTICK = 2;

	static const int TRIGGER_BUTTON = 1;
	static const int THUMB_BUTTON = 2;

	static const int TOP_POV = 0;
	static const int RIGHT_POV = 1;
	static const int BOTTOM_POV = 2;
	static const int LEFT_POV = 3;

	static const int GEAR_HOLDER_UPWARDS_BUTTON = 6;
	static const int GEAR_HOLDER_DOWNWARDS_BUTTON = 4;
};

#endif  // OI_H
