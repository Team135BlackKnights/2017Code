#include "OI.h"

#include <WPILib.h>
#include "Commands/DriveAgitator.h"
#include "Commands/DriveCollection.h"
#include "Commands/DriveGearHolder.h"
#include "Commands/DriveLiftHang.h"
#include "Commands/DriveShooterHood.h"
#include "Commands/Aimbot.h"

OI::OI() {
	// Process operator interface input here.

	for (int i = 0; i < NUM_OF_JOYSTICKS; i++) {
		joystick[i] = new Joystick(i);
		for (int j = 1; j <= NUM_OF_BUTTONS; j++) {
			joystickButton[i][j] = new JoystickButton(joystick[i], j);
		}
	}

	this->ConfigureButtonMapping();
}

double OI::GetYAxis(int joystickNumber) {
	joystickYAxisValue = (-1 * joystick[joystickNumber]->GetAxis(frc::Joystick::AxisType::kYAxis));
	return this->DeadbandJoystickValue(joystickYAxisValue);
}

double OI::DeadbandJoystickValue(double joystickValue) {
	if (joystickValue < JOYSTICK_DEADBAND_VALUE && joystickValue > -JOYSTICK_DEADBAND_VALUE) {
		deadbandJoystickValue = 0.0;
	}
	else {
		deadbandJoystickValue = joystickValue;
	}
	return deadbandJoystickValue;
}

bool OI::GetButtonPressed(int joystickNumber, int joystickButtonNumber) {
	return joystickButton[joystickNumber][joystickButtonNumber]->Get();
}

void OI::ConfigureButtonMapping() {
	joystickButton[MANIPULATOR_JOYSTICK][AGITATOR_FORWARD_BUTTON]->WhileHeld(new DriveAgitator(true));
	joystickButton[MANIPULATOR_JOYSTICK][AGITATOR_BACKWARDS_BUTTON]->WhileHeld(new DriveAgitator(false));

	joystickButton[MANIPULATOR_JOYSTICK][COLLECTION_FORWARD_BUTTON]->WhileHeld(new DriveCollection(true));
	joystickButton[MANIPULATOR_JOYSTICK][COLLECTION_BACKWARDS_BUTTON]->WhileHeld(new DriveCollection(false));

	joystickButton[MANIPULATOR_JOYSTICK][GEAR_HOLDER_UPWARDS_BUTTON]->WhileHeld(new DriveGearHolder(true));
	joystickButton[MANIPULATOR_JOYSTICK][GEAR_HOLDER_DOWNWARDS_BUTTON]->WhileHeld(new DriveGearHolder(false));

	joystickButton[RIGHT_DRIVE_JOYSTICK][TRIGGER_BUTTON]->WhileHeld(new DriveLiftHang());

	joystickButton[MANIPULATOR_JOYSTICK][SHOOTER_HOOD_INCREASE_ANGLE_BUTTON]->WhileHeld(new DriveShooterHood(true));
	joystickButton[MANIPULATOR_JOYSTICK][SHOOTER_HOOD_DECREASE_ANGLE_BUTTON]->WhileHeld(new DriveShooterHood(false));

	//Just for now
	joystickButton[LEFT_DRIVE_JOYSTICK][10]->WhenPressed(new AimBot(0));
}

bool OI::GetAction(int JoyException, int ButtonException)
{
	for(int i = 0; i < NUM_OF_JOYSTICKS-1; i++)
	{
		if(abs(joystick[i]->GetY()) > .15)
			return true;
		for(int j = 0; j < NUM_OF_BUTTONS-1; j++)
		{
				if(i == JoyException && j == ButtonException)
					continue;
				if(joystickButton[i][j]->Get())
					return true;
			}
	}
	return false;
}
