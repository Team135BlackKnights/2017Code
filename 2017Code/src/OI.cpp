#include "OI.h"

#include <WPILib.h>
#include "Commands/DriveAgitator.h"
#include "Commands/DriveCollection.h"
#include "Commands/DriveGearHolder.h"
#include "Commands/DriveLiftHang.h"
#include "Commands/DriveShooterHood.h"

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

double OI::GetThrottleValue(int joystickNumber) {
	return (-1 * joystick[joystickNumber]->GetThrottle());
}

bool OI::GetThrottleUp(int joystickNumber) {
	throttleValue = this->GetThrottleValue(joystickNumber);
	if (throttleValue >= 0) {
		throttleUp = true;
	}
	else if (throttleValue < 0) {
		throttleUp = false;
	}
	return throttleUp;
}

int OI::GetAngleOfPOV(int joystickNumber) {
	return joystick[joystickNumber]->GetPOV(POV_NUMBER);
}

bool OI::POVDirectionPressed(int joystickNumber, int povDirection) {
	povValue = this->GetAngleOfPOV(joystickNumber);

	switch(povDirection) {
	case (TOP_POV):
		if (povValue >= 315.0 || (povValue <= 45.0 && povValue != -1)) {
			povDirectionPressed = true;
		}
		else {
			povDirectionPressed = false;
		}
		break;
	case (RIGHT_POV):
		if (povValue >= 45.0 && povValue <= 135.0) {
			povDirectionPressed = true;
		}
		else {
			povDirectionPressed = false;
		}
		break;
	case (BOTTOM_POV):
		if (povValue >= 135.0 && povValue <= 225.0) {
			povDirectionPressed = true;
		}
		else {
			povDirectionPressed = false;
		}
		break;
	case (LEFT_POV):
		if (povValue >= 225.0 && povValue <= 315.0) {
			povDirectionPressed = true;
		}
		else {
			povDirectionPressed = false;
		}
		break;
	}
	return povDirectionPressed;
}

void OI::ConfigureButtonMapping() {
	joystickButton[MANIPULATOR_JOYSTICK][AGITATOR_FORWARD_BUTTON]->WhileHeld(new DriveAgitator(true));
	joystickButton[MANIPULATOR_JOYSTICK][AGITATOR_BACKWARDS_BUTTON]->WhileHeld(new DriveAgitator(false));

	joystickButton[MANIPULATOR_JOYSTICK][COLLECTION_FORWARD_BUTTON]->WhileHeld(new DriveCollection(true));
	joystickButton[MANIPULATOR_JOYSTICK][COLLECTION_BACKWARDS_BUTTON]->WhileHeld(new DriveCollection(false));

	joystickButton[RIGHT_DRIVE_JOYSTICK][TRIGGER_BUTTON]->WhileHeld(new DriveLiftHang());

	joystickButton[MANIPULATOR_JOYSTICK][SHOOTER_HOOD_DECREASE_ANGLE_BUTTON]->WhileHeld(new DriveShooterHood(true));
	joystickButton[MANIPULATOR_JOYSTICK][SHOOTER_HOOD_INCREASE_ANGLE_BUTTON]->WhileHeld(new DriveShooterHood(false));
}
