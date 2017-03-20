#include "Lidar.h"
#include "../RobotMap.h"

Lidar::Lidar() : Subsystem("Lidar") {

}

void Lidar::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

void Lidar::InitializeLidarDIO() {
	digitalOutputToArduino = new frc::DigitalOutput(OUTPUT_PORT);
	analogInputFromArduino = new frc::AnalogInput(INPUT_PORT);
}

void Lidar::SendArduinoDigitalSignal(bool signal) {
	digitalOutputToArduino->Set(signal);
}

bool Lidar::GetArduinoAnalogSignal() {
	analogInputVoltage = analogInputFromArduino->GetVoltage();
	if (analogInputVoltage > ANALOG_INPUT_VOLTAGE_THRESHOLD) {
		analogInputHigh = true;
	}
	else if (analogInputVoltage <= ANALOG_INPUT_VOLTAGE_THRESHOLD) {
		analogInputHigh = false;
	}
	return analogInputHigh;
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
