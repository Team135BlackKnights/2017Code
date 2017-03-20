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
	digitalInputFromArduino = new frc::DigitalInput(INPUT_PORT);
}

void Lidar::SendArduinoDigitalSignal(bool signal) {
	digitalOutputToArduino->Set(signal);
}

bool Lidar::GetArduinoDigitalSignal() {
	return digitalInputFromArduino->Get();
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
