#ifndef Lidar_H
#define Lidar_H

#include <Commands/Subsystem.h>
#include <DigitalOutput.h>
#include <DigitalInput.h>

class Lidar : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

	frc::DigitalOutput* digitalOutputToArduino;
	frc::DigitalInput* digitalInputFromArduino;

	static const int OUTPUT_PORT = 2;
	static const int INPUT_PORT = 3;

public:
	Lidar();
	void InitDefaultCommand();

	void InitializeLidarDIO();

	void SendArduinoDigitalSignal(bool);
	bool GetArduinoDigitalSignal();
};

#endif  // Lidar_H
