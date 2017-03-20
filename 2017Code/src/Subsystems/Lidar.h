#ifndef Lidar_H
#define Lidar_H

#include <Commands/Subsystem.h>
#include <DigitalOutput.h>
#include <AnalogInput.h>

class Lidar : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

	frc::DigitalOutput* digitalOutputToArduino;
	frc::AnalogInput* analogInputFromArduino;

	double analogInputVoltage = 0.0;
	static constexpr double ANALOG_INPUT_VOLTAGE_THRESHOLD = 2.5;
	bool analogInputHigh = false;

	static const int OUTPUT_PORT = 2;
	static const int INPUT_PORT = 0;

public:
	Lidar();
	void InitDefaultCommand();

	void InitializeLidarDIO();

	void SendArduinoDigitalSignal(bool);
	bool GetArduinoAnalogSignal();
};

#endif  // Lidar_H
