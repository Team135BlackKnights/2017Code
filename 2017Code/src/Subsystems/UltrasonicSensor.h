#ifndef UltrasonicSensor_H
#define UltrasonicSensor_H

#include <Commands/Subsystem.h>
#include <Ultrasonic.h>
#include <DigitalOutput.h>
#include <DigitalInput.h>

class UltrasonicSensor : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

	frc::Ultrasonic* ultrasonicSensor;
	frc::DigitalOutput* pingSignal;
	frc::DigitalInput* echoSignal;

	static const int DIO_PING_PORT = 9;
	static const int DIO_ECHO_PORT = 8;

public:
	UltrasonicSensor();
	void InitDefaultCommand();

	void InitializeUltrasonicSensor();

	double GetUltrasonicSensorValueInches();
};

#endif  // UltrasonicSensor_H
