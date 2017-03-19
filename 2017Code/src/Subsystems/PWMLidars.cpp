#include "PWMLidars.h"
#include "../RobotMap.h"
#include "Commands/ReadPWMLidarValue.h"

PWMLidars::PWMLidars() : Subsystem("PWMLidars") {

}

void PWMLidars::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
	//SetDefaultCommand(new ReadPWMLidarValue());
}

void PWMLidars::InitializePWMLidars() {
	lidarMonitor = new frc::PWM(LIDAR_MONITOR_PWM_PORT);
	lidarTrigger = new frc::DigitalOutput(LIDAR_DIO_PORT);
}

void PWMLidars::StartReceivingLidarValues() {
	lidarTrigger->Set(false);
}

double PWMLidars::GetLidarPWMValue(int distanceUnits) {
	rawPWMValue = lidarMonitor->GetRaw();
	std::cout << "Raw PWM Value: " << rawPWMValue << std::endl;
	lidarValueCM = this->ConvertPulseWidthToLidarValueCM(rawPWMValue);
	lidarValueIN = this->ConvertCentimetersToInches(lidarValueCM);
	lidarValueM = this->ConvertCentimetersToMeters(lidarValueCM);
	if (distanceUnits == CENTIMETERS) {
		returnLidarValue = lidarValueCM;
	}
	else if (distanceUnits == METERS) {
		returnLidarValue = lidarValueM;
	}
	else if (distanceUnits == INCHES) {
		returnLidarValue = lidarValueIN;
	}
	return returnLidarValue;
}

double PWMLidars::ConvertPulseWidthToLidarValueCM(int pulseWidth) {
	return (((double)pulseWidth) * CONVERT_PULSE_WIDTH_TO_LIDAR_VALUE_CM);
}


double PWMLidars::ConvertCentimetersToInches(double lidarValue_CM) {
	return (lidarValue_CM/2.54);
}

double PWMLidars::ConvertCentimetersToMeters(double lidarValue_CM) {
	return (lidarValue_CM/100.0);
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
