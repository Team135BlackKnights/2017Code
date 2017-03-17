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
	lidarTrigger = new frc::DigitalOutput(lidarTrigger);
}

void PWMLidars::StartReceivingLidarValues() {
	lidarTrigger->Set(false);
}

double PWMLidars::GetLidarPWMValue() {
	rawPWMValue = lidarMonitor->GetRaw();
	lidarValueCM = this->ConvertPulseWidthToLidarValueCM(rawPWMValue);
	return lidarValueCM;
}

double PWMLidars::ConvertPulseWidthToLidarValueCM(int pulseWidth) {
	return (((double)pulseWidth) * CONVERT_PULSE_WIDTH_TO_LIDAR_VALUE_CM);
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
