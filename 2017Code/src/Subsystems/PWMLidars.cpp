#include "PWMLidars.h"
#include "../RobotMap.h"

PWMLidars::PWMLidars() : Subsystem("PWMLidars") {

}

void PWMLidars::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

void PWMLidars::InitializePWMLidars() {
	leftLidarMonitor = new frc::PWM(LEFT_LIDAR_MONITOR_PORT);
	rightLidarMonitor = new frc::PWM(RIGHT_LIDAR_MONITOR_PORT);

	leftLidarTrigger = new frc::DigitalOutput(LEFT_LIDAR_TRIGGER_PORT);
	rightLidarTrigger = new frc::DigitalOutput(RIGHT_LIDAR_TRIGGER_PORT);
}

void PWMLidars::StartReceivingLidarValues(bool rightLidar) {
	if (rightLidar == RIGHT_LIDAR) {
		rightLidarTrigger->Set(false);
	}
	else if (rightLidar == LEFT_LIDAR) {
		leftLidarTrigger->Set(false);
	}
}

double PWMLidars::GetLidarPWMValue(bool rightLidar) {
	if (rightLidar == RIGHT_LIDAR) {
		rawPWMValue = rightLidarMonitor->GetRaw();
	}
	else if (rightLidar == LEFT_LIDAR) {
		rawPWMValue = leftLidarMonitor->GetRaw();
	}
	lidarValueCM = this->ConvertPulseWidthToLidarValueCM(rawPWMValue);

	return lidarValueCM;
}

double PWMLidars::ConvertPulseWidthToLidarValueCM(int pulseWidth) {
	return (pulseWidth * CONVERT_PULSE_WIDTH_TO_LIDAR_VALUE_CM);
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
