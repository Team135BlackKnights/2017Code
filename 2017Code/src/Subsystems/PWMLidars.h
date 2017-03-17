#ifndef PWMLidars_H
#define PWMLidars_H

#include <Commands/Subsystem.h>
#include <PWM.h>
#include <DigitalOutput.h>

class PWMLidars : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

	static const int LEFT_LIDAR_MONITOR_PORT = 3;
	static const int RIGHT_LIDAR_MONITOR_PORT = 4;

	static const int LEFT_LIDAR_TRIGGER_PORT = 3;
	static const int RIGHT_LIDAR_TRIGGER_PORT = 4;

	frc::PWM* leftLidarMonitor;
	frc::PWM* rightLidarMonitor;

	frc::DigitalOutput* leftLidarTrigger;
	frc::DigitalOutput* rightLidarTrigger;

	static const bool RIGHT_LIDAR = true;
	static const bool LEFT_LIDAR = !RIGHT_LIDAR;

	int rawPWMValue = 0;
	double lidarValueCM = 0.0;

	static constexpr double CONVERT_PULSE_WIDTH_TO_LIDAR_VALUE_CM = (1.0/10.0);

public:
	PWMLidars();
	void InitDefaultCommand();

	void InitializePWMLidars();

	void StartReceivingLidarValues(bool);
	double GetLidarPWMValue(bool);

	double ConvertPulseWidthToLidarValueCM(int);
};

#endif  // PWMLidars_H
