#ifndef PWMLidars_H
#define PWMLidars_H

#include <Commands/Subsystem.h>
#include <PWM.h>
#include <DigitalOutput.h>

class PWMLidars : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

	frc::PWM* lidarMonitor;
	frc::DigitalOutput* lidarTrigger;

	static const int LIDAR_MONITOR_PWM_PORT = 6;
	static const int LIDAR_DIO_PORT = 3;

	static constexpr double CONVERT_PULSE_WIDTH_TO_LIDAR_VALUE_CM = (1.0/10.0);

	int rawPWMValue = 0;
	double lidarValueCM = 0.0;

public:
	PWMLidars();
	void InitDefaultCommand();

	void InitializePWMLidars();

	void StartReceivingLidarValues();
	double GetLidarPWMValue();

	double ConvertPulseWidthToLidarValueCM(int);
};

#endif  // PWMLidars_H
