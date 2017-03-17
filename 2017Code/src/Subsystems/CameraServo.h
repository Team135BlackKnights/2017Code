#ifndef CameraServo_H
#define CameraServo_H

#include <Commands/Subsystem.h>
#include <Servo.h>

class CameraServo : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

	frc::Servo* cameraServo;
	static const int CAMERA_SERVO_PWM_PORT = 5;

public:
	CameraServo();
	void InitDefaultCommand();

	void InitializeCameraServo();
	void SetCameraServoValue(double);
	static constexpr double SERVO_FRONT_POSITION = 180.0;
	static constexpr double SERVO_BACK_POSITION = 0.0;
};

#endif  // CameraServo_H