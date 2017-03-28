#include "CameraServo.h"
#include "../RobotMap.h"
#include "Commands/DriveCameraServo.h"

CameraServo::CameraServo() : Subsystem("CameraServo") {

}

void CameraServo::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
	//SetDefaultCommand(new DriveCameraServo());
}

void CameraServo::InitializeCameraServo() {
	//cameraServo = new frc::Servo(CAMERA_SERVO_PWM_PORT);
}

void CameraServo::SetCameraServoValue(double servoPosition) {
	//cameraServo->SetAngle(servoPosition);
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
