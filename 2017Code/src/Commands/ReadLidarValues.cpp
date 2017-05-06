#include "ReadLidarValues.h"
#include <Timer.h>

ReadLidarValues::ReadLidarValues() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::lidars.get());
}

// Called just before this Command runs the first time
void ReadLidarValues::Initialize() {
	configuredLidar = false;
}

// Called repeatedly when this Command is scheduled to run
void ReadLidarValues::Execute() {
	if (turnLidarOn == false) {
		CommandBase::lidars->TurnLidarOnOff(Lidars::TURN_LIDAR_ON);
		turnLidarOn = true;
	}

	lidarPowerEnabledButtonPressed = oi->GetButtonPressed(OI::LEFT_DRIVE_JOYSTICK, 8);
	if (lidarPowerEnabledButtonPressed) {
		CommandBase::lidars->TurnLidarOnOff(Lidars::TURN_LIDAR_OFF);
	}
	else {
		CommandBase::lidars->TurnLidarOnOff(Lidars::TURN_LIDAR_ON);
	}

	if (openFrontLidarChannel == false) {
		CommandBase::lidars->OpenLidarChannelOnMultiplexer(Lidars::VALUE_TO_OPEN_FRONT_LIDAR_CHANNEL_6);
		openFrontLidarChannel = true;
	}

	/* if (turnOffDetectorBiasBetweenLidarAcquisitions == false) {
		CommandBase::lidars->TurnOffDetectorBiasBetweenLidarAcquisitions();
		turnOffDetectorBiasBetweenLidarAcquisitions = true;
	} */

	if (configuredLidar == false) {
		CommandBase::lidars->ConfigureLidar();
		configuredLidar = true;
	}
	else if (configuredLidar) {
		lidarUpperByte = CommandBase::lidars->GetUpperByte();
		frc::Wait(.002);
		lidarLowerByte = CommandBase::lidars->GetLowerByte();
		lidarValueIN = CommandBase::lidars->GetLidarValue(lidarLowerByte, lidarUpperByte, Lidars::DISTANCE_UNIT_ARRAY[Lidars::INCHES]);
		configuredLidar = false;
	}

	std::cout << "Lidar Value: " << lidarValueIN << std::endl;

	frc::SmartDashboard::PutNumber("Front Lidar Value IN:", lidarValueIN);
}

// Make this return true when this Command no longer needs to run execute()
bool ReadLidarValues::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ReadLidarValues::End() {
	CommandBase::lidars->TurnLidarOnOff(Lidars::TURN_LIDAR_OFF);
	turnLidarOn = false;
	openFrontLidarChannel = false;
	turnOffDetectorBiasBetweenLidarAcquisitions = false;
	configuredLidar = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ReadLidarValues::Interrupted() {
	End();
}
