#include "DriveHoodAndShooterWithLidar.h"

DriveHoodAndShooterWithLidar::DriveHoodAndShooterWithLidar() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::lidars.get());
	Requires(CommandBase::shooter.get());
	Requires(CommandBase::shooterHood.get());
}

// Called just before this Command runs the first time
void DriveHoodAndShooterWithLidar::Initialize() {
	CommandBase::shooter->ConfigureShooterPID();
	configurePID = true;
	configureVoltageMode = false;

	CommandBase::shooter->SelectPIDProfileSlot(Shooter::FAR_SHOT_PID_VALUES);
	selectedPIDSlot = true;
}

// Called repeatedly when this Command is scheduled to run
void DriveHoodAndShooterWithLidar::Execute() {
	if (configurePID == false) {
		CommandBase::shooter->ConfigureShooterPID();
		configurePID = true;
		configureVoltageMode = false;
	}

	if (selectedPIDSlot == false) {
		CommandBase::shooter->SelectPIDProfileSlot(Shooter::FAR_SHOT_PID_VALUES);
		selectedPIDSlot = true;
	}

	CommandBase::shooter->DriveShooterMotor(Shooter::SHOOTER_SETPOINT_RPM_FAR_SHOT);

	if (turnOnLidar == false) {
		CommandBase::lidars->TurnLidarOnOff(Lidars::TURN_LIDAR_ON);
		turnOnLidar = true;
	}
	else if (configureLidar == false) {
		CommandBase::lidars->ConfigureLidar();
		configureLidar = true;
	}
	else if (configureLidar) {
		lidarUpperByte = CommandBase::lidars->GetUpperByte();
		frc::Wait(.002);
		lidarLowerByte = CommandBase::lidars->GetLowerByte();
		lidarValueIN = CommandBase::lidars->GetLidarValue(lidarLowerByte, lidarUpperByte, Lidars::DISTANCE_UNIT_ARRAY[Lidars::INCHES]);
		configureLidar = false;
	}

	if (openI2CMultiplexerChannelForLidar == false) {
		CommandBase::lidars->OpenLidarChannelOnMultiplexer(Lidars::VALUE_TO_OPEN_FRONT_LIDAR_CHANNEL_6);
		openI2CMultiplexerChannelForLidar = true;
	}

	if (startGettingDesiredHoodEncoderValues == false) {
		if (lidarValueIN > 0.0) {
			startGettingDesiredHoodEncoderValues= true;
		}
	}
	else if (startGettingDesiredHoodEncoderValues) {
		if (intializeDesiredHoodEncoderPosition) {
			desiredHoodEncoderValue = CommandBase::shooterHood->GetDesiredHoodEncoderPositionGivenLidarValue(lidarValueIN);
		}

		if (hoodAtDesiredEncoderPosition == false) {
			hoodAtDesiredEncoderPosition= CommandBase::shooterHood->DriveShooterHoodToDesiredEncoderValue(desiredHoodEncoderValue);
		}
		else if (hoodAtDesiredEncoderPosition) {
			CommandBase::shooterHood->DriveShooterHoodMotor(0.0);
		}
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveHoodAndShooterWithLidar::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void DriveHoodAndShooterWithLidar::End() {
	CommandBase::shooter->ConfigureShooterVoltageMode();
	configureVoltageMode = true;
	configurePID = false;
	CommandBase::shooter->DriveShooterMotor(0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveHoodAndShooterWithLidar::Interrupted() {

}
