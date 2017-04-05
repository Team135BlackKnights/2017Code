#include "AutoRotateRobotForGearPeg.h"
#include <iostream>

AutoRotateRobotForGearPeg::AutoRotateRobotForGearPeg(double motorPower, double desiredAngleToRotateRobot) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get());

	this->desiredAngleToRotateRobot = desiredAngleToRotateRobot;
	this->motorPower = motorPower;

	timer = new frc::Timer();
}

// Called just before this Command runs the first time
void AutoRotateRobotForGearPeg::Initialize() {
	CommandBase::driveTrain->ZeroGyroAngle();
	zeroedGyro = true;

	timer->Reset();
	timer->Start();
	initializeFirstRotationTimer = true;
	initializeFirstTimeWait = false;
	initializeSecondRotationTimer = false;
	initializeSecondTimeWait = false;
	initializeThirdRotationTimer = false;

	firstRotationComplete = false;
	waitBetweenFirstAndSecondRotations = false;
	secondRotationComplete = false;
	waitBetweenSecondAndThirdRotations = false;
	thirdRotationComplete = false;

	initializedNewAutoRotateRobotFirstRotation = false;
	initializedNewAutoRotateRobotSecondRotation = false;
	initializedNewAutoRotateRobotThirdRotation = false;

	CommandBase::driveTrain->is_aiming = true;
}

// Called repeatedly when this Command is scheduled to run
void AutoRotateRobotForGearPeg::Execute() {
	if (zeroedGyro == false) {
		CommandBase::driveTrain->ZeroGyroAngle();
		zeroedGyro = true;
	}

	if (firstRotationComplete == false) {
		if (initializeFirstRotationTimer == false) {
			timer->Stop();
			timer->Reset();
			timer->Start();
			initializeFirstRotationTimer = true;
		}

		timerValue = timer->Get();

		if (initializeFirstRotationTimer) {
			if (timerValue >= FIRST_ROTATION_TIMEOUT) {
				CommandBase::driveTrain->DriveTank(0.0, 0.0);
				firstRotationComplete = true;
			}
			else {
				firstRotationComplete = CommandBase::driveTrain->AutoRotateRobot(this->motorPower, this->desiredAngleToRotateRobot, TURN_LEFT, initializedNewAutoRotateRobotFirstRotation);
				if (initializedNewAutoRotateRobotFirstRotation == false) {
					initializedNewAutoRotateRobotFirstRotation = true;
				}
				std::cout << "First Rotation Complete: " << firstRotationComplete << std::endl;
			}
		}
	}
	else if (firstRotationComplete && waitBetweenFirstAndSecondRotations == false) {
		std::cout << "First Rotation Complete" << std::endl;
		if (initializeFirstTimeWait == false) {
			timer->Stop();
			timer->Reset();
			timer->Start();
			initializeFirstTimeWait = true;
		}

		timerValue = timer->Get();

		CommandBase::driveTrain->DriveTank(0.0, 0.0);
		if (timerValue >= TIME_TO_WAIT_BETWEEN_FIRST_AND_SECOND_ROTATIONS) {
			waitBetweenFirstAndSecondRotations = true;
		}
	}
	else if (firstRotationComplete && waitBetweenFirstAndSecondRotations && secondRotationComplete == false) {
		if (initializeSecondRotationTimer == false) {
			angleTurnedDuringFirstRotation = fabs(driveTrain->GetGyroAngle());
			angleToTurnForSecondRotation = (2.0 * angleTurnedDuringFirstRotation);
			timer->Stop();
			timer->Reset();
			timer->Start();
			initializeSecondRotationTimer = true;
		}

		timerValue = timer->Get();

		if (initializeSecondRotationTimer) {
			if (timerValue >= SECOND_ROTATION_TIMEOUT) {
				CommandBase::driveTrain->DriveTank(0.0, 0.0);
				secondRotationComplete = true;
			}
			else {
				secondRotationComplete = CommandBase::driveTrain->AutoRotateRobot(this->motorPower, angleToTurnForSecondRotation, TURN_RIGHT, initializedNewAutoRotateRobotSecondRotation);
				if (initializedNewAutoRotateRobotSecondRotation == false) {
					initializedNewAutoRotateRobotSecondRotation = true;
				}
			}
		}

	}
	else if (secondRotationComplete && waitBetweenSecondAndThirdRotations == false) {
		std::cout << "Second Rotation Complete" << std::endl;
		if (initializeSecondTimeWait == false) {
			timer->Stop();
			timer->Reset();
			timer->Start();
			initializeSecondTimeWait = true;
		}

		timerValue = timer->Get();

		CommandBase::driveTrain->DriveTank(0.0, 0.0);

		if (timerValue >= TIME_TO_WAIT_BETWEEN_SECOND_AND_THIRD_ROTATIONS) {
			waitBetweenSecondAndThirdRotations = true;
		}
	}
	else if (secondRotationComplete && waitBetweenSecondAndThirdRotations && thirdRotationComplete == false) {
		std::cout << "Third Rotation Running" << std::endl;
		if (initializeThirdRotationTimer == false) {
			angleToTurnForThirdRotation = fabs(driveTrain->GetGyroAngle());
			std::cout << "Desired Angle: " << angleToTurnForThirdRotation << std::endl;
			timer->Stop();
			timer->Reset();
			timer->Start();
			initializeThirdRotationTimer = true;
		}

		timerValue = timer->Get();
		std::cout << "Third Rotation Timer Value: " << timerValue << std::endl;

		if (initializeThirdRotationTimer) {
			if (timerValue >= THIRD_ROTATION_TIMEOUT) {
				CommandBase::driveTrain->DriveTank(0.0, 0.0);
				thirdRotationComplete = true;
				std::cout << "TImeeerrrreerrr" << std::endl;
			}
			else {
				thirdRotationComplete = CommandBase::driveTrain->AutoRotateRobot(this->motorPower, angleToTurnForThirdRotation, TURN_LEFT, initializedNewAutoRotateRobotThirdRotation);
				std::cout << "Third Rotation Complete? " << thirdRotationComplete << std::endl;
				if (initializedNewAutoRotateRobotThirdRotation == false) {
					initializedNewAutoRotateRobotThirdRotation = true;
				}
			}
		}
	}
}

// Make this return true when this Command no longer needs to run execute()
bool AutoRotateRobotForGearPeg::IsFinished() {
	return thirdRotationComplete;
}

// Called once after isFinished returns true
void AutoRotateRobotForGearPeg::End() {
	std::cout << "Third Rotation Complete" << std::endl;
	CommandBase::driveTrain->DriveTank(0.0, 0.0);

	zeroedGyro = false;

	initializeFirstRotationTimer = false;
	initializeFirstTimeWait = false;
	initializeSecondRotationTimer = false;
	initializeSecondTimeWait = false;
	initializeThirdRotationTimer = false;

	firstRotationComplete = false;
	waitBetweenFirstAndSecondRotations = false;
	secondRotationComplete = false;
	waitBetweenSecondAndThirdRotations = false;
	thirdRotationComplete = false;

	initializedNewAutoRotateRobotFirstRotation = false;
	initializedNewAutoRotateRobotSecondRotation = false;
	initializedNewAutoRotateRobotThirdRotation = false;

	CommandBase::driveTrain->is_aiming = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutoRotateRobotForGearPeg::Interrupted() {
	End();
}
