#include "AutoGearOnPeg.h"

AutoGearOnPeg::AutoGearOnPeg() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get());
	Requires(CommandBase::ultrasonicSensor.get());
	Requires(CommandBase::gearHolder.get());

	timer = new frc::Timer();
}

// Called just before this Command runs the first time
void AutoGearOnPeg::Initialize() {
	startMovingTowardsGear = true;
	startPuttingGearOnPeg = false;
	initializeTimerLimitSwitch = false;
	gearHolderDown = false;
	gearOnPeg = false;
	timer->Reset();
	timer->Start();

	startMovingRobot = true;
	retryGearLineUp = false;
	moveGearHolderDown = false;
	stopGearHolderDown = false;
	startTimerForDriveTrainRPMThreshold = false;
	waitingForDriveTrainRPMConfigure = false;

	initializeTimerForRamIntoAirship = false;
	rammedIntoAirship = false;
	initializeTimeWaitBeforeUnRamming = false;
	waitTimeForUnRammming = false;
	initializeTimerToUnRamFromAirship = false;

	moveGearHolderUpDownCounter = 1;
}

// Called repeatedly when this Command is scheduled to run
void AutoGearOnPeg::Execute() {
	ultrasonicValue = CommandBase::ultrasonicSensor->GetUltrasonicSensorValueInches(UltrasonicSensor::RIGHT_ULTRASONIC_SENSOR);
	timerValue = timer->Get();

	if (startMovingTowardsGear) {
		if (ultrasonicValue > DISTANCE_AWAY_FROM_AIRSHIP_TO_DROP_GEAR_IN) {
			rightDriveTrainEncoderRPM = (fabs(CommandBase::driveTrain->GetEncoderRPM(DriveTrain::RIGHT_SIDE_ENCODER)));
			if ((rightDriveTrainEncoderRPM < DRIVE_TRAIN_ENCODER_RPM_THRESHOLD && startMovingRobot == false) || retryGearLineUp) {
				if (retryGearLineUp == false) {
					initialRightEncoderDistance = CommandBase::driveTrain->GetDistance(DriveTrain::RIGHT_SIDE_ENCODER);
					desiredRightEncoderDistance = (initialRightEncoderDistance + DISTANCE_TO_MOVE_AWAY_FROM_GEAR);
					CommandBase::driveTrain->DriveTank(DRIVE_TRAIN_MOTOR_POWER, DRIVE_TRAIN_MOTOR_POWER);
					retryGearLineUp = true;
				}
				else if (retryGearLineUp) {
					currentRightEncoderDistance = CommandBase::driveTrain->GetDistance(DriveTrain::RIGHT_SIDE_ENCODER);
					if (currentRightEncoderDistance >= desiredRightEncoderDistance) {
						CommandBase::driveTrain->DriveTank(0.0, 0.0);
						moveGearHolderDown = true;
						retryGearLineUp = false;
						startMovingRobot = true;
						startTimerForDriveTrainRPMThreshold = false;
					}
					else {
						CommandBase::driveTrain->DriveTank(DRIVE_TRAIN_MOTOR_POWER, DRIVE_TRAIN_MOTOR_POWER);
					}
				}
			}
			else {
				if (moveGearHolderDown) {
					if (stopGearHolderDown == false) {
						timer->Reset();
						timer->Start();
						if ((moveGearHolderUpDownCounter % 2) == 1) {
							gearHolderMotorPower = (-1 * GEAR_HOLDER_MOTOR_POWER);
						}
						else if ((moveGearHolderUpDownCounter % 2) == 0) {
							gearHolderMotorPower = (GEAR_HOLDER_MOTOR_POWER + .25);
						}
						CommandBase::gearHolder->DriveGearHolderMotor(gearHolderMotorPower);
						moveGearHolderUpDownCounter++;
						stopGearHolderDown = true;
					}
					else if (stopGearHolderDown && timerValue < TIME_TO_LOWER_GEAR_HOLDER) {
						CommandBase::gearHolder->DriveGearHolderMotor(gearHolderMotorPower);
					}
					else if (stopGearHolderDown && timerValue >= TIME_TO_LOWER_GEAR_HOLDER) {
						CommandBase::gearHolder->DriveGearHolderMotor(0.0);
						timer->Reset();
						timer->Stop();
						stopGearHolderDown = false;
						moveGearHolderDown = false;
					}
				}
				else {
					if (rightDriveTrainEncoderRPM >= DRIVE_TRAIN_ENCODER_RPM_THRESHOLD || waitingForDriveTrainRPMConfigure) {
						if (startTimerForDriveTrainRPMThreshold == false) {
							timer->Reset();
							timer->Start();
							startMovingRobot = true;
							waitingForDriveTrainRPMConfigure = true;
							startTimerForDriveTrainRPMThreshold = true;
						}
						else if (startTimerForDriveTrainRPMThreshold) {
							if (timerValue >= TIME_ROBOT_IS_ABOVE_DRIVE_TRAIN_RPM_THRESHOLD) {
								startMovingRobot = false;
								waitingForDriveTrainRPMConfigure = false;
							}
							else {
								startMovingRobot = true;
								waitingForDriveTrainRPMConfigure = true;
							}
						}
					}
					else {
						startMovingRobot = true;
					}
					CommandBase::driveTrain->DriveTank(-DRIVE_TRAIN_MOTOR_POWER, -DRIVE_TRAIN_MOTOR_POWER);
				}
			}
		}
		else if (ultrasonicValue <= DISTANCE_AWAY_FROM_AIRSHIP_TO_DROP_GEAR_IN) {
			CommandBase::driveTrain->DriveTank(0.0, 0.0);
			startMovingTowardsGear = false;
			startPuttingGearOnPeg = true;
		}
	}

	if (startPuttingGearOnPeg) {
		if (initializeTimerLimitSwitch == false) {
			timer->Stop();
			timer->Reset();
			timer->Start();
			initializeTimerLimitSwitch = true;
		}

		timerValue = timer->Get();

		lowerLimitSwitchValue = CommandBase::gearHolder->GetLimitSwitchValue(GearHolder::LOWER_LIMIT_SWITCH_PORT);
		if (lowerLimitSwitchValue || (timerValue >= WAIT_TIME_FOR_LIMIT_SWITCH_TO_LOWER)) {
			CommandBase::gearHolder->DriveGearHolderMotor(0.0);
			startPuttingGearOnPeg = false;
			gearHolderDown = true;
			/*if (RAM_INTO_GEAR_PEG == false) {
				gearOnPeg = true;
			} */
		}
		else {
			CommandBase::gearHolder->DriveGearHolderMotor(-GEAR_HOLDER_MOTOR_POWER);
		}
	}

	if (RAM_INTO_GEAR_PEG) {
		if (gearHolderDown && rammedIntoAirship == false) {
			if (initializeTimerForRamIntoAirship == false) {
				timer->Stop();
				timer->Reset();
				timer->Start();
				initializeTimerForRamIntoAirship = true;
			}

			timerValue = timer->Get();

			if (timerValue >= TIME_TO_RAM_INTO_AIRSHIP) {
				CommandBase::driveTrain->DriveTank(0.0, 0.0);
				timer->Stop();
				timer->Reset();
				rammedIntoAirship = true;
			}
			else {
				CommandBase::driveTrain->DriveTank(RAMMING_MOTOR_POWER, RAMMING_MOTOR_POWER);
				rammedIntoAirship = false;
			}
		}
		else if (rammedIntoAirship) {
			if (initializeTimeWaitBeforeUnRamming == false) {
				timer->Stop();
				timer->Reset();
				timer->Start();
				initializeTimeWaitBeforeUnRamming = true;
			}
			else if (initializeTimeWaitBeforeUnRamming && waitTimeForUnRammming == false) {
				timerValue = timer->Get();
				if (timerValue >= TIME_TO_WAIT_BEFORE_UNRAMMING) {
					waitTimeForUnRammming = true;
				}
				CommandBase::driveTrain->DriveTank(0.0, 0.0);
			}

			if (waitTimeForUnRammming) {
				if (initializeTimerToUnRamFromAirship == false) {
					timer->Stop();
					timer->Reset();
					timer->Start();
					initializeTimerToUnRamFromAirship = true;
				}

				timerValue = timer->Get();
				if (timerValue >= TIME_TO_REVERSE_FROM_RAMMING_INTO_AIRSHIP) {
					CommandBase::driveTrain->DriveTank(0.0, 0.0);
					timer->Stop();
					timer->Reset();
					gearOnPeg = true;
				}
				else {
					CommandBase::driveTrain->DriveTank(UNRAMMING_MOTOR_POWER, UNRAMMING_MOTOR_POWER);
					gearOnPeg = false;
				}
			}
		}
	}
}

// Make this return true when this Command no longer needs to run execute()
bool AutoGearOnPeg::IsFinished() {
	return gearOnPeg;
}

// Called once after isFinished returns true
void AutoGearOnPeg::End() {
	std::cout << "Done with Auto Gear On Peg" << std::endl;
	startMovingTowardsGear = true;
	startPuttingGearOnPeg = false;
	initializeTimerLimitSwitch = false;
	gearHolderDown = false;
	gearOnPeg = false;
	CommandBase::driveTrain->DriveTank(0.0, 0.0);
	CommandBase::gearHolder->DriveGearHolderMotor(0.0);
	startMovingRobot = true;
	retryGearLineUp = false;
	moveGearHolderDown = false;
	stopGearHolderDown = false;
	startTimerForDriveTrainRPMThreshold = false;
	waitingForDriveTrainRPMConfigure = false;
	initializeTimerForRamIntoAirship = false;
	rammedIntoAirship = false;
	initializeTimeWaitBeforeUnRamming = false;
	waitTimeForUnRammming = false;
	initializeTimerToUnRamFromAirship = false;

	moveGearHolderUpDownCounter = 1;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutoGearOnPeg::Interrupted() {
	End();
}
