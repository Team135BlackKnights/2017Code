#include "AutonomousCommand.h"
#include "AutoGearOnPeg.h"
#include "DriveDistance.h"
#include "TurnDriveTrainAngle.h"
#include "WaitTime.h"
#include "AimBot.h"
#include "AutoDriveShooter.h"
#include "AutoDriveShooterHood.h"
#include "TurnOneSideOfRobotAngle.h"
#include "AutoGetShooterUpToSpeed.h"
#include "AutoDriveAgitator.h"
#include "DriveToGearWithLidar.h"
#include "DriveDriveTrainCertainTime.h"
#include "AutoDriveCollection.h"
#include "GetReadyForLiftHang.h"
#include "AutoRotateRobotForGearPeg.h"
#include "AimBotWithUltrasonicSensors.h"
#include "AimToGear.h"
#include "DriveBackwardsWithLidar.h"
#include "AutoMoveGearHolder.h"
#include "DriveParallelWithGuardrailWithUltrasonicSensor.h"
AutonomousCommand::AutonomousCommand() {
	// Add Commands here:
	// e.g. AddSequential(new Command1());
	//      AddSequential(new Command2());
	// these will run in order.

	// To run multiple commands at the same time,
	// use AddParallel()
	// e.g. AddParallel(new Command1());
	//      AddSequential(new Command2());
	// Command1 and Command2 will run in parallel.

	// A command group will require all of the subsystems that each member
	// would require.
	// e.g. if Command1 requires chassis, and Command2 requires arm,
	// a CommandGroup containing them would require both the chassis and the
	// arm.

	Requires(CommandBase::driveTrain.get());
	Requires(CommandBase::gearHolder.get());
	Requires(CommandBase::ultrasonicSensor.get());
	Requires(CommandBase::shooter.get());
	Requires(CommandBase::agitator.get());
	Requires(CommandBase::lidars.get());
	Requires(CommandBase::liftHang.get());
	Requires(CommandBase::collection.get());

	baseLine = Preferences::GetInstance()->GetBoolean("BaseLine", false);
	middleGear = Preferences::GetInstance()->GetBoolean("Middle Gear", false);
	middleGearShootRight = Preferences::GetInstance()->GetBoolean("Middle Gear Shoot Right", false);
	middleGearShootLeft = Preferences::GetInstance()->GetBoolean("Middle Gear Shoot Left", false);
	rightGear = Preferences::GetInstance()->GetBoolean("Right Gear", false);
	leftGear = Preferences::GetInstance()->GetBoolean("Left Gear", false);
	rightGearAndShoot = Preferences::GetInstance()->GetBoolean("Right Gear and Shoot", false);
	leftGearAndShoot = Preferences::GetInstance()->GetBoolean("Left Gear and Shoot", false);
	rightGearAndNeutralZone = Preferences::GetInstance()->GetBoolean("Right Gear and Neutral Zone", false);
	leftGearAndNeutralZone = Preferences::GetInstance()->GetBoolean("Left Gear and Neutral Zone", false);
	closeShotRightWithBaseline = Preferences::GetInstance()->GetBoolean("Close Shot Right With Baseline", false);
	closeShotLeftWithBaseline = Preferences::GetInstance()->GetBoolean("Close Shot Left With Baseline", false);
	right40KPa = Preferences::GetInstance()->GetBoolean("Right 40KPa", false);
	left40KPa = Preferences::GetInstance()->GetBoolean("Left 40KPa", false);

	if (baseLine) {
		AddSequential(new DriveDistance(BASE_LINE_PATH_DISTANCE, .7));
		AddParallel(new GetReadyForLiftHang());
	}
	else if (middleGear) {
		AddSequential(new DriveDistance(DISTANCE_FROM_ALLIANCE_WALL_TO_MIDDLE_GEAR, -.55));
		AddParallel(new GetReadyForLiftHang());
		AddSequential(new WaitTime(.25));
		AddSequential(new AutoGearOnPeg());
		AddSequential(new WaitTime(.001));
		AddParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_DOWNWARDS));
		AddSequential(new WaitTime(.6));
		AddSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG));
		//if (USING_AUTO_ROTATE_FOR_GEAR_PEG) { AddSequential(new WaitTime(.1)); AddSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG)); }
		AddSequential(new WaitTime(.25));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_1, .175));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_2, .175));
		AddParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_UPWARDS));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_3, .4));
	}
	else if (middleGearShootRight) {
		AddSequential(new DriveDistance(DISTANCE_FROM_ALLIANCE_WALL_TO_MIDDLE_GEAR, -.55));
		AddParallel(new GetReadyForLiftHang());
		AddSequential(new WaitTime(.25));
		AddSequential(new AutoGearOnPeg());
		AddSequential(new WaitTime(.001));
		AddParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_DOWNWARDS));
		AddSequential(new WaitTime(.6));
		AddSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG));
		//if (USING_AUTO_ROTATE_FOR_GEAR_PEG) { AddSequential(new WaitTime(.1)); AddSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG)); }
		AddSequential(new WaitTime(.25));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_1, .175));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_2, .175));
		AddParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_UPWARDS));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_3, .4));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_TO_SHOOT, .65));
		AddSequential(new WaitTime(.2));
		AddSequential(new TurnDriveTrainAngle(RIGHT_ANGLE_DEGREES, .65, TURN_LEFT));
		AddSequential(new WaitTime(.15));
		AddSequential(new DriveDistance(DISTANCE_TO_TRAVEL_AFTER_TURNING_TO_DRIVE_TOWARDS_BOILER, .7));
		AddParallel(new AutoGetShooterUpToSpeed(Shooter::SHOOTER_SETPOINT_RPM_FAR_SHOT));
		AddSequential(new WaitTime(.2));
		AddSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_TO_FACE_BOILDER_RIGHT_SHOOT_GEAR, .65, TURN_RIGHT));
		AddSequential(new WaitTime(.2));
		AddSequential(new AimBot(SHOOTER_CAMERA));
		AddSequential(new AutoDriveAgitator(STOP_RUNNING_COLLECTION));
	}
	else if (middleGearShootLeft) {
		AddSequential(new DriveDistance(DISTANCE_FROM_ALLIANCE_WALL_TO_MIDDLE_GEAR, -.55));
		AddParallel(new GetReadyForLiftHang());
		AddSequential(new WaitTime(.25));
		AddSequential(new AutoGearOnPeg());
		AddSequential(new WaitTime(.001));
		AddParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_DOWNWARDS));
		AddSequential(new WaitTime(.6));
		AddSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG));
		//if (USING_AUTO_ROTATE_FOR_GEAR_PEG) { AddSequential(new WaitTime(.1)); AddSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG)); }
		AddSequential(new WaitTime(.25));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_1, .175));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_2, .175));
		AddParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_UPWARDS));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_3, .4));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_TO_SHOOT, .65));
		AddSequential(new WaitTime(.2));
		AddSequential(new TurnDriveTrainAngle(RIGHT_ANGLE_DEGREES, .65, TURN_RIGHT));
		AddSequential(new WaitTime(.15));
		AddSequential(new DriveDistance(DISTANCE_TO_TRAVEL_AFTER_TURNING_TO_DRIVE_TOWARDS_BOILER, .7));
		AddParallel(new AutoGetShooterUpToSpeed(Shooter::SHOOTER_SETPOINT_RPM_FAR_SHOT));
		AddSequential(new WaitTime(.2));
		AddSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_TO_FACE_BOILDER_LEFT_SHOOT_GEAR, .65, TURN_LEFT));
		AddSequential(new WaitTime(.2));
		AddSequential(new AimBot(SHOOTER_CAMERA));
		AddSequential(new AutoDriveAgitator(STOP_RUNNING_COLLECTION));
	}
	else if (rightGear) {
		/*if (USING_LIDAR_FOR_GEAR_LINEUP) { AddSequential(new DriveBackwardsWithLidar(-.65, SIDE_GEAR_LIDAR_VALUE_TO_TRAVEL_UNTIL, DISTANCE_FROM_ALLIANCE_WALL_TO_RIGHT_GEAR_PART_1)); }
		else { AddSequential(new DriveDistance(DISTANCE_FROM_ALLIANCE_WALL_TO_RIGHT_GEAR_PART_1, -.775));
			   AddSequential(new DriveDistance(DISTANCE_FROM_ALLIANCE_WALL_TO_SIDE_GEAR_PART_2, -.45)); } */
		AddSequential(new DriveDistance(RIGHT_SIDE_GEAR_DISTANCE_PART_1, -.65));
		AddSequential(new DriveDistance(SIDE_GEAR_DISTANCE_PART_2, -.25));
		AddSequential(new DriveDistance(SIDE_GEAR_DISTANCE_PART_3, -.15));
		AddParallel(new GetReadyForLiftHang());
		AddSequential(new WaitTime(.2));
		AddSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_TO_FACE_SIDE_GEAR, .65, TURN_LEFT));
		AddSequential(new WaitTime(.1));
		AddSequential(new DriveDistance(DISTANCE_AFTER_TURNING_ONTO_SIDE_GEAR, -.4));
		AddSequential(new WaitTime(.15));
		//if (USING_GEAR_CAMERA) { AddSequential(new AimToGear(GEAR_CAMERA)); }
		AddSequential(new AutoGearOnPeg());
		AddSequential(new WaitTime(.001));
		AddParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_DOWNWARDS));
		AddSequential(new WaitTime(.6));
		AddSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG));
		//if (USING_AUTO_ROTATE_FOR_GEAR_PEG) { AddSequential(new WaitTime(.1)); AddSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG)); }
		AddSequential(new WaitTime(.25));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_1, .175));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_2, .175));
		AddParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_UPWARDS));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_3, .4));
	}
	else if (rightGearAndShoot) {
		/* if (USING_LIDAR_FOR_GEAR_LINEUP) { AddSequential(new DriveBackwardsWithLidar(-.65, SIDE_GEAR_LIDAR_VALUE_TO_TRAVEL_UNTIL, DISTANCE_FROM_ALLIANCE_WALL_TO_RIGHT_GEAR_PART_1)); }
		else { AddSequential(new DriveDistance(DISTANCE_FROM_ALLIANCE_WALL_TO_RIGHT_GEAR_PART_1, -.775));
			   AddSequential(new DriveDistance(DISTANCE_FROM_ALLIANCE_WALL_TO_SIDE_GEAR_PART_2, -.45)); } */
		AddSequential(new DriveDistance(RIGHT_SIDE_GEAR_DISTANCE_PART_1, -.65));
		AddParallel(new GetReadyForLiftHang());
		AddSequential(new DriveDistance(SIDE_GEAR_DISTANCE_PART_2, -.25));
		AddSequential(new DriveDistance(SIDE_GEAR_DISTANCE_PART_3, -.15));
		AddSequential(new WaitTime(.2));
		AddSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_TO_FACE_SIDE_GEAR, .65, TURN_LEFT));
		AddSequential(new WaitTime(.1));
		AddSequential(new DriveDistance(DISTANCE_AFTER_TURNING_ONTO_SIDE_GEAR, -.4));
		AddSequential(new WaitTime(.15));
		//if (USING_GEAR_CAMERA) { AddSequential(new AimToGear(GEAR_CAMERA)); }
		AddSequential(new AutoGearOnPeg());
		AddSequential(new WaitTime(.001));
		AddParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_DOWNWARDS));
		AddSequential(new WaitTime(.6));
		AddSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG));
		//if (USING_AUTO_ROTATE_FOR_GEAR_PEG) { AddSequential(new WaitTime(.1)); AddSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG)); }
		AddSequential(new WaitTime(.25));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_1, .175));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_2, .175));
		AddParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_UPWARDS));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_3, .4));
		AddSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_AFTER_DRIVING_AWAY_FROM_RIGHT_GEAR, .55, TURN_RIGHT));
		AddParallel(new AutoGetShooterUpToSpeed(Shooter::SHOOTER_SETPOINT_RPM_FAR_SHOT));
		AddSequential(new WaitTime(.2));
		AddSequential(new DriveDistance(DISTANCE_TO_TRAVEL_AFTER_PLACING_SIDE_GEAR, .6));
		AddSequential(new WaitTime(.15));
		AddSequential(new AimBot(SHOOTER_CAMERA));
		AddSequential(new AutoDriveAgitator(STOP_RUNNING_COLLECTION));
	}
	else if (rightGearAndNeutralZone) {
		/* if (USING_LIDAR_FOR_GEAR_LINEUP) { AddSequential(new DriveBackwardsWithLidar(-.65, SIDE_GEAR_LIDAR_VALUE_TO_TRAVEL_UNTIL, DISTANCE_FROM_ALLIANCE_WALL_TO_RIGHT_GEAR_PART_1)); }
		else { AddSequential(new DriveDistance(DISTANCE_FROM_ALLIANCE_WALL_TO_RIGHT_GEAR_PART_1, -.775));
			   AddSequential(new DriveDistance(DISTANCE_FROM_ALLIANCE_WALL_TO_SIDE_GEAR_PART_2, -.45)); } */
		AddSequential(new DriveDistance(RIGHT_SIDE_GEAR_DISTANCE_PART_1, -.65));
		AddParallel(new GetReadyForLiftHang());
		AddSequential(new DriveDistance(SIDE_GEAR_DISTANCE_PART_2, -.25));
		AddSequential(new DriveDistance(SIDE_GEAR_DISTANCE_PART_3, -.15));
		AddSequential(new WaitTime(.2));
		AddSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_TO_FACE_SIDE_GEAR, .65, TURN_LEFT));
		AddSequential(new WaitTime(.1));
		AddSequential(new DriveDistance(DISTANCE_AFTER_TURNING_ONTO_SIDE_GEAR, -.4));
		AddSequential(new WaitTime(.15));
		//if (USING_GEAR_CAMERA) { AddSequential(new AimToGear(GEAR_CAMERA)); }
		AddSequential(new AutoGearOnPeg());
		AddSequential(new WaitTime(.001));
		AddParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_DOWNWARDS));
		AddSequential(new WaitTime(.6));
		AddSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG));
		//if (USING_AUTO_ROTATE_FOR_GEAR_PEG) { AddSequential(new WaitTime(.1)); AddSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG)); }
		AddSequential(new WaitTime(.25));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_1, .175));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_2, .175));
		AddParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_UPWARDS));
		AddSequential(new DriveDistance((DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_3 + 8.0), .4));
		AddSequential(new WaitTime(.2));
		AddSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_TO_FACE_NEUTRAL_ZONE, .65, TURN_RIGHT));
		AddSequential(new DriveDistance(DISTANCE_TO_TRAVEL_TOWARDS_AND_INTO_NEUTRAL_ZONE, -.65));
	}
	else if (leftGear) {
		/*if (USING_LIDAR_FOR_GEAR_LINEUP) { AddSequential(new DriveBackwardsWithLidar(-.65, SIDE_GEAR_LIDAR_VALUE_TO_TRAVEL_UNTIL, DISTANCE_FROM_ALLIANCE_WALL_TO_LEFT_GEAR_PART_1)); }
		else { AddSequential(new DriveDistance(DISTANCE_FROM_ALLIANCE_WALL_TO_LEFT_GEAR_PART_1, -.775));
			   AddSequential(new DriveDistance(DISTANCE_FROM_ALLIANCE_WALL_TO_SIDE_GEAR_PART_2, -.45)); } */
		AddSequential(new DriveDistance(LEFT_SIDE_GEAR_DISTANCE_PART_1, -.65));
		AddParallel(new GetReadyForLiftHang());
		AddSequential(new DriveDistance(SIDE_GEAR_DISTANCE_PART_2, -.25));
		AddSequential(new DriveDistance(SIDE_GEAR_DISTANCE_PART_3, -.15));
		AddSequential(new WaitTime(.2));
		AddSequential(new TurnDriveTrainAngle((ANGLE_TO_TURN_TO_FACE_SIDE_GEAR), .65, TURN_RIGHT));
		AddSequential(new WaitTime(.1));
		AddSequential(new DriveDistance(DISTANCE_AFTER_TURNING_ONTO_SIDE_GEAR, -.4));
		AddSequential(new WaitTime(.15));
		//if (USING_GEAR_CAMERA) { AddSequential(new AimToGear(GEAR_CAMERA)); }
		AddSequential(new AutoGearOnPeg());
		AddSequential(new WaitTime(.001));
		AddParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_DOWNWARDS));
		AddSequential(new WaitTime(.6));
		AddSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG));
		//if (USING_AUTO_ROTATE_FOR_GEAR_PEG) { AddSequential(new WaitTime(.1)); AddSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG)); }
		AddSequential(new WaitTime(.25));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_1, .175));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_2, .175));
		AddParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_UPWARDS));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_3, .4));
	}
	else if (leftGearAndShoot) {
		/* if (USING_LIDAR_FOR_GEAR_LINEUP) { AddSequential(new DriveBackwardsWithLidar(-.65, SIDE_GEAR_LIDAR_VALUE_TO_TRAVEL_UNTIL, DISTANCE_FROM_ALLIANCE_WALL_TO_LEFT_GEAR_PART_1)); }
		else { AddSequential(new DriveDistance(DISTANCE_FROM_ALLIANCE_WALL_TO_RIGHT_GEAR_PART_1, -.775));
			   AddSequential(new DriveDistance(DISTANCE_FROM_ALLIANCE_WALL_TO_SIDE_GEAR_PART_2, -.45)); } */
		AddSequential(new DriveDistance(LEFT_SIDE_GEAR_DISTANCE_PART_1, -.65));
		AddParallel(new GetReadyForLiftHang());
		AddSequential(new DriveDistance(SIDE_GEAR_DISTANCE_PART_2, -.25));
		AddSequential(new DriveDistance(SIDE_GEAR_DISTANCE_PART_3, -.15));
		AddSequential(new WaitTime(.2));
		AddSequential(new TurnDriveTrainAngle((ANGLE_TO_TURN_TO_FACE_SIDE_GEAR), .65, TURN_RIGHT));
		AddSequential(new WaitTime(.1));
		AddSequential(new DriveDistance(DISTANCE_AFTER_TURNING_ONTO_SIDE_GEAR, -.4));
		AddSequential(new WaitTime(.15));
		//if (USING_GEAR_CAMERA) { AddSequential(new AimToGear(GEAR_CAMERA)); }
		AddSequential(new AutoGearOnPeg());
		AddSequential(new WaitTime(.001));
		AddParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_DOWNWARDS));
		AddSequential(new WaitTime(.6));
		AddSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG));
		//if (USING_AUTO_ROTATE_FOR_GEAR_PEG) { AddSequential(new WaitTime(.1)); AddSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG)); }
		AddSequential(new WaitTime(.25));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_1, .175));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_2, .175));
		AddParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_UPWARDS));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_3, .4));
		AddSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_AFTER_DRIVING_AWAY_FROM_LEFT_GEAR, .6, TURN_LEFT));
		AddParallel(new AutoGetShooterUpToSpeed(Shooter::SHOOTER_SETPOINT_RPM_FAR_SHOT));
		AddSequential(new WaitTime(.15));
		AddSequential(new DriveDistance(DISTANCE_TO_TRAVEL_AFTER_PLACING_SIDE_GEAR, .6));
		AddSequential(new WaitTime(.15));
		AddSequential(new AimBot(SHOOTER_CAMERA));
		AddSequential(new AutoDriveAgitator(STOP_RUNNING_COLLECTION));
	}
	else if (leftGearAndNeutralZone) {
		/* if (USING_LIDAR_FOR_GEAR_LINEUP) { AddSequential(new DriveBackwardsWithLidar(-.65, SIDE_GEAR_LIDAR_VALUE_TO_TRAVEL_UNTIL, DISTANCE_FROM_ALLIANCE_WALL_TO_LEFT_GEAR_PART_1)); }
		else { AddSequential(new DriveDistance(DISTANCE_FROM_ALLIANCE_WALL_TO_RIGHT_GEAR_PART_1, -.775));
			   AddSequential(new DriveDistance(DISTANCE_FROM_ALLIANCE_WALL_TO_SIDE_GEAR_PART_2, -.45)); } */
		AddSequential(new DriveDistance(LEFT_SIDE_GEAR_DISTANCE_PART_1, -.65));
		AddParallel(new GetReadyForLiftHang());
		AddSequential(new DriveDistance(SIDE_GEAR_DISTANCE_PART_2, -.25));
		AddSequential(new DriveDistance(SIDE_GEAR_DISTANCE_PART_3, -.15));
		AddSequential(new WaitTime(.2));
		AddSequential(new TurnDriveTrainAngle((ANGLE_TO_TURN_TO_FACE_SIDE_GEAR), .65, TURN_RIGHT));
		AddSequential(new WaitTime(.1));
		AddSequential(new DriveDistance(DISTANCE_AFTER_TURNING_ONTO_SIDE_GEAR, -.4));
		AddSequential(new WaitTime(.15));
		//if (USING_GEAR_CAMERA) { AddSequential(new AimToGear(GEAR_CAMERA)); }
		AddSequential(new AutoGearOnPeg());
		AddSequential(new WaitTime(.001));
		AddParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_DOWNWARDS));
		AddSequential(new WaitTime(.6));
		AddSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG));
		//if (USING_AUTO_ROTATE_FOR_GEAR_PEG) { AddSequential(new WaitTime(.1)); AddSequential(new AutoRotateRobotForGearPeg(AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER, ANGLE_TO_ROTATE_FOR_GEAR_PEG)); }
		AddSequential(new WaitTime(.25));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_1, .175));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_2, .175));
		AddParallel(new AutoMoveGearHolder(AUTO_MOVE_GEAR_HOLDER_UPWARDS));
		AddSequential(new DriveDistance((DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_3 + 8.0), .4));
		AddSequential(new WaitTime(.2));
		AddSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_TO_FACE_NEUTRAL_ZONE, .65, TURN_LEFT));
		AddSequential(new WaitTime(.1));
		AddSequential(new DriveDistance(DISTANCE_TO_TRAVEL_TOWARDS_AND_INTO_NEUTRAL_ZONE, -.65));
	}
	else if (closeShotRightWithBaseline) {
		//  Make Sure Shooter Hood Encoder is at Proper Value - Encoder Value at 5000
		AddSequential(new AutoDriveShooter(Shooter::SHOOTER_SETPOINT_RPM_CLOSE_SHOT));
		AddSequential(new DriveDistance(DISTANCE_TO_TRAVEL_BACKWARDS_AFTER_CLOSE_SHOT, -.5));
		AddParallel(new GetReadyForLiftHang());
		AddSequential(new WaitTime(.2));
		AddSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_AFTER_DRIVING_OFF_RIGHT_ALLIANCE_WALL, .6, TURN_RIGHT));
		AddSequential(new WaitTime(.2));
		AddSequential(new DriveDistance(DISTANCE_TO_TRAVEL_TO_BASELINE_AFTER_CLOSE_SHOT, -.65));
	}
	else if (closeShotLeftWithBaseline) {
		//  Make Sure Shooter Hood Encoder is at Proper Value
		AddSequential(new AutoDriveShooter(Shooter::SHOOTER_SETPOINT_RPM_CLOSE_SHOT));
		AddSequential(new TurnOneSideOfRobotAngle(ANGLE_TO_TURN_AFTER_DRIVING_OFF_LEFT_ALLIANCE_WALL, DRIVE_LEFT_SIDE_DRIVE_TRAIN, -.65));
		AddSequential(new WaitTime(.1));
		AddSequential(new DriveDistance(DISTANCE_TO_TRAVEL_OFF_LEFT_ALLIANCE_WALL, -.6));
		AddParallel(new GetReadyForLiftHang());
		AddSequential(new WaitTime(.2));
		AddSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_AFTER_TURNING_OFF_LEFT_ALLIANCE_WALL, .55, TURN_RIGHT));
		AddSequential(new WaitTime(.2));
		AddSequential(new DriveDistance(DISTANCE_TO_TRAVEL_ABOUT_PARALLEL_WITH_LEFT_ALLIANCE_WALL, .65));
		AddSequential(new WaitTime(.1));
		AddSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_TO_DRIVE_TO_BASELINE_AFTER_LEFT_ALLIANCE_WALL, .55, TURN_RIGHT));
		AddSequential(new WaitTime(.1));
		AddSequential(new DriveDistance(DISTANCE_TO_TRAVEL_TO_BASELINE_AFTER_CLOSE_SHOT, .7));
	}
	else if (right40KPa) {
		if (KPA_AUTONOMOUS_OPTION_1) {
			AddSequential(new DriveDistance(DISTANCE_FROM_ALLIANCE_WALL_TO_MIDDLE_OF_HOPPER_PANEL_PART_1, -.35));
			AddSequential(new DriveDistance(DISTANCE_FROM_ALLIANCE_WALL_TO_MIDDLE_OF_HOPPER_PANEL_PART_2, -.7));
			AddSequential(new DriveDistance(DISTANCE_FROM_ALLIANCE_WALL_TO_MIDDLE_OF_HOPPER_PANEL_PART_3, -.15));
			AddSequential(new WaitTime(.1));
			AddSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_INTO_HOPPER_PANEL, .6, TURN_LEFT));
			AddParallel(new AutoDriveCollection());
			AddParallel(new AutoGetShooterUpToSpeed(Shooter::SHOOTER_SETPOINT_RPM_FAR_SHOT));
			AddSequential(new WaitTime(.8));
			AddSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_OUT_OF_HOPPER_PANEL, .6, TURN_RIGHT));
			AddSequential(new WaitTime(.1));
			//AddSequential(new AimBot(SHOOTER_CAMERA));
			AddSequential(new AutoDriveAgitator(STOP_RUNNING_COLLECTION));
		}
		else if (KPA_AUTONOMOUS_OPTION_1 == false) {
			AddSequential(new DriveDistance(DISTANCE_TO_DRIVE_FROM_ALLIANCE_WALL_TO_GUARDRAIL, -.35));
			AddSequential(new TurnOneSideOfRobotAngle(ANGLE_TO_TURN_TO_BE_PARALLEL_WITH_GUARDRAIL, DRIVE_LEFT_SIDE_DRIVE_TRAIN, -.55));
			AddSequential(new DriveParallelWithGuardrailWithUltrasonicSensor(DISTANCE_TO_DRIVE_TOWARDS_HOPPER, DISTANCE_AWAY_FROM_GUARDRAIL_TO_DRIVE, -.45, RIGHT_HOPPER_AND_SHOOT));
			AddSequential(new WaitTime(.05));
			AddSequential(new TurnOneSideOfRobotAngle(ANGLE_TO_TURN_TO_LINE_UP_TO_HIT_PANEL, DRIVE_LEFT_SIDE_DRIVE_TRAIN, -.55));
			AddSequential(new WaitTime(.05));
			AddParallel(new AutoDriveCollection());
			AddSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_AWAY_FROM_HOPPER_PANEL, .6, TURN_RIGHT));
			AddParallel(new AutoGetShooterUpToSpeed(Shooter::SHOOTER_SETPOINT_RPM_FAR_SHOT));
			AddSequential(new WaitTime(.1));
			AddSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_TOWARDS_HOPPER, .6, TURN_LEFT));
			AddSequential(new WaitTime(.8));
			//AddSequential(new AimBot(SHOOTER_CAMERA));
			AddSequential(new AutoDriveAgitator(STOP_RUNNING_COLLECTION));
		}
	}
	else if (left40KPa) {
		if (KPA_AUTONOMOUS_OPTION_1) {

		}
		else if (KPA_AUTONOMOUS_OPTION_1 == false) {
			AddSequential(new DriveDistance(DISTANCE_TO_DRIVE_FROM_ALLIANCE_WALL_TO_GUARDRAIL, -.35));
			AddSequential(new TurnOneSideOfRobotAngle(ANGLE_TO_TURN_TO_BE_PARALLEL_WITH_GUARDRAIL, DRIVE_RIGHT_SIDE_DRIVE_TRAIN, -.55));
			AddSequential(new DriveParallelWithGuardrailWithUltrasonicSensor(DISTANCE_TO_DRIVE_TOWARDS_HOPPER, DISTANCE_AWAY_FROM_GUARDRAIL_TO_DRIVE, .45, RIGHT_HOPPER_AND_SHOOT));
			AddSequential(new WaitTime(.05));
			AddSequential(new TurnOneSideOfRobotAngle(ANGLE_TO_TURN_TO_LINE_UP_TO_HIT_PANEL, DRIVE_RIGHT_SIDE_DRIVE_TRAIN, -.55));
			AddSequential(new WaitTime(.05));
			AddParallel(new AutoDriveCollection());
			AddSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_AWAY_FROM_HOPPER_PANEL, .6, TURN_LEFT));
			AddParallel(new AutoGetShooterUpToSpeed(Shooter::SHOOTER_SETPOINT_RPM_FAR_SHOT));
			AddSequential(new WaitTime(.1));
			AddSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_TOWARDS_HOPPER, .6, TURN_RIGHT));
			AddSequential(new WaitTime(.8));
			//AddSequential(new AimBot(SHOOTER_CAMERA));
			AddSequential(new AutoDriveAgitator(STOP_RUNNING_COLLECTION));
		}
	}
}
