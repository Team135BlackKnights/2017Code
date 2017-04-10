#ifndef AutonomousCommand_H
#define AutonomousCommand_H

#include <Commands/CommandGroup.h>
#include "../RobotMap.h"

class AutonomousCommand : public CommandGroup {
private:
	//  Robot Dimensions with Bumpers
	static constexpr double ROBOT_WITH_BUMPERS_LENGTH = 39.0;
	static constexpr double ROBOT_WITH_BUMPERS_WIDTH = 36.0;

	//  Simple Base Line Autonomous Path
	static constexpr double BASE_LINE_PATH_DISTANCE = 85.0;

	static constexpr double RIGHT_SIDE_GEAR_DISTANCE_PART_1 = 37.0;
	static constexpr double LEFT_SIDE_GEAR_DISTANCE_PART_1 = 36.0;
	static constexpr double SIDE_GEAR_DISTANCE_PART_2 = 20.0;
	static constexpr double SIDE_GEAR_DISTANCE_PART_3 = 20.0;

	//  Gear Autonomous Programs Distances
	static constexpr double DISTANCE_FROM_ALLIANCE_WALL_TO_MIDDLE_GEAR = (80.0 - ROBOT_WITH_BUMPERS_LENGTH);
	static constexpr double DISTANCE_FROM_ALLIANCE_WALL_TO_RIGHT_GEAR_PART_1 = (93.0 - ROBOT_WITH_BUMPERS_LENGTH);
	static constexpr double DISTANCE_FROM_ALLIANCE_WALL_TO_LEFT_GEAR_PART_1 = (91.0 - ROBOT_WITH_BUMPERS_LENGTH);
	static constexpr double DISTANCE_FROM_ALLIANCE_WALL_TO_SIDE_GEAR_PART_2 = 20.0;

	static constexpr double ANGLE_TO_TURN_TO_FACE_SIDE_GEAR = 43.0;
	static constexpr double ANGLE_FROM_SIDE_GEAR_TO_BASE_LINE_PATH = 75.0;

	static constexpr double DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_1 = 11.0;
	static constexpr double DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_2 = 6.0;
	static constexpr double DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_PART_3 = 8.0;

	static constexpr double DISTANCE_TO_MOVE_AWAY_FROM_SIDE_GEAR_TO_PURSUE_BASELINE = 30.0;
	static constexpr double DISTANCE_OF_SIDE_GEAR_BASE_LINE_PATH = 36.0;

	static constexpr double DISTANCE_AFTER_TURNING_ONTO_SIDE_GEAR = 13.5;

	//  Gear Autonomous Program and Neutral Zone Variables
	static constexpr double ANGLE_TO_TURN_TO_FACE_NEUTRAL_ZONE = 42.0;
	static constexpr double DISTANCE_TO_TRAVEL_TOWARDS_AND_INTO_NEUTRAL_ZONE = 220.0;

	//  Gear Autonomous Program With Lidar Variables
	static constexpr double DISTANCE_TO_TRAVEL_UNITL_TURNING_PARALLEL_WITH_AIRSHP = 45.0;
	static constexpr double ANGLE_TO_TURN_TO_BE_PARALLEL_WITH_AIRSHIP = 30.0;
	static constexpr double DISTANCE_TO_TRAVEL_TO_START_LIDAR_DETECTING = 10.0;

	//  Shooter Autonomous Programs
	//  Both Close Shot Right and Close Shot Left
	static constexpr double DISTANCE_TO_TRAVEL_TO_BASELINE_AFTER_CLOSE_SHOT = 70.0;
	//  Close Shot Right
	static constexpr double DISTANCE_TO_TRAVEL_BACKWARDS_AFTER_CLOSE_SHOT = 15.0;
	static constexpr double ANGLE_TO_TURN_AFTER_DRIVING_OFF_RIGHT_ALLIANCE_WALL = 38.0;
	//  Close Shot Left
	static constexpr double ANGLE_TO_TURN_AFTER_DRIVING_OFF_LEFT_ALLIANCE_WALL = 15.0;
	static constexpr double DISTANCE_TO_TRAVEL_OFF_LEFT_ALLIANCE_WALL = 12.0;
	static constexpr double ANGLE_TO_TURN_AFTER_TURNING_OFF_LEFT_ALLIANCE_WALL = 45.0;
	static constexpr double DISTANCE_TO_TRAVEL_ABOUT_PARALLEL_WITH_LEFT_ALLIANCE_WALL = 20.0;
	static constexpr double ANGLE_TO_TURN_TO_DRIVE_TO_BASELINE_AFTER_LEFT_ALLIANCE_WALL = 50.0;

	//  Both Shoot After Right Gear and Shoot After Left Gear
	static constexpr double DISTANCE_TO_TRAVEL_AFTER_PLACING_SIDE_GEAR = 18.0;
	static constexpr double DISTANCE_TO_TRAVEL_AFTER_TURNING_TO_DRIVE_TOWARDS_BOILER = 35.0;
	//  Shoot After Right Gear
	static constexpr double ANGLE_TO_TURN_AFTER_DRIVING_AWAY_FROM_RIGHT_GEAR = 18.0;//18.0;
	//  Shoot After Left Gear
	static constexpr double ANGLE_TO_TURN_AFTER_DRIVING_AWAY_FROM_LEFT_GEAR = 14.0;

	//  Both Shoot Right After Placing Middle Gear and Shoot Left After Placing Middle Gear
	static constexpr double DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING_TO_SHOOT = 25.0;
	static constexpr double DISTANCE_LIDAR_AWAY_FROM_SIDE_RAIL = 80.0;
	//  Shoot Right After Placing Middle Gear
	static constexpr double ANGLE_TO_TURN_TO_FACE_BOILDER_RIGHT_SHOOT_GEAR = 25.0;
	//  Shoot Left After Placing Middle Gear
	static constexpr double ANGLE_TO_TURN_TO_FACE_BOILDER_LEFT_SHOOT_GEAR = 20.0;

	//  40 KPa Autonomous Variables
	static constexpr double DISTANCE_FROM_ALLIANCE_WALL_TO_HOPPER_PART_1 = 33.0;
	static constexpr double DISTANCE_FROM_ALLIANCE_WALL_TO_HOPPER_PART_2 = 20.0;
	static constexpr double DISTANCE_TO_DRIVE_TO_HIT_HOPPER = 15.0;
	static constexpr double TIME_TO_RAM_ROBOT_INTO_HOPPER = .35;
	static constexpr double DISTANCE_TO_DRIVE_AWAY_FROM_HOPPER = 10.0;
	static constexpr double ANGLE_TO_TURN_ON_TO_BOILER = 75.0;

	//  General Variables
	static const bool TURN_RIGHT = true;
	static const bool TURN_LEFT = !TURN_RIGHT;

	static const bool DRIVE_RIGHT_SIDE_DRIVE_TRAIN = true;
	static const bool DRIVE_LEFT_SIDE_DRIVE_TRAIN = !DRIVE_RIGHT_SIDE_DRIVE_TRAIN;

	static const bool RIGHT_GEAR = true;
	static const bool LEFT_GEAR = !RIGHT_GEAR;

	static const int GEAR_CAMERA = 1;
	static const int SHOOTER_CAMERA = 0;

	static constexpr double RIGHT_ANGLE_DEGREES = 90.0;

	//  Booleans for Autonomous Operation
	static const bool USING_LIDAR_FOR_GEAR_LINEUP = false;
	static const bool USING_AUTO_ROTATE_FOR_GEAR_PEG = true;

	static const bool USING_GEAR_CAMERA = false;
	static const bool USING_SHOOTER_CAMERA = true;
	static const bool USING_ULTRASONIC_SENSOR_AIM_BOT = false;

	static const bool RAMMING_INTO_HOPPER = true;
	static const bool NOT_RAMMING_INTO_HOPPER = false;

	static const bool KPA_AUTONOMOUS_OPTION_1 = true;

	static constexpr double ANGLE_TO_ROTATE_FOR_GEAR_PEG = 2.0;
	static constexpr double AUTO_ROTATE_FOR_GEAR_PEG_MOTOR_POWER = .5;

	static constexpr double MIDDLE_GEAR_LIDAR_VALUE_TO_TRAVEL_UNITL = 42.0;
	static constexpr double SIDE_GEAR_LIDAR_VALUE_TO_TRAVEL_UNTIL = 87.0;  //  85.0

	static const bool AUTO_MOVE_GEAR_HOLDER_UPWARDS = true;
	static const bool AUTO_MOVE_GEAR_HOLDER_DOWNWARDS = !AUTO_MOVE_GEAR_HOLDER_UPWARDS;

	bool baseLine = false;
	bool middleGear = false;
	bool middleGearShootRight = false;
	bool middleGearShootLeft = false;
	bool rightGear = false;
	bool leftGear = false;
	bool rightGearAndShoot = false;
	bool leftGearAndShoot = false;
	bool rightGearAndNeutralZone = false;
	bool leftGearAndNeutralZone = false;
	bool closeShotRightWithBaseline = false;
	bool closeShotLeftWithBaseline = false;
	bool right40KPa = false;
	bool left40KPa = false;
public:
	AutonomousCommand();
};

#endif  // AutonomousCommand_H
