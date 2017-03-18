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
	static constexpr double BASE_LINE_PATH_DISTANCE = 130.0;

	//  Gear Autonomous Programs Distances
	static constexpr double DISTANCE_FROM_ALLIANCE_WALL_TO_MIDDLE_GEAR = (80.0 - ROBOT_WITH_BUMPERS_LENGTH);
	static constexpr double DISTANCE_FROM_ALLIANCE_WALL_TO_RIGHT_GEAR_PART_1 = (93.0 - ROBOT_WITH_BUMPERS_LENGTH);
	static constexpr double DISTANCE_FROM_ALLIANCE_WALL_TO_LEFT_GEAR_PART_1 = (98.0 - ROBOT_WITH_BUMPERS_LENGTH);
	static constexpr double DISTANCE_FROM_ALLIANCE_WALL_TO_SIDE_GEAR_PART_2 = 20.0;

	static constexpr double ANGLE_TO_TURN_TO_FACE_SIDE_GEAR = 48.0;
	static constexpr double ANGLE_FROM_SIDE_GEAR_TO_BASE_LINE_PATH = 75.0;
	static constexpr double RIGHT_ANGLE_DEGREES = 90.0;

	static constexpr double DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING = 24.0;

	static constexpr double DISTANCE_TO_MOVE_AWAY_FROM_SIDE_GEAR_TO_PURSUE_BASELINE = 30.0;
	static constexpr double DISTANCE_OF_SIDE_GEAR_BASE_LINE_PATH = 36.0;

	static constexpr double DISTANCE_AFTER_TURNING_ONTO_SIDE_GEAR = 10;

	//  Shooter Autonomous Programs
	//  Both Close Shot Right and Close Shot Left
	static constexpr double DISTANCE_TO_TRAVEL_TO_BASELINE_AFTER_CLOSE_SHOT = 65.0;
	//  Close Shot Right
	static constexpr double DISTANCE_TO_TRAVEL_BACKWARDS_AFTER_CLOSE_SHOT = 15.0;
	static constexpr double ANGLE_TO_TURN_AFTER_DRIVING_OFF_RIGHT_ALLIANCE_WALL = 55.0;
	//  Close Shot Left
	static constexpr double ANGLE_TO_TURN_AFTER_DRIVING_OFF_LEFT_ALLIANCE_WALL = 25.0;
	static constexpr double DISTANCE_TO_TRAVEL_OFF_LEFT_ALLIANCE_WALL = 24.0;
	static constexpr double ANGLE_TO_TURN_AFTER_TURNING_OFF_LEFT_ALLIANCE_WALL = 50.0;

	//  Both Shoot After Right Gear and Shoot After Left Gear
	static constexpr double DISTANCE_TO_TRAVEL_AFTER_PLACING_SIDE_GEAR = 18.0;
	static constexpr double DISTANCE_TO_TRAVEL_AFTER_TURNING_TO_DRIVE_TOWARDS_BOILER = 35.0;
	//  Shoot After Right Gear
	static constexpr double ANGLE_TO_TURN_AFTER_DRIVING_AWAY_FROM_RIGHT_GEAR = 18.0;
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
	static constexpr double DISTANCE_TO_TRAVEL_FROM_ALLIANCE_WALL_TO_GUARDRAIL = 12.0;
	static constexpr double ANGLE_TO_TURN_TO_BE_PARALLEL_TO_GUARDRAIL = 45.0;
	static constexpr double DISTANCE_TO_TRAVEL_TO_HOPPER = 40.0;
	static constexpr double ANGLE_TO_TURN_TO_OR_AWAY_FROM_HOPPER = 25.0;
	static constexpr double DISTANCE_TO_BACK_ANGLED_TOWARDS_BOILER = 12.0;


	//  General Variables
	static const bool TURN_RIGHT = true;
	static const bool TURN_LEFT = !TURN_RIGHT;

	static const bool DRIVE_RIGHT_SIDE_DRIVE_TRAIN = true;
	static const bool DRIVE_LEFT_SIDE_DRIVE_TRAIN = !DRIVE_RIGHT_SIDE_DRIVE_TRAIN;

	static const int GEAR_CAMERA = 1;
	static const int SHOOTER_CAMERA = 0;

	static const bool USING_LIDAR = false;

	AutonomousSelection autonomousSelection;
	SecondTask secondTask;
public:
	AutonomousCommand(AutonomousSelection, SecondTask);
};

#endif  // AutonomousCommand_H
