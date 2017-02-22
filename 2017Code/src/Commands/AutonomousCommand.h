#ifndef AutonomousCommand_H
#define AutonomousCommand_H

#include <Commands/CommandGroup.h>
#include "../RobotMap.h"

class AutonomousCommand : public CommandGroup {
private:
	static constexpr double ROBOT_WITH_BUMPERS_LENGTH = 39.0;
	static constexpr double ROBOT_WITH_BUMPERS_WIDTH = 36.0;

	static constexpr double DISTANCE_FROM_ALLIANCE_WALL_TO_MIDDLE_GEAR = (80.0 - ROBOT_WITH_BUMPERS_LENGTH);
	static constexpr double DISTANCE_FROM_ALLIANCE_WALL_TO_SIDE_GEAR = (120.0 - ROBOT_WITH_BUMPERS_LENGTH);

	static constexpr double ANGLE_TO_TURN_TO_FACE_SIDE_GEAR = 45.0;
	static constexpr double ANGLE_FROM_SIDE_GEAR_TO_BASE_LINE_PATH = 75.0;
	static constexpr double RIGHT_ANGLE_DEGREES = 90.0;

	static constexpr double DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING = 24.0;

	static constexpr double DISTANCE_TO_MOVE_AWAY_FROM_MIDDLE_GEAR_TO_PURSUE_BASELINE = 25.0;
	static constexpr double DISTANCE_AWAY_FROM_MIDDLE_GEAR_LANE_TO_SIDE_LANE = 85.0;
	static constexpr double DISTANCE_TO_TRAVEL_FOR_ROBOT_TO_CROSS_BASELINE = 100.0;

	static constexpr double DISTANCE_TO_MOVE_AWAY_FROM_SIDE_GEAR_TO_PURSUE_BASELINE = 30.0;
	static constexpr double DISTANCE_OF_SIDE_GEAR_BASE_LINE_PATH = 36.0;

	static constexpr double DISTANCE_AFTER_TURNING_ONTO_SIDE_GEAR = 10.0;

	static const bool TURN_RIGHT = true;
	static const bool TURN_LEFT = !TURN_RIGHT;

	AutonomousSelection autonomousSelection;
	BaseLinePath baseLinePath;
public:
	AutonomousCommand(AutonomousSelection, BaseLinePath);
};

#endif  // AutonomousCommand_H
