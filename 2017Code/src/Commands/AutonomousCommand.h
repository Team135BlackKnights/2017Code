#ifndef AutonomousCommand_H
#define AutonomousCommand_H

#include <Commands/CommandGroup.h>
#include "../RobotMap.h"

class AutonomousCommand : public CommandGroup {
private:
	static constexpr double DISTANCE_FROM_ALLIANCE_WALL_TO_MIDDLE_GEAR = 36.0;
	static constexpr double DISTANCE_FROM_ALLIANCE_WALL_TO_SIDE_GEAR = 48.0;

	static constexpr double ANGLE_TO_TURN_TO_FACE_SIDE_GEAR = 45.0;
	static constexpr double ANGLE_FROM_SIDE_GEAR_TO_BASE_LINE_PATH = 75.0;

	static constexpr double DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING = 12.0;
	static constexpr double DISTANCE_TO_MOVE_AWAY_FROM_SIDE_GEAR_TO_PURSUE_BASELINE = 30.0;
	static constexpr double DISTANCE_OF_SIDE_GEAR_BASE_LINE_PATH = 36.0;

	static const bool TURN_RIGHT = true;
	static const bool TURN_LEFT = !TURN_RIGHT;

	AutonomousSelection autonomousSelection;
	BaseLinePath baseLinePath;
public:
	AutonomousCommand(AutonomousSelection, BaseLinePath);
};

#endif  // AutonomousCommand_H
