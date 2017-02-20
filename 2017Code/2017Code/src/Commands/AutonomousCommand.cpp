#include "AutonomousCommand.h"
#include "AutoGearOnPeg.h"
#include "DriveDistance.h"
#include "TurnDriveTrainAngle.h"
#include "WaitTime.h"

AutonomousCommand::AutonomousCommand(AutonomousSelection autonomousSelection, BaseLinePath baseLinePath) {
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

	this->autonomousSelection = autonomousSelection;
	this->baseLinePath = baseLinePath;
	if (this->autonomousSelection == AutonomousSelection::MiddleGear) {
		AddSequential(new DriveDistance(DISTANCE_FROM_ALLIANCE_WALL_TO_MIDDLE_GEAR, -.6));
		AddSequential(new WaitTime(.25));
		//  Camera Lines Up To Gear
		AddSequential(new AutoGearOnPeg());
		AddSequential(new WaitTime(1.0));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING, .3));
		//  Go To Baseline
	}
	else if (this->autonomousSelection == AutonomousSelection::RightGear) {
		AddSequential(new DriveDistance(DISTANCE_FROM_ALLIANCE_WALL_TO_SIDE_GEAR, -.6));
		AddSequential(new WaitTime(.25));
		AddSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_TO_FACE_SIDE_GEAR, .5, TURN_LEFT));
		//  Camera Lines Up To Gear
		AddSequential(new AutoGearOnPeg());
		AddSequential(new WaitTime(1.0));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING, .3));
		if (this->baseLinePath == BaseLinePath::SideGear) {
			AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_SIDE_GEAR_TO_PURSUE_BASELINE, -.5));
			AddSequential(new WaitTime(.25));
			AddSequential(new TurnDriveTrainAngle(ANGLE_FROM_SIDE_GEAR_TO_BASE_LINE_PATH, .5, TURN_LEFT));
			AddSequential(new DriveDistance(DISTANCE_OF_SIDE_GEAR_BASE_LINE_PATH, .6));
		}
	}
	else if (this->autonomousSelection == AutonomousSelection::LeftGear) {
		AddSequential(new DriveDistance(DISTANCE_FROM_ALLIANCE_WALL_TO_SIDE_GEAR, .6));
		AddSequential(new WaitTime(.25));
		AddSequential(new TurnDriveTrainAngle(ANGLE_TO_TURN_TO_FACE_SIDE_GEAR, .5, TURN_RIGHT));
		//  Camera Lines Up To Gear
		AddSequential(new AutoGearOnPeg());
		AddSequential(new WaitTime(1.0));
		AddSequential(new DriveDistance(DISTANCE_TO_MOVE_AWAY_FROM_GEAR_AFTER_PLACING, -.3));
	}
}
