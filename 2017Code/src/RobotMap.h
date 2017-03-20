#ifndef ROBOTMAP_H
#define ROBOTMAP_H

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

// For example to map the left and right motors, you could define the
// following variables to use with your drivetrain subsystem.
// constexpr int LEFTMOTOR = 1;
// constexpr int RIGHTMOTOR = 2;


// If you are using multiple modules, make sure to define both the port
// number and the module. For example you with a rangefinder:
// constexpr int RANGE_FINDER_PORT = 1;
// constexpr int RANGE_FINDER_MODULE = 1;

	const bool COMPETITION_BOT = true;

	//  Practice Bot Motor Ports
	//  VictorSP motor ports
	const int PB_COLLECTION_VICTOR_PWM_PORT = 3;
	const int PB_HANG_VICTOR_PWM_PORT = 2;
	const int PB_GEAR_VICTOR_PWM_PORT = 0;
	const int PB_AGITATOR_VICTOR_PWM_PORT = 1;

	//  CANTalon motor ports
	const int PB_FRONT_LEFT_TALON_ID = 3;
	const int PB_REAR_LEFT_TALON_ID = 4;
	const int PB_FRONT_RIGHT_TALON_ID = 1;
	const int PB_REAR_RIGHT_TALON_ID = 2;

	const int PB_SHOOTER_MOTOR_TALON_ID = 6;
	const int PB_HOOD_MOTOR_TALON_ID = 5;

	const int PB_GEAR_HOLDER_SERVO_PWM_PORT = 4;

	//  Motor Inversions
	const bool PB_AGITATOR_INVERTED = true;
	const bool PB_COLLECTION_INVERTED = false;
	const bool PB_DRIVE_TRAIN_FRONT_LEFT_INVERTED = false;
	const bool PB_DRIVE_TRAIN_REAR_LEFT_INVERTED = false;
	const bool PB_DRIVE_TRAIN_FRONT_RIGHT_INVERTED = true;
	const bool PB_DRIVE_TRAIN_REAR_RIGHT_INVERTED = true;
	const bool PB_GEAR_HOLDER_INVERTED = true;
	const bool PB_LIFT_HANG_MOTOR_INVERTED = false;
	const bool PB_SHOOTER_MOTOR_INVERTED = false;
	const bool PB_SHOOTER_HOOD_MOTOR_INVERTED = false;
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

	//  Competition Bot Motor Ports
	//  Victor SP Motor Ports
	const int CB_COLLECTION_VICTOR_PWM_PORT = 11;
	const int CB_HANG_VICTOR_PWM_PORT = 10;
	const int CB_GEAR_VICTOR_PWM_PORT = 17;
	const int CB_AGITATOR_VICTOR_PWM_PORT = 18;

	//  CANTalon Motor Ports
	const int CB_FRONT_LEFT_TALON_ID = 1;
	const int CB_REAR_LEFT_TALON_ID = 2;
	const int CB_FRONT_RIGHT_TALON_ID = 3;
	const int CB_REAR_RIGHT_TALON_ID = 4;

	const int CB_SHOOTER_MOTOR_TALON_ID = 6;
	const int CB_HOOD_MOTOR_TALON_ID = 5;

	const int CB_GEAR_HOLDER_SERVO_PWM_PORT = 0;

	//  Motor Inversions
	//  To Be Determined
	const bool CB_AGITATOR_INVERTED = true;
	const bool CB_COLLECTION_INVERTED = true;
	const bool CB_DRIVE_TRAIN_FRONT_LEFT_INVERTED = false;
	const bool CB_DRIVE_TRAIN_REAR_RIGHT_INVERTED = true;
	const bool CB_DRIVE_TRAIN_FRONT_RIGHT_INVERTED = true;
	const bool CB_DRIVE_TRAIN_REAR_LEFT_INVERTED = false;
	const bool CB_GEAR_HOLDER_INVERTED = true;
	const bool CB_LIFT_HANG_MOTOR_INVERTED = false;
	const bool CB_SHOOTER_MOTOR_INVERTED = false;
	const bool CB_SHOOTER_HOOD_MOTOR_INVERTED = true;

	enum AutonomousSelection {
		MiddleGear,
		RightGear,
		LeftGear,
		CloseShotShooterLeft,
		CloseShotShooterRight,
		RightKPaAutonomous,
		LeftKPaAutonomous,
		BaseLine
	};

	enum SecondTask {
		MiddleGearShootRight,
		MiddleGearShootLeft,
		SideGearShoot,
		CloseShotBaseLine,
		None
	};

	enum ShooterPIDSelection {
		PID_CompetitionBot,
		PID_Ramp_Up,
		Voltage
	};

	const ShooterPIDSelection SHOOTER_PID_SELECTION = ShooterPIDSelection::PID_CompetitionBot;

#endif  // ROBOTMAP_H
