#ifndef COMMAND_BASE_H
#define COMMAND_BASE_H

#include <memory>
#include <string>

#include <Commands/Command.h>

#include "OI.h"
#include "Subsystems/DriveTrain.h"
#include "Subsystems/Agitator.h"
#include "Subsystems/Collection.h"
#include "Subsystems/GearHolder.h"
#include "Subsystems/LiftHang.h"
#include "Subsystems/Shooter.h"
#include "Subsystems/ShooterHood.h"

/**
 * The base for all commands. All atomic commands should subclass CommandBase.
 * CommandBase stores creates and stores each control system. To access a
 * subsystem elsewhere in your code in your code use
 * CommandBase::exampleSubsystem
 */
class CommandBase: public frc::Command {
public:
	CommandBase(const std::string& name);
	CommandBase() = default;

	// Create a single static instance of all of your subsystems
	static std::unique_ptr<OI> oi;
	static std::unique_ptr<DriveTrain> driveTrain;
	static std::unique_ptr<Agitator> agitator;
	static std::unique_ptr<Collection> collection;
	static std::unique_ptr<GearHolder> gearHolder;
	static std::unique_ptr<LiftHang> liftHang;
	static std::unique_ptr<Shooter> shooter;
	static std::unique_ptr<ShooterHood> shooterHood;
};

#endif  // COMMAND_BASE_H
