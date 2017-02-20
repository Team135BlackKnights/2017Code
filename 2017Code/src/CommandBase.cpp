#include "CommandBase.h"

#include <Commands/Scheduler.h>

// Initialize a single static instance of all of your subsystems. The following
// line should be repeated for each subsystem in the project.

std::unique_ptr<OI> CommandBase::oi = std::make_unique<OI>();
std::unique_ptr<DriveTrain> CommandBase::driveTrain = std::make_unique<DriveTrain>();
std::unique_ptr<Agitator> CommandBase::agitator = std::make_unique<Agitator>();
std::unique_ptr<Collection> CommandBase::collection = std::make_unique<Collection>();
std::unique_ptr<GearHolder> CommandBase::gearHolder = std::make_unique<GearHolder>();
std::unique_ptr<Lidars> CommandBase::lidars = std::make_unique<Lidars>();
std::unique_ptr<LiftHang> CommandBase::liftHang = std::make_unique<LiftHang>();
std::unique_ptr<Shooter> CommandBase::shooter = std::make_unique<Shooter>();
std::unique_ptr<ShooterHood> CommandBase::shooterHood = std::make_unique<ShooterHood>();

CommandBase::CommandBase(const std::string &name) :
		frc::Command(name) {

}
