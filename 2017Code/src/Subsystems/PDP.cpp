#include "PDP.h"
#include "../RobotMap.h"

PDP::PDP() : Subsystem("PDP") {

}

void PDP::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

void PDP::InitializePDP() {
	//pdp = new frc::PowerDistributionPanel(PDP_ID);
}

double PDP::GetCurrentOfPDPPort(int pdpPort) {
	//return pdp->GetCurrent(pdpPort);
	return 0.0;
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
