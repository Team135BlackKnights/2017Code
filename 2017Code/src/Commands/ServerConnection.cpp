#include "ServerConnection.h"

ServerConnection::ServerConnection() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(server.get());
}

// Called just before this Command runs the first time
void ServerConnection::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void ServerConnection::Execute() {
	server->Run();
}

// Make this return true when this Command no longer needs to run execute()
bool ServerConnection::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ServerConnection::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ServerConnection::Interrupted() {

}
