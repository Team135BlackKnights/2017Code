#include "LogValues.h"
#include "../RobotMap.h"

LogValues::LogValues() : Subsystem("ExampleSubsystem") {

}

void LogValues::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

void LogValues::OpenFile()
{
	LogFile.open("");
}
void LogValues::CloseFile()
{
	LogFile.close;
}
void LogValues::WriteToFile(double data, string dataType)
{
	this->OpenFile();
	LogFile << dataType << "value is"<< data << endl;
	this->CloseFile();
}
void LogValues::ChangeFileName(string newName) {
	this->FileName = newName;
}

void LogValues::ChangeFilePath(string newPath) {
	this-FilePath = newPath;
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
