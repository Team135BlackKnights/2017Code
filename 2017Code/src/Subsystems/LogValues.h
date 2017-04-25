#ifndef LogValues_H
#define LogValues_H

#include <Commands/Subsystem.h>
#include <fstream>
#include <string>

using namespace std;

class LogValues : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

	ofstream LogFile;
	string FileName;
	string FilePath;


public:
	LogValues();
	void InitDefaultCommand();

	void OpenFile();
	void CloseFile();
	void WriteToFile(double, string);
	void ChangeFileName(string);
	void ChangeFilePath(string);

	bool isFileOpen;


};

#endif  // LogValues_H
