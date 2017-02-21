#ifndef ServerConnection_H
#define ServerConnection_H

#include "../CommandBase.h"

class ServerConnection : public CommandBase {
public:
	ServerConnection();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // ServerConnection_H
