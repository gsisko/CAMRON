/*
 * WaitCommand.h
 *
 * A command which will complete after a certain amount of time (in ms)
 *
 *  Created on: Oct 5, 2012
 *      Author: Mitchell Wills
 */

#ifndef WAITCOMMAND_H_
#define WAITCOMMAND_H_

#include "Command.h"

class WaitCommand : public Command {
public:
	WaitCommand(unsigned long time_ms);
	virtual void initCommand();
	virtual void run();
	virtual bool isFinished();
	virtual void end();
	virtual ~WaitCommand();
private:
	unsigned long time_ms;
	unsigned long startTime;
};

#endif /* WAITCOMMAND_H_ */
