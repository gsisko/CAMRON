/*
 * PeriodicCommand.h
 *
 * A command which will call the periodicRun method with a specified delay
 *
 *  Created on: Oct 7, 2012
 *      Author: Mitchell Wills
 */

#ifndef PERIODICCOMMAND_H_
#define PERIODICCOMMAND_H_

#include "Command.h"

class PeriodicCommand: public Command {
public:
	PeriodicCommand(unsigned long delay);
	virtual void initCommand();
	virtual void run();
	virtual void end();
	virtual bool isFinished();
	virtual void periodicRun() = 0;
	virtual ~PeriodicCommand();
private:
	unsigned long delay;
	unsigned long lastRun;
};

#endif /* PERIODICCOMMAND_H_ */
