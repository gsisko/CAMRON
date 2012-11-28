/*
 * DynamicSequentialCommand.h
 *
 * A command which runs the commands returned by its nextCommand method until the method returns NULL
 *
 *  Created on: Sep 14, 2012
 *      Author: Mitchell Wills
 */

#ifndef DYNAMICSEQUENTIALCOMMAND_H_
#define DYNAMICSEQUENTIALCOMMAND_H_

#include "Arduino.h"
#include "Command.h"


class DynamicSequentialCommand : public Command {
public:
	DynamicSequentialCommand();

	virtual void initCommand();
	virtual void run();
	virtual void end();
	virtual bool isFinished();
	virtual Command* nextCommand() = 0;
protected:
	virtual ~DynamicSequentialCommand();
private:
	Command* currentCommand;
};

#endif /* DYNAMICSEQUENTIALCOMMAND_H_ */
