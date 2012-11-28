/*
 * SequentialCommand.h
 *
 * A command which will run a set of commands in sequence
 * A command is added using the addSequential method
 *
 *  Created on: Sep 11, 2012
 *      Author: Mitchell
 */

#ifndef SEQUENTIALCOMMAND_H_
#define SEQUENTIALCOMMAND_H_

#include "Arduino.h"
#include "DynamicSequentialCommand.h"


struct SequentialCommandElement{
	Command* command;
	struct SequentialCommandElement* nextCommand;
};
typedef struct SequentialCommandElement SequentialCommandElement;

class SequentialCommand: public DynamicSequentialCommand {
public:
	SequentialCommand();
	virtual ~SequentialCommand();
	virtual void initCommand();
	void addCommand(Command* command);
protected:
	virtual Command* nextCommand();
private:
	SequentialCommandElement* firstCommand;
	SequentialCommandElement* currentCommand;
};

#endif /* SEQUENTIALCOMMAND_H_ */
