/*
 * ParallelCommand.h
 *
 * A command which runs a set of commands in parallel
 * Commands are added using the addParallel method
 *
 *  Created on: Oct 8, 2012
 *      Author: Mitchell Wills
 */

#ifndef PARALLELCOMMAND_H_
#define PARALLELCOMMAND_H_

#include "Command.h"

struct ParallelCommandElement{
	Command* command;
	bool done;
	struct ParallelCommandElement* nextCommand;
};
typedef struct ParallelCommandElement ParallelCommandElement;


class ParallelCommand: public Command {
public:
	ParallelCommand();
	virtual void initCommand();
	virtual void run();
	virtual void end();
	virtual bool isFinished();
	void addParallel(Command* command);
	virtual ~ParallelCommand();
private:
	ParallelCommandElement* firstCommand;
};

#endif /* PARALLELCOMMAND_H_ */
