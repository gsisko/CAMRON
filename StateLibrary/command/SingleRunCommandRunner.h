/*
 * SingleRunCommandRunner.h
 *
 * A simple class that will handle the running of a command exactly once
 * it has a run method which should be called continuously in main
 *
 *  Created on: Oct 7, 2012
 *      Author: Mitchell Wills
 */

#ifndef SINGLERUNCOMMANDRUNNER_H_
#define SINGLERUNCOMMANDRUNNER_H_

#include "command/Command.h"

class SingleRunCommandRunner {
public:
	SingleRunCommandRunner(Command* command);
	void run();
	virtual ~SingleRunCommandRunner();
private:
	Command* command;
	bool hasInit;
	bool done;
};

#endif /* SINGLERUNCOMMANDRUNNER_H_ */
