/*
 * SingleRunCommandRunner.cpp
 *
 *  Created on: Oct 7, 2012
 *      Author: Mitchell Wills
 */

#include "command/SingleRunCommandRunner.h"

SingleRunCommandRunner::SingleRunCommandRunner(Command* _command) : command(_command) {
	hasInit = false;
	done = false;
}

void SingleRunCommandRunner::run(){
	if(!hasInit){
		command->initCommand();
		hasInit = true;
	}
	if(command->isFinished()){
		command->end();
		done = true;
	}
	if(!done)
		command->run();
}

SingleRunCommandRunner::~SingleRunCommandRunner() {
}

