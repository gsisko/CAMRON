/*
 * ParallelCommand.cpp
 *
 *  Created on: Oct 8, 2012
 *      Author: Mitchell Wills
 */

#include "ParallelCommand.h"
#include "Arduino.h"

ParallelCommand::ParallelCommand() {
}
void ParallelCommand::addParallel(Command* command){
	ParallelCommandElement* newSequenceElement = (ParallelCommandElement*)malloc(sizeof(ParallelCommandElement));
	newSequenceElement->command = command;
	newSequenceElement->nextCommand = 0;
	if(firstCommand){
		ParallelCommandElement* sequenceElement = firstCommand;
		while(sequenceElement->nextCommand)
			sequenceElement = sequenceElement->nextCommand;
		sequenceElement->nextCommand = newSequenceElement;
	}
	else{
		firstCommand = newSequenceElement;
	}
}

void ParallelCommand::initCommand(){
	if(firstCommand){
		ParallelCommandElement* sequenceElement = firstCommand;
		do{
			sequenceElement->done = false;
			sequenceElement->command->initCommand();
			sequenceElement = sequenceElement->nextCommand;
		} while(sequenceElement);
	}
}
void ParallelCommand::run(){
	if(firstCommand){
		ParallelCommandElement* sequenceElement = firstCommand;
		do{
			if(sequenceElement->command->isFinished()){
				sequenceElement->command->end();
				sequenceElement->done = true;
			}
			if(!sequenceElement->done)
				sequenceElement->command->run();

			sequenceElement = sequenceElement->nextCommand;
		} while(sequenceElement);
	}
}
void ParallelCommand::end(){
}
bool ParallelCommand::isFinished(){
	bool done = true;
	if(firstCommand){
		ParallelCommandElement* sequenceElement = firstCommand;
		do{
			if(!sequenceElement->done)
				done = false;
			sequenceElement = sequenceElement->nextCommand;
		} while(sequenceElement);
	}
	return done;
}

ParallelCommand::~ParallelCommand() {
}

