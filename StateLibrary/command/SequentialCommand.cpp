/*
 * SequentialCommand.cpp
 *
 *  Created on: Sep 11, 2012
 *      Author: Mitchell
 */

#include "SequentialCommand.h"

SequentialCommand::SequentialCommand() {
	firstCommand = 0;
	currentCommand = 0;
}

void SequentialCommand::addCommand(Command* command){
	SequentialCommandElement* newSequenceElement = (SequentialCommandElement*)malloc(sizeof(SequentialCommandElement));
	newSequenceElement->command = command;
	newSequenceElement->nextCommand = 0;
	if(firstCommand){
		SequentialCommandElement* sequenceElement = firstCommand;
		while(sequenceElement->nextCommand)
			sequenceElement = sequenceElement->nextCommand;
		sequenceElement->nextCommand = newSequenceElement;
	}
	else{
		firstCommand = newSequenceElement;
	}
}

void SequentialCommand::initCommand(){
	currentCommand = 0;
	DynamicSequentialCommand::initCommand();
}

Command* SequentialCommand::nextCommand(){
	if(currentCommand)
		currentCommand = currentCommand->nextCommand;
	else
		currentCommand = firstCommand;
	return currentCommand->command;
}

SequentialCommand::~SequentialCommand() {
}

