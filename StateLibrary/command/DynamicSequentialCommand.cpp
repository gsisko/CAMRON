/*
 * DynamicSequentialCommand.cpp
 *
 *  Created on: Sep 14, 2012
 *      Author: Mitchell Wills
 */

#include "DynamicSequentialCommand.h"
#include "Arduino.h"

DynamicSequentialCommand::DynamicSequentialCommand() {
	currentCommand = 0;
}

DynamicSequentialCommand::~DynamicSequentialCommand() {
}


void DynamicSequentialCommand::end(){
}



void DynamicSequentialCommand::initCommand(){
	currentCommand = nextCommand();
	if(currentCommand)
		currentCommand->initCommand();
}

void DynamicSequentialCommand::run(){
	if(!currentCommand)
		return;
	if(currentCommand->isFinished()){
		currentCommand->end();
		currentCommand = nextCommand();
		if(currentCommand)
			currentCommand->initCommand();
	}
	if(currentCommand)
		currentCommand->run();
}
bool DynamicSequentialCommand::isFinished(){
	return currentCommand==0;
}
