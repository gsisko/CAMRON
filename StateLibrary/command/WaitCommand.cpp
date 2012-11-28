/*
 * WaitCommand.cpp
 *
 *  Created on: Oct 5, 2012
 *      Author: Mitchell Wills
 */

#include "WaitCommand.h"
#include "Arduino.h"

WaitCommand::WaitCommand(unsigned long _time_ms) {
	time_ms = _time_ms;
}

void WaitCommand::initCommand(){
	startTime = millis();
}

void WaitCommand::run(){
}

bool WaitCommand::isFinished(){
	return millis()-startTime>=time_ms;
}
void WaitCommand::end(){
}

WaitCommand::~WaitCommand() {
}

