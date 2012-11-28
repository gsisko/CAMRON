/*
 * PeriodicCommand.cpp
 *
 *  Created on: Oct 7, 2012
 *      Author: Mitchell Wills
 */

#include "PeriodicCommand.h"
#include "Arduino.h"

PeriodicCommand::PeriodicCommand(unsigned long _delay) {
	delay = _delay;
}

void PeriodicCommand::initCommand(){
	lastRun = 0;
}
void PeriodicCommand::run(){
	unsigned long currentTime = millis();
	if(currentTime-lastRun>=delay){
		lastRun = currentTime;
		periodicRun();
	}
}
void PeriodicCommand::end(){
}
bool PeriodicCommand::isFinished(){
	return false;
}

PeriodicCommand::~PeriodicCommand() {
}

