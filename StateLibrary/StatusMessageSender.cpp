/*
 * StatusMessageSender.cpp
 *
 *  Created on: Oct 7, 2012
 *      Author: Mitchell Wills
 */

#include "StatusMessageSender.h"
#include "RobotState.h"
#include "Arduino.h"
#include "pins.h"

StatusMessageSender::StatusMessageSender(BluetoothReceiver& _bluetoothReceiver) : PeriodicCommand(2000), bluetoothReceiver(_bluetoothReceiver) {
}

void StatusMessageSender::initCommand(){
	pinMode(ROD_LED_PIN, OUTPUT);
	lastLightToggle = 0;
	PeriodicCommand::initCommand();
}

void StatusMessageSender::run(){
	switch(RobotState::goal){
	case PickingUpSpent:
	case DrivingToSpentStorage:
	case DepositingSpent:
		if(millis()-lastLightToggle>=SPENT_BLINK_RATE){
			digitalWrite(ROD_LED_PIN, !digitalRead(ROD_LED_PIN));
			lastLightToggle = millis();
		}
		break;
	case ExtractingNewFromStorage:
	case GoingToReactorWithNew:
	case InsertingNewRod:
		if(millis()-lastLightToggle>=NEW_BLINK_RATE){
			digitalWrite(ROD_LED_PIN, !digitalRead(ROD_LED_PIN));
			lastLightToggle = millis();
		}
		break;

	case GoingToReactor:
	case RetreatingFromSpentStorage:
	case DrivingToNew:
	case Start:
	case Done:
		digitalWrite(ROD_LED_PIN, LOW);
		break;
	}
	PeriodicCommand::run();
}

void StatusMessageSender::periodicRun(){
	switch(RobotState::goal){

	case PickingUpSpent:
	case DrivingToSpentStorage:
	case DepositingSpent:
		bluetoothReceiver.sendRadiationAlert(false);
		break;
	case ExtractingNewFromStorage:
	case GoingToReactorWithNew:
	case InsertingNewRod:
		bluetoothReceiver.sendRadiationAlert(true);
		break;

	case GoingToReactor:
	case RetreatingFromSpentStorage:
	case DrivingToNew:
	case Start:
	case Done:
		break;
	}
}

StatusMessageSender::~StatusMessageSender() {
}

