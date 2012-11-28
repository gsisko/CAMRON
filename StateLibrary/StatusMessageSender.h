/*
 * StatusMessageSender.h
 *
 * A command which sends radiation alert messages and flashes the warning led on the robot
 *
 *  Created on: Oct 7, 2012
 *      Author: Mitchell Wills
 */

#ifndef STATUSMESSAGESENDER_H_
#define STATUSMESSAGESENDER_H_

#include "command/PeriodicCommand.h"
#include "bluetooth/BluetoothReceiver.h"

#define SPENT_BLINK_RATE 100
#define NEW_BLINK_RATE 500

class StatusMessageSender: public PeriodicCommand {
public:
	StatusMessageSender(BluetoothReceiver& bluetoothReceiver);
	void initCommand();
	void run();
	void periodicRun();
	virtual ~StatusMessageSender();
private:
	BluetoothReceiver& bluetoothReceiver;
	unsigned long lastLightToggle;
};

#endif /* STATUSMESSAGESENDER_H_ */
