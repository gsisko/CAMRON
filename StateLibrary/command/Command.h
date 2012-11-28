/*
 * Command.h
 *
 * The heat of the command library
 * A command consists of an initalize method which is called when the command starts
 * a run method which is called continuously while the command is running
 * an isFinished method which indicates if the command is finished running
 * a end method which is called when the command ends
 *
 *  Created on: Sep 11, 2012
 *      Author: Mitchell
 */

#ifndef COMMAND_H_
#define COMMAND_H_

class Command {
public:
	virtual void initCommand() = 0;
	virtual void run() = 0;
	virtual bool isFinished() = 0;
	virtual void end() = 0;
};

#endif /* COMMAND_H_ */
