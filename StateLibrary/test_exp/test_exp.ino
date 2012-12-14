/*********************************
 ** RBE 2002
 ** Team 10: CAMRON
 ** Gabe Isko, Brendan McLeod, Lucine Batiarian
 ** test_exp.ino
 ** Main code for CAMRON's Operation
 ********************************/



#include <MsTimer2.h> //used to Time the PID Sampling 
#include <Servo.h>    //used to control the force feedback and Axis of motion motors
/**
 *there is quite a glut of global variables for camron, however they are mostly global constants governed by things like time, or only changed in interrupts. 
 * This is where the state system of camron first appears, which tells camron which location on the whiteboard it should go till.
 */
Servo liftMotor;  //object for the force Feedback motor
Servo xMotor;     //object for the X axis motor
Servo yMotor;     //object for the Y axis motor

//encoder ticks are buffered in these two variables before collected by a robot state object
int xticks = 0;
int yticks = 0;

int outputFreq = 20; //the time between PID samples, in milliseconds

//since variabls are used to timestamp certain events, in order to determine when they happened later
long sinceDebug = millis();       //timestamp variable for sending debug information over serial
long sinceStateChange = millis(); //timestamp variable for the change of the current waypoint of the robot


int machineState = 0; //Machine state is the position of the current waypoint in the array of waypoints
bool exeState = 0;    //exe state keeps track of whether Camron should head toward the current waypoint (1) or wait for user defined coordinates (0)

//RobotState was defined in a header file in order for portability
//Sadly, the Arduino IDE does not have the linker options available for using more than one library that isn't installed
#include "RobotState.h"

/**
 * Robot state is the actually instance of the Robot state object. It keeps track of CAMRON's position,
 * speed, force against the white board, and force feedback. It also contains the PID controller object.
 */
RobotState myRobot;

/**
 *After trying and failing to make a dynamic list of points in the robot state object, the list of Waypoints became
 *a constant arrray of coordinates in the format: X coordinate, Y coordinate, whether to retract the eraser or not, and the wait time once the position is reached.
 *WAYPOINTS is used not only to set the size of the array, but to refer to the array's size later in the program
 */
//down: 2670, 4873, 8250,10000
//const int WAYPOINTS 7
/*
	//this path wipes back and forth 5 times across a window pain
	{0,0,1,2000},
	{450,0,1,0},
	{0,0,0,0},
	{450,0,0,0},
	{0,0,0,0},
	{450,0,0,0},
	{0,0,0,0},
	{450,0,0,0},
	{0,0,0,0},
	{450,0,0,0},
	{0,0,0,0},
	//*/
#define WAYPOINTS 7
const int pathMesh[WAYPOINTS][4] =
{
	
	//starting from the top left corner, this path will 
        //go one pass over the pane and 
	//lift itself down from the bottom right corner
	{0,0,0,0},
	{450,0,0,0},
	{450,2203,0,0},
	{0,2203,0,0},
	{0,5580,0,0},
	{450,5580,0,0},
	{450,7330,1,0}
};


//state holds an opcode for debugging commands. '0' is reserved for no debugging.
char state = '0';
void setup()
{
	//digital pins are set to the correct mode and pulled high to prevent floating values.
	pinMode(XENCODERPIN, INPUT);
	digitalWrite(XENCODERPIN, HIGH);
	pinMode(YENCODERPIN, INPUT);
	digitalWrite(YENCODERPIN, HIGH);
	pinMode(XQUAD, INPUT);
	digitalWrite(XQUAD, HIGH);
	pinMode(YQUAD, INPUT);
	digitalWrite(YQUAD, HIGH);
	pinMode(TOGGLEPIN, OUTPUT);
	digitalWrite(TOGGLEPIN, HIGH);
	pinMode(WIPERMOTOR, OUTPUT);
	digitalWrite(WIPERMOTOR, HIGH);
	pinMode(TOPLIMIT, INPUT);
	pinMode(BOTTOMLIMIT, INPUT);
	digitalWrite(TOPLIMIT, HIGH);
	digitalWrite(BOTTOMLIMIT, HIGH);

	//the servo modules are attached to the correct pins, and their minimum and maximum periods are set to the accepted frequences of the 393 controllers.
	liftMotor.attach(LIFTMOTOR, 1000, 2000);
	xMotor.attach(XMOTOR, 1000, 2000);
	yMotor.attach(YMOTOR, 1000, 2000);

	//SerialD is the serial used for debugging. It is defined in a pre-Processor directive
	SerialD.begin(9600);
	//the robot stae variable is initialized with the following arguments: initial X, initial Y, Kp, Ki, Kd, and the setpoint. the last three are passed to the PID controller.
	myRobot.Init(0,0,2,0,.1,300);

	//here interrpts are attached to the X and Y encoders to buffer ticks.
	attachInterrupt(XENCODER, xisr, RISING);
	attachInterrupt(YENCODER, yisr, RISING);
	digitalWrite(WIPERMOTOR, LOW);
	//MsTimer2 is used as a timer interrupts control the PID loop. Handler will sample the force sensor and run it through PID. 
	MsTimer2::set(outputFreq, handler);
	MsTimer2::start();
}

//Hanles X encoder ticks, and checks quadrature
void xisr()
{
	if(digitalRead(XQUAD))
		xticks--;
	else
		xticks++;
}

//Hanles Y encoder ticks, and checks quadrature
void yisr()
{
	if(digitalRead(YQUAD))
		yticks++;
	else
		yticks--;
}

void loop() 
{
	//this is the section that transfers the buffered ticks into the State objects position tracking. The functions UpdateX and Y do this. the veritcal resolution is reduced
	//since, due to the worm gear used to control the Y position,  it has such a high resolution on the field that overflow errors are risked.
	noInterrupts();
	myRobot.UpdateX(xticks);
	myRobot.UpdateY(yticks); //change this resolution
	xticks = 0;
	yticks = 0;
	interrupts();

	//Here the motor Speeds are ubdated. The speeds are in terms of -90 to 90, while servo Write accepts 0-180, so a shift is applied.
	xMotor.write(90+myRobot.SpeedX);
	yMotor.write(90+myRobot.SpeedY);

	//screw speed is the PID controlled speed of lead screws that move the eraser away and closer to the board.
	//Because the eraser is attached to springs, this controls the force of the eraser on the board.
	float output = myRobot.getScrewSpeed();

	/**
	 * This block of code controls whether the robot actively tries to seek out a way point in the waypoint array. It is also responsible for 
	 * detecting whether it has reached a waypoint and for advancing it after the ammount of time has passed, as well as stopping the robot after the last waypoint has been reached.
	 * It only runs if the robot is actively seeking waypoints, and not if it is waiting for user instructions.
	 * NOTE: The robot does not currently wait for the set ammount of time. This code still has to be debugged and fixed, or re-written. 
	 */
	if(exeState)
	{
		if(myRobot.inPosition()&&(machineState < WAYPOINTS))
		{
			if(!max(0,pathMesh[machineState][3]-(max(0,(signed long)millis() - sinceStateChange))))
			{
				machineState++;
				gotoNext();
				sinceStateChange = millis();
			}
		}
		else if(machineState >= WAYPOINTS)
		{

			myRobot.recenter();
			myRobot.GoalPositionX = 0;
			myRobot.GoalPositionY = 0;
		}
	}
	//moveTo is where the robot determines the speeds that will be sent to the motors based on the goal positions and it's current position. The Goal positons are either at waypoints,
	//or specified by a user. 
	myRobot.moveTo();

	//This is where the PID controlled speed is sent to the Feedback motor. The motor is banded by two limit switches that keep it from overdriving, and having the eraser mount
	//go off the lead screws.
	liftMotor.write(!digitalRead(TOPLIMIT)? max(90+output, 90) : (!digitalRead(BOTTOMLIMIT)? min(90+output, 90) : 90+output) ); 

	/**
	 * The readings that are sent back to the robot are sent over serial here. It will only run once every tenth of a second, to avoid getting bogged down by serial transmission
	 * Here is a list of every reading the robot debugs:
	 *  -The position that the robot is at, based off encoder readings.
	 *  -The Position that the robot is trying to get to
	 *  -The Current speeds that the robot is sending to each axis motor
	 *  -the state of the limit switches on the eraser mechanism
	 *  -PID controlled output to the feedback motor
	 *  -The raw reading of the potentiometer used to force control
	 *  -The position of the current waypoint the robot is focusing on in the waypoint array-list
	 *  -The Time elapsed since the robot has tried to pursuit the current waypoint, in milliseconds.
	 *  -The Time a robot will wait before it pursuits the next way point (this actually debugs the right value, despite that the robot doesn't ever wait)
	 */
	if((millis()-sinceDebug) > 1000)
	{
		myRobot.printPos();
		SerialD.print("    ");
		SerialD.print(myRobot.GoalPositionX);
		SerialD.print(",");
		SerialD.print(myRobot.GoalPositionY);
		SerialD.print("    ");
		SerialD.print(-myRobot.SpeedX);
		SerialD.print(",");
		SerialD.print(myRobot.SpeedY);
		SerialD.print("    ");
		SerialD.print(!digitalRead(TOPLIMIT));
		SerialD.print("     ");
		SerialD.print(!digitalRead(BOTTOMLIMIT));
		SerialD.print("   ");
		SerialD.print(output);
		SerialD.print("        ");
		SerialD.print(analogRead(POT));
		SerialD.print("   ");
		SerialD.print(machineState);
		SerialD.print("   ");
		SerialD.print(myRobot.inPosition());
		SerialD.print("   ");
		SerialD.print((millis() - sinceStateChange));
		SerialD.print("   ");
		SerialD.print(
				(myRobot.inPosition()&&(machineState < WAYPOINTS))?
				pathMesh[machineState][3]-(max(0,(signed long)millis() - sinceStateChange)) : pathMesh[machineState][3]);
		SerialD.println();


	}


	/**
	 *  The next block checks to handle data a user can send to the robot in order to modify it's prorgramming. 
	 *  This can be used to tune, and even directly controler the robot.
	 *  It works by accepting an opcode from the user that tells the robot what will be modified. The opcode is followed by the data the user wants send to the robot.
	 *  while the robot waits for user data, it stops all motors in order to prevent overdriving.
	 *  The Opcodes and values that they correspond to in order to modify are:
	 *    -P: Kp for the force on the white board's PID controller
	 *    -I: Ki for the force on the white board's PID controller
	 *    -D: Kd for the force on the white board's PID controller
	 *    -Y: The setpoint for the force of the eraser on the white board.
	 *    -C: This prompts the robot to print the current Kp, Ki, Kd, and the setpoint to the screen.
	 *    -K: a new user-defined X location for the robot
	 *    -J: a new user-defined Y location for the robot
	 *    -S: a the current waypoint the robot is pursuing from the waypoint array
	 *    -N: the rate that the PID is running as a period length in milliseconds.
	 *    -E: toggles whether the robot is travelling through the list of waypoints autonomously or waiting for user input
	 *    -G: tells robot to move to the next waypoint in the array and then wait for user commands before continuing
	 *    -R: sets the robots current position to the new origin
	 *    -Z: Toggles eraser into either a retracted position or into active force sensing
	 *    -W: toggles the enable for the wiping motor
	 */
	if(SerialD.available())
	{
		state = SerialD.read();
		switch (state)
		{
			case 'P':
				stahp();
				SerialD.println("\n New P:");
				while(!SerialD.available());
				myRobot.setP(SerialD.parseFloat());
				state = '0';
				break;
			case 'I':
				stahp();
				SerialD.println("\n New I:");
				while(!SerialD.available());
				myRobot.setI(SerialD.parseFloat());
				state = '0';
				break;
			case 'D':
				stahp();
				SerialD.println("\n New D:");
				while(!SerialD.available());
				myRobot.setD(SerialD.parseFloat());
				state = '0';
				break;
			case 'Y':
				stahp();
				SerialD.println("\n New desired:");
				while(!SerialD.available());
				myRobot.setDES(SerialD.parseFloat());
				state = '0';
				break;
			case 'C':
				stahp();
				SerialD.print("PID and desired:");
				myRobot.printPID();
				state = '0';
				break;
			case 'K':
				stahp();
				SerialD.println("\n New X Location:");
				while(!SerialD.available());
				myRobot.GoalPositionX = SerialD.parseInt();
				state = '0';
				break;
			case 'J':
				stahp();
				SerialD.println("\n New Y Location:");
				while(!SerialD.available());
				myRobot.GoalPositionY = SerialD.parseInt();
				state = '0';
				break;
			case 'S':
				stahp();
				SerialD.println("\n New State:");
				while(!SerialD.available());
				machineState = SerialD.parseInt();
				machineState = max(0,min(WAYPOINTS -1, machineState));
				state = '0';
				break;
			case 'N':
				stahp();
				SerialD.println("\n New SampleRate:");
				while(!SerialD.available());
				outputFreq = SerialD.parseInt();
				MsTimer2::stop();
				MsTimer2::set(outputFreq, handler);
				MsTimer2::start();
				state = '0';
				break;
			case 'E':
				exeState = !exeState;
				sinceStateChange = millis();
				state = '0';
				break;
			case 'G':
				gotoNext();
				state = '0';
				break;
			case 'R':
				myRobot.recenter();
				myRobot.GoalPositionX = 0;
				myRobot.GoalPositionY = 0;
				state = '0';
				break;
			case 'Z':
				myRobot.retracted = !myRobot.retracted;
				state = '0';
				break;
			case 'W':
				digitalWrite(WIPERMOTOR, !digitalRead(WIPERMOTOR));
				state = '0';
		}
		sinceDebug = millis();
	}
}
//This is the ISR that samples the potentiometer for force sensing and runs the PID. It only does this if the eraser is not 
//in a retracted position. For instance when the robot moves over the barrier, the eraser needs to be in a retracted position so that it doesn't catch.
void handler()
{
	digitalWrite(TOGGLEPIN,  !digitalRead(TOGGLEPIN));
	if(!myRobot.retracted)
	{
		myRobot.ForceSense();
	}
	else
	{
		myRobot.retractEraser();
	}
}
//this is a function that will stop all motors on the robot that are actively controlled by the arduino. 
//This function is mainly used to prevent over driving while waiting for user input.
//this function does not stop the wiper becuase over driving the wiper is prevented by an external logic circuit.

void stahp()
{
	liftMotor.write(90);
	xMotor.write(90);
	yMotor.write(90);
}
//Comrin... wat R u doin? Comrin, stahp

//this functon is in charge of setting the current state in the waypoint array as the desired position for the robot.
void gotoNext()
{
	myRobot.GoalPositionX = pathMesh[machineState][0];
	myRobot.GoalPositionY = pathMesh[machineState][1];
	digitalWrite(WIPERMOTOR, !pathMesh[machineState][2]);
	myRobot.retracted = pathMesh[machineState][2];
}

