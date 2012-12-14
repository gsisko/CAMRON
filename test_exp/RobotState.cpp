/*********************************
 ** RBE 2002
 ** Team 10: CAMRON
 ** Gabe Isko, Brendan McLeod, Lucine Batiarian
 ** RobotState.cpp
 ** Implenation of the final libraries.
 ********************************/
#include "RobotState.h"


//some getters and setters for the positon variables 
int RobotState::getPosX()
{
	return PosX;
}
int RobotState::getPosY()
{
	return PosY;
}
//instead of dirctly setting the position, the encoder ticks are just added to the current position
void RobotState::UpdateX(int ticks)
{
	PosX += ticks;
}
void RobotState::UpdateY(int ticks)
{
	PosY += ticks;
}
//All setup is done in an Init function that is called in setup(). This is good practice in arduino, because it allows arduino to
//go through setup before other objects start using it.
//The PID controller for the robot is initially set to just a 1 to 1 proportional control
RobotState::RobotState(){}
void RobotState::Init(int initialX, int initialY, float _P = 1, float _I = 0, float _D = 0, float _desired = 1)
{
	bool retracted = 0;
	PosX = initialX;
	PosY = initialY;
	SpeedX = 0;
	SpeedY = 0;
	GoalPositionX = 0;
	GoalPositionY = 0;
	controller = new PIDControl<float>( _P, _I, _D, _desired);
}
//this function retracts the eraser
float RobotState::retractEraser()
{
	ScrewSpeed = -90;
	return ScrewSpeed;
}
//This function samples the potentiomter and then runs the PID controller. It also bounds the otput to the servo.
float RobotState::ForceSense() 
{
	EraserForce = (float)analogRead(POT);
	controller->operate(&EraserForce, &ScrewSpeed);

	//adjust ScrewSpeed
	ScrewSpeed = max(-90, min(ScrewSpeed, 90)); 

	return ScrewSpeed;
}
//this function decides what speed the axis motors should be going at in order to move to a desired position
//the X speed is bounded at a reduced speed in order to ensure that the robot will move slow enogh for the eraser to be effective. 
//even at top speed, the robot moves slow enough in the Y direction for it to not need bonding, due to the worm gear drive used in the vertical drive system.
void RobotState::moveTo()
{

	noInterrupts();
	SpeedX = GoalPositionX - PosX;
	SpeedY = GoalPositionY - PosY;
	interrupts();
	SpeedX = max(-70, min(-SpeedX, 70)); 
	SpeedY = max(-90, min(-SpeedY, 90)); 

}
//Seters for the constants and setpoints of the PID controller
void RobotState::setPID(float _P, float _I, float _D, float _desired)
{
	controller->P = _P;
	controller->I = _I;
	controller->D = _D;
	controller->desired = _desired;
}
void RobotState::setP(float _P)
{controller->P = _P;}
void RobotState::setI(float _I)
{controller->I = _I;}
void RobotState::setD(float _D)
{controller->D = _D;}
void RobotState::setDES(float _desired)
{controller->desired = _desired;}

//a debuging function for the PID controller
void RobotState::printPID()
{
	SerialD.print(controller->P);
	SerialD.print("      ");
	SerialD.print(controller->I);
	SerialD.print("      ");
	SerialD.print(controller->D);
	SerialD.print("      ");
	SerialD.println(controller->desired);
	while(!SerialD.available());
	SerialD.flush();
}
//a function that sets the robots current position as the origin
void RobotState::recenter()
{
	PosX = 0;
	PosY = 0;
}

//a debugging function for the positon of the robot
void RobotState::printPos()
{
	SerialD.print(PosX);
	SerialD.print(",");
	SerialD.print(PosY);
}
//because A PID controller is only designed to be implemented by other classes, it is safe to use a constructor instead of an init functon.
//The constructor allows us to initiallize the tunning constants and the initial setponint.
	template <class T>
PIDControl<T>::PIDControl(T _P,T _I,T _D, T _desired)
{



	desired = _desired;
	P = _P; 
	I = _I; 
	D = _D;
	for(int i=0; i<SUM_SIZE;i++)
	{
		sumnation[i] = 0;
	}
	sumposition = 0;
	sum = 0;
	diff = 0;
}

//This is the function that actually runs the PID loop
//it determines the necesarry values by storing values in an array and summing them for the integral
//or finding the difference between the current value and the last value for the derivative.
//One improvement that could be made would be to find the average value to use inplace of the sum for the integral control, so that
//the time that we integrate over does not need to be accounted for in tuning.
//this was not a problem since the integral control isn't needed in this robot due to the acurrate response of the lead screws.
	template <class T>
T PIDControl<T>::operate(T *input, T* output)
{
	diff = sumnation[sumposition];
	sumposition += 1;
	if(sumposition >= SUM_SIZE)
		sumposition = 0;
	sumnation[sumposition] = desired - *input;
	diff = sumnation[sumposition] - diff; 
	int i = SUM_SIZE;
	sum = 0;
	for(i = 0; i < SUM_SIZE; i++)
	{
		sum += sumnation[i];
	}
	return *output = (sumnation[sumposition]*P)+(sum*I)-(diff*D);
}

//setter for the goal position of the robot
void RobotState::gotoState(int _goalX, int _goalY, int _ret)
{
	GoalPositionX = _goalX;
	GoalPositionY = _goalY;
	retracted = _ret;
}
//A function that checks whether or not the robot has reached it's desired position. There is a deadband that is acconted for.
//An improvement that still needs to be made is to account for the deadband error on the X axis.
//the error on the Y axis is negligible due to the resolution of the Y motor encoder.
bool RobotState::inPosition()
{
	return (abs(GoalPositionY - PosY) < 70)&&(abs(GoalPositionX - PosX) < 70);
}
//A getter for the force feedback motor speed.
float RobotState::getScrewSpeed()
{
	noInterrupts();
	return ScrewSpeed;
	interrupts();
}
