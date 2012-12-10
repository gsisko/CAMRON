
#include "RobotState.h"




float* RobotState::checkErase()
{
  Serial.print((int)controller->checkLoc());
  Serial.print("      ");
  return &EraserForce;
  
}

void RobotState::UpdateX(int ticks)
{
  PosX += ticks;
}
void RobotState::UpdateY(int ticks)
{
  PosY += ticks;
}
//P is initially .42 because this is the estimated 1 to 1 mapping of the input to the output across their respect ranges
//this estimation is based on that we are mapping 60% of the range of a 10 bit value to an 8 bit value.
RobotState::RobotState(){}
void RobotState::Init(int initialX, int initialY, float _P = .42, float _I = 0, float _D = 0, float _desired = 1)
{
	PosX = initialX;
	PosY = initialY;
	SpeedX = 0;
	SpeedY = 0;
	GoalPositionX = 0;
	GoalPositionY = 0;
	controller = new PIDControl<float>( _P, _I, _D, _desired);
        //Serial.println((int)&EraserForce);
	/*GoalForce = 0;
	  P = 0; 
	  I = 0; 
	  D = 0;
	  for(int i=0; i<SUM_SIZE;i++)
	  {
	  sumnation[i] = 0;
	  }
	  sumposition = 0;
	 */
}

float RobotState::ForceSense() //these funcitions try to attain the goal force and position of the robot, returns error
{
        EraserForce = (float)analogRead(POT);
        controller->operate(&EraserForce, &ScrewSpeed);
	/*
	   sumposition += 1;
	   if(sumposition > SUM_SIZE)
	   sumposition = 0;
	   sumnation[sumposition] = GoalForce - EraserForce; 
	   diff = sumnation[sumposition] - sumnation[((sumposition - 1) < 0) ?SUM_SIZE : (sumposition - 1)];
	   sum += sumnation[sumposition];
	   sum -= sumnation[((sumposition+1) < SUM_SIZE)? (sumposition+1) : 0];
	 

	   float ScrewOut = (sumnation[sumposition]*P)+(sum*I)-(diff*D);
	 */

  //adjust ScrewSpeed, make sure value is bounded in the future
  
  
  ScrewSpeed = max(-90, min(ScrewSpeed, 90)); 

  return ScrewSpeed;
}

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

void RobotState::printPos()
{
  SerialD.print(PosX);
  SerialD.print(",");
  SerialD.print(PosY);
}

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
            //Serial.println(sumnation[i]);
            sum += sumnation[i];
          }
        
        Serial.print(sum);
        Serial.print("...");
        Serial.print(diff);
        Serial.print("...    ");
	return *output = (sumnation[sumposition]*P)+(sum*I)-(diff*D);
        
}


