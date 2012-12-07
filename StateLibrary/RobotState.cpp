#include RobotState.h


float* RobotState::checkErase()
{
  Serial.print((int)controller->checkLoc());
  Serial.print("      ");
  return &EraserForce;
  
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
        EraserForce = (float)analogRead(0);
        return controller->operate(&EraserForce, &ScrewSpeed);
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
  //ScrewSpeed = max(0, min(ScrewSpeed, 255)); 
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