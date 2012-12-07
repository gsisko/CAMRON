#include RobotState.h


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