#include RobotState.h


RobotState::RobotState(int initialX, int initialY)
{
  PosX = initialX;
  PosY = initialY;
  SpeedX = 0;
  SpeedY = 0;
  GoalPositionX = 0;
  GoalPositionY = 0;
  GoalForce = 0;
  P = 0; 
  I = 0; 
  D = 0;
  for(int i=0; i<SUM_SIZE;i++)
  {
    sumnation[i] = 0;
  }
  sumposition = 0;
}

float RobotState::ForceSense() //these funcitions try to attain the goal force and position of the robot, returns error
{
  sumposition += 1;
  if(sumposition > SUM_SIZE)
    sumposition = 0;
  sumnation[sumposition] = GoalForce - EraserForce; 
  diff = sumnation[sumposition] - sumnation[((sumposition - 1) < 0) ?SUM_SIZE : (sumposition - 1)];
  sum = sumnation[sumposition];
  sum -= sumnation[((sumposition+1) < SUM_SIZE)? (sumposition+1) : 0];

  float ScrewOut = (sumnation[sumposition]*P)+(sum*I)-(diff*D);
  //adjust ScrewSpeed, make sure value is bounded in the future
  ScrewSpeed = max(0, min(ScrewOut, 255)); 
  return sumnation[sumposition];
}
