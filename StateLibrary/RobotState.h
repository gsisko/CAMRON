#define SUM_SIZE 10


class RobotState
{
private: 
  //Movement Variables:
  int PosX; 	 //X and Y position on the whiteboard. origin is at starting position, in Encoder ticks
  int PosY;      
  char SpeedX;
  char SpeedY;



  //Erasing Variables
  float EraserForce;	//force on the Eracser, in newtons
  float ScrewSpeed;
  //PID Variables, should really make a PID class
  //PID class instead!
  PIDControl<float> *controller;
  /*float sum;
  float diff;
  float sumnation[SUM_SIZE-1];
  int sumposition;
   */
  //There might need to be varibles here that are PID related, however they will probably be static variables in the PID function
public:
  RobotState(int, int, float, float, float, float); //takes the initial position
  void UpdateX(int); //adds buffered Encoder ticks into current position
  void UpdateY(int);
  /*float GoalForce; //we want the eraser to have, each side of the eraser should be at half this force
  float P,I,D;*/
  int GoalPositionX, GoalPositionY;
  float ForceSense(); //these funcitions try to attain the goal force and position of the robot, returns error
  int Move(); 



};

