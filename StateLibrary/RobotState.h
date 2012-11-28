class RobotState()
{
	private: 
	//Movement Variables:
	 int PosX; 	 //X and Y position on the whiteboard. origin is at starting position, in Encoder ticks
	 int PosY;      
	 char SpeedX;
	 char SpeedY;
	 
	 

	//Erasing Variables
	 float EraserForceLeft;	//force on the Eracser, in newtons
	 float EraserForceRight;
	 char ScrewRight;  //speeds of lead screw chnanges, as output to vex motor contorollers 
	 char ScrewLeft;
	 //There might need to be varibles here that are PID related, however they will probably be static variables in the PID functionjjj
        public:
	 RobotState(float, int); //takes the initial goal force and position
	 void UpdateX(int); //adds buffered Encoder ticks into current position
	 void UpdateY(int);
	 float GoalForce; //we want the eraser to have, each side of the eraser should be at half this force
         int GoalPosition;
	 float ForceSense(); //these funcitions try to attain the goal force and position of the robot.
	 int Move(); 
	  


}

