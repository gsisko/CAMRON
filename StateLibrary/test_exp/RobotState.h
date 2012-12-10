
#include <Servo.h>
//#include "PID.h"
#define TOGGLEPIN 0
#define POT 0
#define LIFTMOTOR 7
#define WIPERMOTOR 6 
#define TOPLIMIT 31
#define BOTTOMLIMIT 32
#define SerialD Serial
#define SUM_SIZE 50 
#include <arduino.h>



template <class T>
class  PIDControl
{
	private:
		//PID Variables, should really make a PID class
		T sum;
		T diff;
		T sumnation[SUM_SIZE];
		int sumposition;

	public:
		T *output;
		T desired;
		T P,I,D;
                PIDControl(T,T,T, T);
		T operate(T*, T*);
                T* checkLoc();
};



class RobotState
{
private: 
  //Movement Variables:
  int PosX; 	 //X and Y position on the whiteboard. origin is at starting position, in Encoder ticks
  int PosY;      
  char SpeedX;
  char SpeedY;



  //Erasing Variables
  
  //PID Variables, should really make a PID class
  //PID class instead!
  float EraserForce;	//force on the Eraser
  float ScrewSpeed;
  PIDControl<float> *controller;
  /*float sum;
  float diff;
  float sumnation[SUM_SIZE-1];
  int sumposition;
   */
  //There might need to be varibles here that are PID related, however they will probably be static variables in the PID function
public:
 
  RobotState();
  void Init(int, int, float, float, float, float); //takes the initial position
  void UpdateX(int); //adds buffered Encoder ticks into current position
  void UpdateY(int);
  /*float GoalForce; //we want the eraser to have, each side of the eraser should be at half this force
  float P,I,D;*/
  int GoalPositionX, GoalPositionY;
  float ForceSense(); //these funcitions try to attain the goal force and position of the robot, returns error
  int Move();
  void setPID(float, float, float, float);
  void setP(float);
  void setI(float);
  void setD(float);
  void setDES(float);
  float* checkErase();
  void printPID();



};

