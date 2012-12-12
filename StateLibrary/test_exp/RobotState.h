
#include <Servo.h>
//#include "PID.h"
#define TOGGLEPIN 0
#define POT 0
#define LIFTMOTOR 7
#define WIPERMOTOR 6 
#define TOPLIMIT 31
#define BOTTOMLIMIT 32
#define XENCODER 0 // HOR A
#define XQUAD 3 // HOR B
#define XENCODERPIN 2
#define YENCODER 5   //VERT A
#define YQUAD 19 //VERT B
#define YENCODERPIN 18
#define XMOTOR 9
#define YMOTOR 8
#define SerialD Serial
#define SUM_SIZE 50 
#include <arduino.h>

typedef int coords;//[3];


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
  int SpeedX;
  int SpeedY;
  RobotState();
  void Init(int, int,  float, float, float, float); //takes the initial position
  void UpdateX(int); //adds buffered Encoder ticks into current position
  void UpdateY(int);
  /*float GoalForce; //we want the eraser to have, each side of the eraser should be at half this force
  float P,I,D;*/
  bool retracted;
  int GoalPositionX, GoalPositionY;
  float ForceSense(); //these funcitions try to attain the goal force and position of the robot, returns error
  void moveTo();
  void setPID(float, float, float, float);
  void setP(float);
  void setI(float);
  void setD(float);
  void setDES(float);
  void printPID();
  void printPos();
  void recenter(); //set origin to current position
  float* checkErase();
  float retractEraser();
  int getPosX();
  int getPosY();
  void gotoState(int, int, int);
  bool inPosition();
 


};

