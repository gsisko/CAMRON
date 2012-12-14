/*********************************
 ** RBE 2002
 ** Team 10: CAMRON
 ** Gabe Isko, Brendan McLeod, Lucine Batiarian
 ** RobotState.h
 ** The Header file for The RobotState and PID class.
 ********************************/
#include <Servo.h>

//Defines for all of the Pins and Interrupts our robot uses. This is common arduino practice, since if a pin that a component uses is changed
//it only needs to be changed once here and it will be reflected every time it is used later in the code.
#define TOGGLEPIN 13 	// output pin that toggles every time the PID is run. This pin also controls an LED on the arduino.
#define POT 0	     	// our potentiometer
#define LIFTMOTOR 7	// The force feedback motor 
#define WIPERMOTOR 6 	// The enable pin for our wiper motor
#define TOPLIMIT 31	// The Top limit switch for our eraser mechanism	
#define BOTTOMLIMIT 32	// the Bottom limit switch for our eraser mechanism.
#define XENCODER 0 	// Our horizontal encoder interrupt, it corresponds to HOR A on the whiteboard
#define XQUAD 3 	// Our second horizontal encoder pin, used in a quadrature setup to determine direction. It corresponds to HOR B on the whiteboard.
#define XENCODERPIN 2	// The actual pin number of the horizontal encoder, rather than the interrupt number
#define YENCODER 5   	// These are the same as the correspoinding X defines, but are for the vertical encoder. This define corresponds to VERT A on the whiteboard.
#define YQUAD 19 	// Correspsonds to VERT B on the whiteboard.
#define YENCODERPIN 18
#define XMOTOR 9
#define YMOTOR 8

//This define is for the Serial that we used to debug information. We Experimented with using Xbees to debug information
//and this define allowed us to switch back and forth between the Xbees and the wire without changing every instance that we use
//serial in our code
#define SerialD Serial
//this is the number of entries in our array that stores past values of the Potentiometer samples. 
//they get summed for the integral component of the PID controller
#define SUM_SIZE 50 
#include <arduino.h>

/**This is the definition for the PID control class. The PID class works by having a defined set of tuning contstants
 * and a sepoint, and then taking in an input, runing it through the tuning algorithm, storing the input for future reference
 * and then setting the ouput to a given variable. It operates on locations in memory rather than actuall variables, which makes it much easier to plug variables into.
 * The PID class is completely data agnostic, and is also a template class, so it can operate on any variable type. 
 * It was designed to be reused in other projects. It is supposed to be in its own header file, however, due to a linker snafu, the arduino IDE cannot handle multiple
 * header files unless they are installed libraries. There already is a PID library for arduino that is pretty much accepted as the standard, however it is designed to
 * create its own interrupt software interrupt to run on. For this project, the interrupt that runs the PID loop also checks whether the eraser should be retracted and 
 * to toggle a pin to reference the sample speed.
 */
template <class T>
class  PIDControl
{
	private:
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


/** RobotState is a class whose primary purpose is to store various information and data about where the robot is on the white board and what it should be doing.
 * It is designed to be agnostic to any actuation method so that it could potentially be used across different physical contructions of the robot, however it fails
 * that specification in a couple ways. For instance, it assumes that the range of values it will output to motors is from -90 to 90, whith 0 being the stop position.
 * This conviently matches the range of arduino's servo library, which goes from 0 to 180. It also uses the Serial library to output debugged data. One way to improve
 * this would be print debug data into string buffers and than feed that into the serial code elsewhere.
 * 
 */
class RobotState
{
private: 
  //Movement Variables:
  int PosX; 	 //X and Y position on the whiteboard. origin is at starting position, in Encoder ticks
  int PosY;      
  

  //Erasing Variables
  float EraserForce;	//force on the Eraser
  float ScrewSpeed;	//the speed that the force feedback motor should travel at
  PIDControl<float> *controller;	//the instnance of the PID controller that will control the force on the Whiteboard
public:
  int SpeedX;
  int SpeedY;
  //See the implementation file RobotState.cpp for explanation of what each of these functions do
  RobotState();
  void Init(int, int,  float, float, float, float); //takes the initial position
  void UpdateX(int); //adds buffered Encoder ticks into current position
  void UpdateY(int);
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
  float getScrewSpeed();


};
