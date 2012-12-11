#include <MsTimer2.h>
#include <Servo.h>
Servo liftMotor;
Servo xMotor;
Servo yMotor;
int xticks = 0;
int yticks = 0;
int outputFreq = 20;
long sinceDebug = 0;
bool retracted = 0;
//#include "PID.h"
#include "RobotState.h"


RobotState myRobot;


/*
//this is the map of points that the robot has to go to
#define WAYPOINTS
int pathMesh[WAYPOINTS][3] =
{
  //format: {X,Y,shouldretract},
  {,},
  {,},
  {,},
  {,},
  {,},
  {,},
  {,},
  {,},
};
//*/



char state = '0';
void setup()
{
  pinMode(XENCODERPIN, INPUT);
  digitalWrite(XENCODERPIN, HIGH);
  pinMode(YENCODERPIN, INPUT);
  digitalWrite(YENCODERPIN, HIGH);
  pinMode(XQUAD, INPUT);
  digitalWrite(XQUAD, HIGH);
  pinMode(YQUAD, INPUT);
  digitalWrite(YQUAD, HIGH);
  
  pinMode(TOGGLEPIN, OUTPUT);
  digitalWrite(TOGGLEPIN, HIGH);
  pinMode(WIPERMOTOR, OUTPUT);
  digitalWrite(WIPERMOTOR, HIGH);
  pinMode(TOPLIMIT, INPUT);
  pinMode(BOTTOMLIMIT, INPUT);
  digitalWrite(TOPLIMIT, HIGH);
  digitalWrite(BOTTOMLIMIT, HIGH);
  liftMotor.attach(LIFTMOTOR, 1000, 2000);
  xMotor.attach(XMOTOR, 1000, 2000);
  yMotor.attach(YMOTOR, 1000, 2000);
  SerialD.println("setup");
  SerialD.begin(9600);
  myRobot.Init(0,0,0x00,5,0,0,300);
  attachInterrupt(XENCODER, xisr, RISING);
  attachInterrupt(YENCODER, yisr, RISING);
  //MsTimer2::set(outputFreq, handler);
  //MsTimer2::start();
}

void xisr()
{
  ///*
  if(digitalRead(XQUAD))
    xticks--;
  else
    xticks++;
    //*/
   
}

void yisr()
{
  if(digitalRead(YQUAD))
    yticks++;
  else
    yticks--;
}

void loop() 
{
  noInterrupts();
  myRobot.UpdateX(xticks);
  myRobot.UpdateY(yticks);
  xticks = 0;
  yticks = 0;
  interrupts();
  myRobot.printPos();
  
  float output = (!retracted)?
  myRobot.ForceSense():
  myRobot.retractEraser();
  xMotor.write(90+myRobot.SpeedX);
  yMotor.write(90+myRobot.SpeedY);
  myRobot.moveTo();
  liftMotor.write(!digitalRead(TOPLIMIT)? max(90+output, 90) : (!digitalRead(BOTTOMLIMIT)? min(90+output, 90) : 90+output) ); //checks the limit switches here
  
  if((millis()-sinceDebug) < 100)
  {
    SerialD.print("    ");
    SerialD.print(-myRobot.SpeedX);
    SerialD.print(",");
    SerialD.print(myRobot.SpeedY);
    SerialD.print("    ");
    SerialD.print(!digitalRead(TOPLIMIT));
    SerialD.print("     ");
    SerialD.print(!digitalRead(BOTTOMLIMIT));
    SerialD.print("   ");
    SerialD.print(output);
    SerialD.print("        ");
    SerialD.println(analogRead(0));
    sinceDebug = millis();
  }//*/
  if(SerialD.available())
  {
    state = SerialD.read();
    switch (state)
    {
      case 'P':
        stahp();
        SerialD.println("\n New P:");
        while(!SerialD.available());
        myRobot.setP(SerialD.parseFloat());
        state = '0';
        break;
      case 'I':
        stahp();
        SerialD.println("\n New I:");
        while(!SerialD.available());
        myRobot.setI(SerialD.parseFloat());
        state = '0';
        break;
      case 'D':
        stahp();
        SerialD.println("\n New D:");
        while(!SerialD.available());
        myRobot.setD(SerialD.parseFloat());
        state = '0';
        break;
      case 'Y':
        stahp();
        SerialD.println("\n New desired:");
        while(!SerialD.available());
        myRobot.setDES(SerialD.parseFloat());
        state = '0';
        break;
      case 'C':
        stahp();
        SerialD.print("PID and desired:");
        myRobot.printPID();
        state = '0';
        break;
      case 'K':
        stahp();
        SerialD.println("\n New X Location:");
        while(!SerialD.available());
        myRobot.GoalPositionX = SerialD.parseInt();
        state = '0';
        break;
       case 'J':
        stahp();
        SerialD.println("\n New Y Location:");
        while(!SerialD.available());
        myRobot.GoalPositionY = SerialD.parseInt();
        state = '0';
        break;
      case 'R':
        myRobot.recenter();
        myRobot.GoalPositionX = 0;
        myRobot.GoalPositionY = 0;
        state = '0';
        break;
      case 'Z':
        retracted = !retracted;
        state = '0';
        break;
      case 'W':
        digitalWrite(WIPERMOTOR, !digitalRead(WIPERMOTOR));
        state = '0';
    }
  }
}

void handler()
{
  digitalWrite(TOGGLEPIN,  !digitalRead(TOGGLEPIN));
}

void stahp()
{
  liftMotor.write(90);
  xMotor.write(90);
  yMotor.write(90);
}


