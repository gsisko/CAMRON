#include <MsTimer2.h>
#include <Servo.h>
Servo liftMotor;
Servo xMotor;
Servo yMotor;
int xticks = 0;
int yticks = 0;
int outputFreq = 20;
long sinceDebug = 0;
int machineState = 0;
bool exeState = 0;
//#include "PID.h"
#include "RobotState.h"

bool exeMachine = 0;
RobotState myRobot;

#define WAYPOINTS 3
int pathMesh[WAYPOINTS][3] =
{
  {0,0,1},
  {0,200,0},
  {0,-200,0}
};



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
  SerialD.begin(9600);
  myRobot.Init(0,0,5,0,0,300);
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
  myRobot.UpdateY(yticks/10);
  xticks = 0;
  yticks = 0;
  interrupts();
  
  float output = (!myRobot.retracted)?
  myRobot.ForceSense():
  myRobot.retractEraser();
  xMotor.write(90+myRobot.SpeedX);
  yMotor.write(90+myRobot.SpeedY);
  if(exeState)
  {
    if(myRobot.inPosition()&&(machineState < WAYPOINTS))
    {
      machineState++;
      gotoNext();
    }
  }
  myRobot.moveTo();
  liftMotor.write(!digitalRead(TOPLIMIT)? max(90+output, 90) : (!digitalRead(BOTTOMLIMIT)? min(90+output, 90) : 90+output) ); //checks the limit switches here
  
  if((millis()-sinceDebug) < 100)
  {
    myRobot.printPos();
    SerialD.print("    ");
    SerialD.print(myRobot.GoalPositionX);
    SerialD.print(",");
    SerialD.print(myRobot.GoalPositionY);
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
    SerialD.print(analogRead(0));
    SerialD.print("   ");
    SerialD.print(machineState);
    SerialD.print("   ");
    SerialD.print(myRobot.inPosition());
    SerialD.println();
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
      case 'S':
        stahp();
        SerialD.println("\n New State:");
        while(!SerialD.available());
        machineState = SerialD.parseInt();
        machineState = max(0,min(WAYPOINTS -1, machineState));
        state = '0';
        break;
      case 'E':
        exeState = !exeState;
        state = '0';
        break;
      case 'G':
        gotoNext();
        state = '0';
        break;
      case 'R':
        myRobot.recenter();
        myRobot.GoalPositionX = 0;
        myRobot.GoalPositionY = 0;
        state = '0';
        break;
      case 'Z':
        myRobot.retracted = !myRobot.retracted;
        state = '0';
        break;
      case 'W':
        digitalWrite(WIPERMOTOR, !digitalRead(WIPERMOTOR));
        state = '0';
    }
  sinceDebug = millis();
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
void gotoNext()
{
  myRobot.GoalPositionX = pathMesh[machineState][0];
  myRobot.GoalPositionY = pathMesh[machineState][1];
  myRobot.retracted = pathMesh[machineState][2];
}

