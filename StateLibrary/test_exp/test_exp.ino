#include <MsTimer2.h>
#include <Servo.h>
Servo liftMotor;

int outputFreq = 20;

//#include "PID.h"
#include "RobotState.h"


RobotState myRobot;


char state = 'P';
void setup()
{
  pinMode(TOGGLEPIN, OUTPUT);
  digitalWrite(TOGGLEPIN, HIGH);
  pinMode(WIPERMOTOR, OUTPUT);
  digitalWrite(WIPERMOTOR, HIGH);
  pinMode(TOPLIMIT, INPUT);
  pinMode(BOTTOMLIMIT, INPUT);
  digitalWrite(TOPLIMIT, HIGH);
  digitalWrite(BOTTOMLIMIT, HIGH);
  liftMotor.attach(LIFTMOTOR, 1000, 2000);
  SerialD.println("setup");
  SerialD.begin(9600);
  myRobot.Init(0,0,5,0,5,300);
  //MsTimer2::set(outputFreq, handler);
  //MsTimer2::start();
}
void loop() 
{
  SerialD.print(!digitalRead(TOPLIMIT));
  SerialD.print("     ");
  SerialD.print(!digitalRead(BOTTOMLIMIT));
  SerialD.print("   ");
  float output = myRobot.ForceSense();
  liftMotor.write(!digitalRead(TOPLIMIT)? max(90+output, 90) : (!digitalRead(BOTTOMLIMIT)? min(90+output, 90) : 90+output) ); //checks the limit switches here
  SerialD.print(output);
  SerialD.print("        ");
  SerialD.println(analogRead(0));
  if(SerialD.available())
  {
    state = SerialD.read();
    switch (state)
    {
      case 'P':
        SerialD.println("\n New P:");
        while(!SerialD.available());
        myRobot.setP(SerialD.parseFloat());
        state = '0';
        break;
      case 'I':
        SerialD.println("\n New I:");
        while(!SerialD.available());
        myRobot.setI(SerialD.parseFloat());
        state = '0';
        break;
      case 'D':
        SerialD.println("\n New D:");
        while(!SerialD.available());
        myRobot.setD(SerialD.parseFloat());
        state = '0';
        break;
      case 'Y':
        SerialD.println("\n New desired:");
        while(!SerialD.available());
        myRobot.setDES(SerialD.parseFloat());
        state = '0';
        break;
      case 'C':
        SerialD.print("PID and desired:");
        myRobot.printPID();
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
