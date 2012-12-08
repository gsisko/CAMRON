#include <Servo.h>
Servo liftMotor;



//#include "PID.h"
#include "RobotState.h"


RobotState myRobot;


char state = 'P';
void setup()
{
  pinMode(TOPLIMIT, INPUT);
  pinMode(BOTTOMLIMIT, INPUT);
  liftMotor.attach(LIFTMOTOR, 1000, 2000);
  SerialD.println("setup");
  SerialD.begin(9600);
  myRobot.Init(0,0,.3,0,0,300);
  
}
void loop() 
{
 
  float output = myRobot.ForceSense();
  liftMotor.write((digitalRead(TOPLIMIT)||digitalRead(BOTTOMLIMIT))? 90 : 90+output);
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
    }
  }
}
