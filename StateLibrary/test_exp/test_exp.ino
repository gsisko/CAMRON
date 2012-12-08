//#include "RobotState.h"

RobotState myRobot;


char state = 'P';
void setup()
{
        Serial.println("setup");
  Serial1.begin(9600);
  Serial.begin(9600);
  myRobot.Init(0,0,.5,.001,0,300);
  
}
void loop() 
{
  
  Serial.print(myRobot.ForceSense());
  Serial.print("        ");
  Serial.println(300-analogRead(0));
  if(Serial.available())
  {
    state = Serial.read();
    switch (state)
    {
      case 'P':
        Serial.println("\n New P:");
        while(!Serial.available());
        myRobot.setP(Serial.parseFloat());
        state = '0';
        break;
      case 'I':
        Serial.println("\n New I:");
        while(!Serial.available());
        myRobot.setI(Serial.parseFloat());
        state = '0';
        break;
      case 'D':
        Serial.println("\n New D:");
        myRobot.setD(Serial.parseFloat());
        state = '0';
        break;
      case 'Y':
        Serial.println("\n New desired:");
        while(!Serial.available());
        myRobot.setDES(Serial.parseFloat());
        state = '0';
    }
  }
}
