/*
robotartistmakeblock - openframeworks based classes for managing robots
Copyright (c) 2016 Mark J Shavlik.  All right reserved.This file is part of myRobotSketch.

myRobotSketch is free software : you can redistribute it and / or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

myRobotSketch is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with myRobotSketch.If not, see <http://www.gnu.org/licenses/>.
*/

#include "MeOrion.h"
#include <SoftwareSerial.h>

/* commands
      1. void MeStepper::setpin(uint8_t dir_data, uint8_t step_data);
 *    2. void MeStepper::moveTo(long absolute); 
 *    3. void MeStepper::move(long relative);
 *    4. boolean MeStepper::run();
 *    5. boolean MeStepper::runSpeed();
 *    6. void MeStepper::setMaxSpeed(float speed);
 *    7. void MeStepper::setAcceleration(float acceleration);
 *    8. void MeStepper::setSpeed(float speed);
 *    9. float MeStepper::speed();
 *    10. long MeStepper::distanceToGo();
 *    11. long MeStepper::targetPosition();
 *    12. long MeStepper::currentPosition();  
 *    13. void MeStepper::setCurrentPosition(long position);  
 *    14. void MeStepper::runToPosition();
 *    15. boolean MeStepper::runSpeedToPosition();
 *    16. void MeStepper::runToNewPosition(long position);
 *    17. void MeStepper::disableOutputs();
 *    18. void MeStepper::enableOutputs();
 */

  enum Command : uint8_t{NoCommand, SignOn, SetPin, MoveTo, Move, Run, RunSpeed, SetMaxSpeed, SetAcceleration, SetSpeed, SetCurrentPosition, RunToPosition, RunSpeedToPosition, DisableOutputs, EnableOutputs, GetDistanceToGo, GetTargetPositon, GetCurrentPosition,Crash, Echo  } ;
  enum DataType :uint8_t{MAKERBOT_ID=4, IKM_MAKERBOTXY=5 };

class xyRobot
{    
  public:
    
    void begin(int baud);
    int read();         // must be called regularly to clean out Serial buffer
    void BinaryPacket(Command, uint8_t data1, uint8_t data2);
    void AsciiPacket(Command, uint8_t data1, uint8_t data2);
    
  /* input data, byte 0 is not saved in packet
   * byte 0 - 0xee (not saved in packet)
   * byte 1 stepper index (0 or 1)
   * byte 2 cmd for stepper 
   * other data read by command itself
   */

    uint8_t getStepperId(){return packet[1];}
    uint8_t getCommand() {return packet[2];}
    MeStepper& getStepper();
  private:
    void exec();
    // internal variables used for reading messages
    uint8_t packet[3];  // temporary values, moved after we confirm checksum
    int index=-1;              // -1 = waiting for new packet
};

xyRobot robot;

MeStepper stepper1(PORT_1); 
MeStepper stepper2(PORT_2); 

// simple wrapper to get correct stepper, enum helps assure range is ok
MeStepper& xyRobot::getStepper(){
  switch(getStepperId()){
    case 0:
    return stepper1;
    case 1:
    return stepper2;
  }
}

// send everything through here so we can change to print if we need to for example
void print(uint8_t val){
  Serial.print(val);
}
void write(uint8_t val){
  Serial.write(val);
}

void buzz(int count){
  if (count > 0){
    for (int i = 0; i < count; ++i){
      buzzerOn();
      delay(i+1*1000);
      buzzerOff();
    }
  
  }
}

// send binary
void xyRobot::BinaryPacket(Command cmd, uint8_t data1, uint8_t data2)  {
  
  write(0xee);
  write(data1);// ARM ID for example
  write(data2); 
  write(cmd); 
  write((255 - (data1+data2+cmd)%256));  
}
void xyRobot::AsciiPacket(Command cmd, uint8_t data1, uint8_t data2)  {
// left off here, fix client code to handle string results  
  write(0xee);
  write((255 - (data1+data2+cmd)%256));  
  print(data1);
  print(data2); 
  print(cmd); 
}

void signon(){
  robot.IDPacket(SignOn, MAKERBOT_ID, IKM_MAKERBOTXY);
  Serial.println("Makeblock flatbed");
  Serial.println("bob");
}

void setup(){   
 
  robot.begin(19200);
  buzz(1);
}

void loop(){
  
  robot.read();
  stepper1.run();
  //stepper2.run();
}

void xyRobot::begin(int baud){
  
  Serial.begin(baud);
  Serial.setTimeout(5*1000);
  while (!Serial) {
     ; // wait for serial port to connect. Needed for native USB
  }
  
  // Change these to suit your stepper if you want, but set some reasonable defaults now
  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(20000);
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(20000);
  index = -1;
}

//bugbug keep this wrapper until we are using getFloat, (a reminder)
float getFloat(){
  return Serial.parseFloat(); //bugbug does not seem to work
}

void xyRobot::exec(){
  long l;
      stepper1.move(200);
  switch(getCommand()){
   case Move:
      l = Serial.parseInt();
      IDPacket(getCommand(), l, getStepperId());
      stepper1.move(l);
      break;
   case SignOn:
      signon();
      return;
   }

}

// does not send serial data
int xyRobot::read(){
if (Serial.readBytes(packet, sizeof packet) == sizeof packet){
  if (packet[0] == 0xee){
    buzz(1);
      exec();
      memset(packet, 0, sizeof packet);
      return 1;
  }
}
  return 0;
}
/*
   while(Serial.available() > 0){
   
        if(index == -1){         // looking for new packet
            if(Serial.read() == 0xee){
               
              // new packet found
              index = 0;
            }
        }
        else   if (index >= 0){
            if(index == sizeof packet){ // packet complete
                exec();
                memset(packet, 0, sizeof packet);
                index = -1;
                return 1;
            }
            else if (index < sizeof packet){
              packet[index] = (uint8_t)Serial.read();
            }
            index++;
        }
    }
    return 0;
 
}
 */
  
