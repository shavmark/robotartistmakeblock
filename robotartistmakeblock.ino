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

  enum Command : uint8_t{NoCommand, SignOn, SetPin, MoveTo, Move, Run, RunSpeed, SetMaxSpeed, SetAcceleration, SetSpeed, SetCurrentPosition, RunToPosition, RunSpeedToPosition, DisableOutputs, EnableOutputs, GetDistanceToGo, GetTargetPositon, GetCurrentPosition,  Crash, Echo  } ;
  enum DataType :uint8_t{MAKERBOT_ID=4, IKM_MAKERBOTXY=5 };
  void BinaryPacket(Command, uint8_t data1, uint8_t data2);
  void AsciiPacket(Command, long data);
  void AsciiPacket(Command, float data);
  void cmdHeader(Command cmd);
  void buzz(int count);
  void begin(int baud);
  void signon();
  
class serialData {    
  public:
    
    int read();         // must be called regularly to clean out Serial buffer
    
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
};

serialData data;

MeStepper stepper1(PORT_1); // would like to make these members of xyRobot but their constructors make it difficult
MeStepper stepper2(PORT_2); 

void serialData::exec(){

  if (getCommand() == SignOn){
      signon();
      return;
  }
  
  switch(getCommand()){
   case Move:
      getStepper().move(Serial.parseInt());
      break;
   case MoveTo:
      getStepper().moveTo(Serial.parseInt());
      break;
    case SetAcceleration:
      getStepper().setAcceleration(Serial.parseFloat());
      break;
    case SetSpeed:
      getStepper().setSpeed(Serial.parseFloat());
      break;
    case GetCurrentPosition:
      return AsciiPacket(GetCurrentPosition, getStepper().currentPosition());
   }

   cmdHeader(getCommand()); // let people know we made it this far

}

// simple wrapper to get correct stepper, enum helps assure range is ok
MeStepper& serialData::getStepper(){
  switch(getStepperId()){
    case 0:
    return stepper1;
    case 1:
    return stepper2;
  }
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

// send binary, this is compatable with the Trossen sign on data to make things easier, down to the byte order 
void BinaryPacket(Command cmd, uint8_t data1, uint8_t data2)  {
  Serial.write(0xee);
  Serial.write(data1);// ARM ID for example
  Serial.write(data2); 
  Serial.write(cmd); 
  Serial.write((255 - (data1+data2+cmd)%256));  
}
// Command will determine data type
void cmdHeader(Command cmd){
  Serial.write(0xee);
  Serial.write(cmd); 
}
void AsciiPacket(Command cmd, long data)  {
  cmdHeader(cmd);
  Serial.println(data);
}
void AsciiPacket(Command cmd, float data)  {
  cmdHeader(cmd);
  Serial.println(data);
}

void signon(){
  BinaryPacket(SignOn, MAKERBOT_ID, IKM_MAKERBOTXY);
  Serial.println("Makeblock flatbed");
  Serial.println("bob");
}

void setup(){  
  begin(19200);
  buzz(1);
}

void loop(){
  
  data.read();
 
  stepper1.runToPosition();
  stepper2.runToPosition();
}

void begin(int baud){
  
  Serial.begin(baud);
  Serial.setTimeout(25*1000);
  while (!Serial) {
     ; // wait for serial port to connect. Needed for native USB
  }
  
  // Change these to suit your stepper if you want, but set some reasonable defaults now
  stepper1.setMaxSpeed(1000.0f);
  stepper1.setAcceleration(20000);
  stepper2.setMaxSpeed(1000.0f);
  stepper2.setAcceleration(20000);

}


// does not send serial data
int serialData::read(){
if (Serial.readBytes(packet, sizeof packet) == sizeof packet){
  if (packet[0] == 0xee){
      exec();
      memset(packet, 0, sizeof packet);
      return 1;
  }
}
  return 0;
}
  
