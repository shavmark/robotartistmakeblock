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

  enum Command : uint8_t{NoCommand, SignOn, SetPin, MoveTo, Move, Run, RunSpeed, SetMaxSpeed, SetAcceleration, SetSpeed, SetCurrentPosition, RunToPosition, RunSpeedToPosition, DisableOutputs, EnableOutputs, GetState, Crash  } ;
  enum DataType :uint8_t{MAKERBOT_ID=4, IKM_MAKERBOTXY=5 };
  void BinaryPacket(Command, uint8_t data1, uint8_t data2);
  void AsciiPacket(Command, long data);
  void AsciiPacket(Command, long data1, long data2);
  void AsciiPacket(Command, float data);
  void cmdHeader(Command cmd);
  void buzz(int count);
  void begin(int baud);
  void signon();
  
class serialData {    
  public:
    
    int read();         // must be called regularly to clean out Serial buffer
    void AsciiSendState();
  /* input data, byte 0 is not saved in packet
   * byte 0 - 0xee (not saved in packet)
   * byte 1 stepper index (0 == X or 1 == Y)
   * byte 2 cmd for stepper 
   * other data read by command itself
   */
   
  private:
  
   uint8_t getStepperId(){return packet[1];}
   uint8_t getCommand() {return packet[2];}
   void exec();
   MeStepper& getStepper();
   uint8_t packet[3]; 
};

serialData data;

MeStepper stepperX(PORT_1); // would like to make these members of xyRobot but their constructors make it difficult
MeStepper stepperY(PORT_2); 

//bugbug should we cook up some json? maybe for 2.0?
void serialData::AsciiSendState(){
  cmdHeader(GetState);
  Serial.println(3800); // max x
  Serial.println(3000); // max y
  Serial.println(getStepper().currentPosition()); 
  Serial.println(getStepper().targetPosition()); 
  Serial.println(getStepper().distanceToGo()); 
  Serial.println(getStepper().speed()); 
}

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
    case GetState:
      AsciiSendState();
      return;
   }


}

// simple wrapper to get correct stepper, enum helps assure range is ok
MeStepper& serialData::getStepper(){
  switch(getStepperId()){
    case 0:
    return stepperX;
    case 1:
    return stepperY;
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
void AsciiPacket(Command cmd, long data1, long data2)  {
  AsciiPacket(cmd, data1);
  Serial.println(data2);
}
void AsciiPacket(Command cmd, float data)  {
  cmdHeader(cmd);
  Serial.println(data);
}

void signon(){
  BinaryPacket(SignOn, MAKERBOT_ID, IKM_MAKERBOTXY);
   // send the x, y max info first, assuming the remaining data is comptable with other cards
  Serial.println("Makeblock flatbed");
  Serial.println("bob"); // name
}

void setup(){  
  begin(19200);
  buzz(1);
}

void loop(){
  
  data.read();
 
  stepperX.runToPosition();
  stepperY.runToPosition();
}

void begin(int baud){
  
  Serial.begin(baud);
  Serial.setTimeout(1*1000);
  while (!Serial) {
     ; // wait for serial port to connect. Needed for native USB
  }
  
  // Change these to suit your stepper if you want, but set some reasonable defaults now
  stepperX.setMaxSpeed(1000.0f);
  stepperX.setAcceleration(20000);
  stepperY.setMaxSpeed(1000.0f);
  stepperY.setAcceleration(20000);
  stepperX.moveTo(0);
  stepperY.moveTo(0);

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
  
