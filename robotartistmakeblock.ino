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

/*
 * very simple driver, no state is stored other than the current draw and logic is done by the caller
 */

  enum Command : uint8_t{NoCommand, SignOn, Move, GetState  } ;
  enum DataType :uint8_t{MAKERBOT_ID=4, IKM_MAKERBOTXY=5 };

 class Port : public MePort{
  public:

    void draw();
    void setup(int port);
 
    int steps=0;
    int delaytime=800;//bugbug make api to set this
 };
void Port::draw() {
  if (steps != 0){
    digitalWrite(s1,steps < 0);
    int puresteps = abs(steps); // sign accounted for in direction
    delay(50);
    for(int i=0;i<puresteps;i++) {
      digitalWrite(s2, HIGH);
      delayMicroseconds(delaytime);
      digitalWrite(s2, LOW);
      delayMicroseconds(delaytime); 
    }
  }
}
void Port::setup(int port){
    _port = port;
    s1 = mePort[_port].s1;
    s2 = mePort[_port].s2;
    pinMode(s1, OUTPUT);
    pinMode(s2, OUTPUT);
}

class Machine {    
  public:
    void setup(int baud);
    int update();         // must be called regularly to clean out Serial buffer
    
  /* input data, byte 0 is not saved in packet
   * byte 0 - 0xee (not saved in packet)
   * byte 1 stepper index (0 == X or 1 == Y)
   * byte 2 cmd for stepper 
   * other data read by command itself
   */
  private:
    Port ports[2]; // X and Y ports
    void buzz(int count);
    void signon();
    // simple wrapper to get correct stepper, enum helps assure range is ok
    Port& getStepper(){ return ports[getStepperId()];}
    uint8_t getStepperId(){return packet[1];}
    uint8_t getCommand() {return packet[2];}
    void exec();
    uint8_t packet[3]; 
};

Machine machine;

void Machine::exec(){
  int saveSteps, delaytime;
  switch(getCommand()){
   case SignOn:
      signon();
      return;
   case Move:
      getStepper().steps = Serial.parseInt();
      if (getStepper().steps > 0){
        getStepper().draw();
      }
      Serial.write(0xee);
      Serial.write(Move);
      Serial.println(getStepper().steps);
      break;
    case GetState:
      Serial.write(0xee);
      Serial.write(GetState); 
      Serial.println(4*4000+1000); // max x
      Serial.println(8*4000+500); // max y
      return;
   }
}

void Machine::buzz(int count){
  if (count > 0){
    for (int i = 0; i < count; ++i){
      buzzerOn();
      delay(i+1*1000);
      buzzerOff();
    }
  }
}
void Machine::signon(){
   // send the x, y max info first, assuming the remaining data is comptable with other cards
  Serial.write(0xee);
  Serial.write(MAKERBOT_ID);// ARM ID for example
  Serial.write(IKM_MAKERBOTXY); 
  Serial.write(SignOn); 
  Serial.write((255 - (MAKERBOT_ID+IKM_MAKERBOTXY+SignOn)%256));  
  Serial.println("Makeblock flatbed");
  Serial.println("bob"); // name
}

void setup(){  
  
  machine.setup(19200);

}

void loop(){
  
  machine.update();
}

void Machine::setup(int baud){
  buzz(1);

  ports[0].setup(PORT_1);
  ports[1].setup(PORT_2);

  Serial.begin(baud);
  Serial.setTimeout(2000);
  while (!Serial) {
     ; // wait for serial port to connect. Needed for native USB
  }

}

int Machine::update(){
   if (Serial.available()) {
      if (Serial.readBytes(packet, sizeof packet) == sizeof packet){
        if (packet[0] == 0xee){
          exec();
          memset(packet, 0, sizeof packet);
          return 1;
        }
      }
   }

  return 0;
}
  
