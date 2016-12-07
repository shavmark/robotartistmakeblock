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
    void update();
    void endDraw();
    void move(long steps, int wait);
    
    long steps=0;
    int delaytime=800;//bugbug make api to set this? or is it just a basic setting
    
 };
  void Port::endDraw(){
    Serial.write(0xee);
    Serial.write(Move);
    Serial.println(_port);
    Serial.println(steps);
  }
 void Port::update(){
  steps = Serial.parseInt(); // data for stepper x or y

  if (steps != 0){
    digitalWrite(s1,steps < 0);
    delay(50);
  }
 }
 // move x steps 
 void Port::move(long increment, int wait) {
    for(long i=0; i < increment; i++) {
      digitalWrite(s2, HIGH);
      delayMicroseconds(wait);
      digitalWrite(s2, LOW);
      delayMicroseconds(wait); 
    }
 }
 
 // move entire way
void Port::draw() {
  if (steps != 0){
    update();
    move(abs(steps),delaytime); 
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
    Port ports[2]; // X and Y ports
    
  /* input data, byte 0 is not saved in packet
   * byte 0 - 0xee (not saved in packet)
   * byte 1 cmd for stepper X (can be NoCommand)
   * byte 2 cmd for stepper Y (can be NoCommand) 
   * other data read by command itself
   */
  private:
    void buzz(int count);
    void signon();
    void exec();
    uint8_t packet[3]; 
};

Machine machine;

void Machine::exec(){

  // sign on has no data and is set via either motor, in the future maybe each motor gets a sign on? not sure
  if (packet[1] == SignOn || packet[2] == SignOn){
      signon();
      return;
  }

  // GetState is foreither motor, in the future maybe each motor gets GetState? not sure
  if (packet[1] == GetState || packet[2] == GetState){
      Serial.write(0xee);
      Serial.write(GetState); 
      Serial.println((long)4*4000+1000); // max x
      Serial.println((long)8*5000); // max y bugbug validate this
      return;
  }

  // echo back error
   if (packet[1] != Move && packet[2] != Move){
      buzz(5);
      Serial.write(0xee);
      Serial.write(NoCommand); 
      Serial.println(packet[1]);
      Serial.println(packet[2]);
      return;
   }
   
    ports[0].update();
    ports[1].update();
    
    // move both at once, start by moving the amount of the longest run then only move the motor with remaining data
    long count = max(ports[0].steps, ports[1].steps);
    int wait;
    for (long i = 1; i <= count; i+=1){ // bugbug see if +5 works too?
       if (i <= ports[0].steps){
        wait = (i <= ports[1].steps) ? ports[0].delaytime*2 : ports[0].delaytime;
        ports[0].move(i, wait);
       }
       if (i <= ports[1].steps){
        wait = (i <= ports[0].steps) ? ports[1].delaytime*2 : ports[1].delaytime;
        ports[1].move(i, wait);
       }

       if (count < 5){
        delay(500); // start slow to ramp up
       }
       
    }

    if (ports[0].steps > 0){
      ports[0].endDraw();
    }

    if (ports[1].steps > 0){
      ports[1].endDraw();
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
  
