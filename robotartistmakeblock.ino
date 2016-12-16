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

const long xMax = (long)4*4000+1000;
const long yMax = (long)8*4000+1300;// max y bugbug validate this

enum Command : uint8_t{NoCommand, SignOn, xyPolyLine,Trace  } ;
enum DataType :uint8_t{MAKERBOT_ID=4, IKM_MAKERBOTXY=5 };
void buzz(int count);

// data always comes together
void get(long&x, long&y){
  while (!Serial.available()) {}
  x = (long)(Serial.parseFloat()*xMax);
  y = (long)(Serial.parseFloat()*yMax);
}

class Port : public MePort{
  public:

    void setup(int port);
    void move();
    long steps=0;
    void setWaitLong(bool b = true);
    int direction; // undefined to start
    int delaytime;//bugbug make api to set this? or is it just a basic setting
    private:
    int lastDirection=-1;
 };

 void Port::setWaitLong(bool b = true) {
  if (b){
    delaytime=1600;
  }
  else {
    delaytime=800;
  }
}
// move x steps (does not use local variable steps
 void Port::move() {
  if (steps == 0){
    return;
  }
  if (lastDirection != direction){
    lastDirection = direction;
    digitalWrite(s1, direction);
    delay(50);
  }

  for(long i=0; i < abs(steps); i++) {
    digitalWrite(s2, HIGH);
    delayMicroseconds(delaytime);
    digitalWrite(s2, LOW);
    delayMicroseconds(delaytime); 
  }
 }
 
void Port::setup(int port){
    setWaitLong(false);
    direction = -1;
    _port = port;
    s1 = mePort[_port].s1;
    s2 = mePort[_port].s2;
    pinMode(s1, OUTPUT);
    pinMode(s2, OUTPUT);
}

class Machine {    
  public:
    void setup(int baud);
    int update();
    Port ports[2]; // X and Y ports
    void polyline();
    void draw(long x, long y);
    void trace(long count, long index, long x, long y);
  private:
    void signon();
    void setHeader(uint8_t cmd);
    const int defaultSerialTimeout = 22000;
};
// send in binary, always the same size
void Machine::setHeader(uint8_t cmd){
   Serial.write(0xee);
   Serial.write(cmd);
}
void Machine::trace(long count, long index, long x, long y){
  setHeader(Trace);
  Serial.println(count);
  Serial.println(index);
  Serial.println(x);
  Serial.println(y);
}

 void Machine::draw(long x, long y){
  ports[0].direction =  x < 0;
  ports[1].direction =  y < 0;
  x = abs(x);
  y = abs(y);
  if (x > xMax || y > yMax){
     buzz(5);
    return; // fail safe
  }
  long count = max(x,y);
  ports[0].steps = 1; // one step 
  ports[1].steps = 1; // not sure why but y is 2x (give or take)
  for (long i = 1; i <= count; ++i){ 
     if (i <= x){
      ports[0].setWaitLong(i <= y); // move slower if x,y as things need to settle to avoid shaking
      ports[0].move();
     }
     if (i <= y){
      ports[1].setWaitLong(i <= x); 
      ports[1].move();
     }
  }
}
// read count line points from serial, this streams data
void Machine::polyline(){
  long count = Serial.parseInt();
  if (count){
    long *x = new long[count];
    long *y = new long[count];
    if (y && x){
      for (long i = 0; i < count; ++i){
        get(x[i],y[i]);
      }
      for (long i = 0; i < count; ++i){
        trace(count, i+1, x[i], y[i]);
        draw(x[i], y[i]);
      }
    }
    if (y != nullptr){
      delete y;
    }
    if (x != nullptr){
      delete x;
    }
  }
  setHeader(xyPolyLine);
}

void buzz(int count){
  if (count > 0){
    for (int i = 0; i < count; ++i){
      buzzerOn();
      delay(i+1*500);
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


void Machine::setup(int baud){
  buzz(1);

  ports[0].setup(PORT_1);
  ports[1].setup(PORT_2);

  Serial.begin(baud);
  Serial.setTimeout(defaultSerialTimeout);
  while (!Serial) {
     ; // wait for serial port to connect. Needed for native USB
  }

}
/* input data, byte 0 is not saved in packet
 * byte 0 - 0xee (not saved in packet)
 * byte 1 cmd 
 * other data read by command itself
 */
int Machine::update(){
   if (Serial.available()) {
      uint8_t packet[2]; 
      if (Serial.readBytes(packet, sizeof packet) == sizeof packet){
        if (packet[0] == 0xee){
          if (packet[1] == SignOn){
             signon();
          }
          else if (packet[1] == xyPolyLine){
            polyline();
          }
          memset(packet, 0, sizeof packet);
          return 1;
        }
      }
   }

  return 0;
}

 Machine machine;

void setup(){  
  
  machine.setup(19200);
}

void loop(){
  
  machine.update();
}
 
