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
    if (increment != 0){
      digitalWrite(s1, increment < 0);
      delay(50);
    }

    for(long i=0; i < abs(increment); i++) {
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
    void line(long x1, long y1);
    void circle(long r);
    void ellipse(long width, long height);
    void polylineFancy(int count);
    void polylineBasic(int count);
    void bezier(long x1, long y1, long x2, long y2, long x3, long y3);
    void draw(long x, long y);

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
 void Machine::draw(long x, long y){
    ports[0].steps = x;
    ports[1].steps = y;
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
  
 }
// read count line points from serial
void Machine::polylineBasic(int count){
  for (int i = 0; i < count; ++i){
    long x = Serial.parseInt(); 
    long y = Serial.parseInt(); 
    ports[0].move(x, ports[0].delaytime*2);
    ports[1].move(y, ports[1].delaytime*2);
  }
}
void Machine::polylineFancy(int count){
  for (int i = 0; i < count; ++i){
    long x = Serial.parseInt(); 
    long y = Serial.parseInt(); 
    line(x,y);
  }
}
void Machine::ellipse(long width, long height) {

  for (long y = -height; y <= height; y++) {
    for (long x = -width; x <= width; x++) {
      if (x*x*height*height + y*y*width*width <= height*height*width*width) {
        ports[0].move(x, ports[0].delaytime*2);
        ports[1].move(y, ports[1].delaytime*2);
      }
    }
  }
}
void Machine::circle(long r) {
  float slice = 2 * M_PI / 10;
  for (int i = 0; i < 10; i++) {
    float angle = slice * i;
    long x = r * cos(angle);
    long y = r * sin(angle);
    ports[0].move(x, ports[0].delaytime*2);
    ports[1].move(y, ports[1].delaytime*2);
  }
}
// fancy line to a point from current point
void Machine::line(long x1, long y1) {
  long x0=0, y0=0;
  
  long dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
  long dy = abs(y1-y0), sy = y0<y1 ? 1 : -1; 
  long err = (dx>dy ? dx : -dy)/2, e2;
 
  for(;;){
    ports[0].move(x0, ports[0].delaytime*2);
    ports[1].move(y0, ports[1].delaytime*2);
    if (x0==x1 && y0==y1) break;
    e2 = err;
    if (e2 >-dx) { err -= dy; x0 += sx; }
    if (e2 < dy) { err += dx; y0 += sy; }
  }
}

long getPt( long n1 , long n2 , float perc ){
    long diff = n2 - n1;

    return n1 + ( diff * perc );
}    
//http://stackoverflow.com/questions/785097/how-do-i-implement-a-b%C3%A9zier-curve-in-c
void Machine::bezier(long x1, long y1, long x2, long y2, long x3, long y3){
  for( float i = 0 ; i < 1 ; i += 0.01 ){
      // The Green Line
      long xa = getPt( x1 , x2 , i );
      long ya = getPt( y1 , y2 , i );
      long xb = getPt( x2 , x3 , i );
      long yb = getPt( y2 , y3 , i );
  
      // The Black Dot
      long x = getPt( xa , xb , i );
      long y = getPt( ya , yb , i );
  
      ports[0].move(x, ports[0].delaytime*2);
      ports[1].move(y, ports[1].delaytime*2);
  } 
}
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
      Serial.write(0xee);
      Serial.write(NoCommand); 
      Serial.println(packet[1]);
      Serial.println(packet[2]);
      return;
   }

   draw(Serial.parseInt(), Serial.parseInt());

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
  
