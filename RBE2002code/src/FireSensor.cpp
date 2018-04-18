#include "FireSensor.h"
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "PID.h"
#include "Fan.h"


#define STARTVAL 90  //where Servo normally starts
#define CENTER_VAL 90 //Where the flame is centered according to the sensor

//Global Variables
extern Servo fanServo;
PID centerFan;
//Constructor
FireSensor::FireSensor(){}

//Method that writes to IR Sensor
void FireSensor::Write_2bytes(byte d1, byte d2)
{
    Wire.beginTransmission(slaveAddress);
    Wire.write(d1);
    Wire.write(d2);
    Wire.endTransmission();
}

//Method that initalizes the IR sensor
void FireSensor::initialize(){
  // This results in 0x21 as the address to pass to TWI
  slaveAddress = IRsensorAddress >> 1;
  Serial.begin(19200);
  // Set the LED pin as output
  pinMode(ledPin, OUTPUT);
  Wire.begin();
  // IR sensor initialize
  Write_2bytes(0x30,0x01); delay(10);
  Write_2bytes(0x30,0x08); delay(10);
  Write_2bytes(0x06,0x90); delay(10);
  Write_2bytes(0x08,0xC0); delay(10);
  Write_2bytes(0x1A,0x40); delay(10);
  Write_2bytes(0x33,0x33); delay(10);
  delay(100);
  centerFan.setpid(1,.2,.02);    //create PID to center the flame in the center
}

//Method that saves and array of x and y from the sensor reading
void FireSensor::useSensor(){
    //IR sensor read
    Wire.beginTransmission(slaveAddress);
    Wire.write(0x36);
    Wire.endTransmission();

     // Request the 2 byte heading (MSB comes first)
    Wire.requestFrom(slaveAddress, 16);
    for (i=0;i<16;i++) { data_buf[i]=0; }
    i=0;
    while(Wire.available() && i < 16) {
        data_buf[i] = Wire.read();
        i++;
    }

    Ix[0] = data_buf[1];
    Iy[0] = data_buf[2];
    s   = data_buf[3];
    Ix[0] += (s & 0x30) <<4;
    Iy[0] += (s & 0xC0) <<2;

    Ix[1] = data_buf[4];
    Iy[1] = data_buf[5];
    s   = data_buf[6];
    Ix[1] += (s & 0x30) <<4;
    Iy[1] += (s & 0xC0) <<2;

    Ix[2] = data_buf[7];
    Iy[2] = data_buf[8];
    s   = data_buf[9];
    Ix[2] += (s & 0x30) <<4;
    Iy[2] += (s & 0xC0) <<2;

    Ix[3] = data_buf[10];
    Iy[3] = data_buf[11];
    s   = data_buf[12];
    Ix[3] += (s & 0x30) <<4;
    Iy[3] += (s & 0xC0) <<2;
}

//Method that prints the Sensor's readings
void FireSensor::showAll(){
  for(i=0; i<4; i++)
   {
     if (Ix[i] < 1000)
       Serial.print("");
     if (Ix[i] < 100)
       Serial.print("");
     if (Ix[i] < 10)
       Serial.print("");
     Serial.print( int(Ix[i]) );
     Serial.print(",");
     if (Iy[i] < 1000)
       Serial.print("");
     if (Iy[i] < 100)
       Serial.print("");
     if (Iy[i] < 10)
       Serial.print("");
     Serial.print( int(Iy[i]) );
     if (i<3)
       Serial.print(",");
   }
   Serial.println("");
   delay(15);
}

//Return true if the fire sensor is triggered
bool FireSensor::isFire(){
  if((Ix[0]>1023) && (Iy[0] >1023)){
    Serial.print("Fire Seen");
    seen = true;
  }else{
    seen = false;
  }
  return seen;
}

//Return the z value for the fire
int FireSensor::getz(){
  return Iy[0];
}

//Return the z value for the fire
int FireSensor::getx(){
  return Ix[0];
}

//move servo so flame is centered with servo
void FireSensor::centerHeight(){
  // static int i = STARTVAL;      //TODO: Change this to be appropriate starting value
  // if(getz() < CENTER_VAL){
  //   fanServo.write(++i);
  // }
  int fanError = centerFan.calc(CENTER_VAL, getz()); //Use PID to center flame
  fanServo.write(STARTVAL + fanError); //add error to initial starting position
}

void FireSensor::blowOutCandle(){
  if(isFire()){
    fanState(ON);
  }
  //have fan oscillate up and down in order to be sure to extinguish flame
//WARNING:
      //This code is set to go from 0 to 180, but the range of the servo is probably less
      //Do NOT run this code until the range is determined, otherwise the servo will try and go the
      //full 180 degrees and make either stall the servo or break something on the robot
  for(int i = 0; i < 180; i++){
    fanServo.write(i);
  }
  for(i = 180; i > 0; i--){
    fanServo.write(i);
  }
}
