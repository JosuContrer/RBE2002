#include "FireSensor.h"
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "PID.h"
#include "Fan.h"


//////////////
//CONSTANTS //
//////////////
#define STARTVAL 80  //where Servo normally starts
#define CENTER_VAL 300 //Where the flame is centered according to the sensor


//////////////////////
// Global Variables //
//////////////////////
extern Servo fanServo;
PID centerFan;
Fan fan;

/////////////////
// Constructor //
/////////////////
FireSensor::FireSensor(){}


/**
* Method that writes to IR Sensor
* @param d1 Input byte 1
* @param d2 Input byte 2
*/
void FireSensor::Write_2bytes(byte d1, byte d2)
{
  Wire.beginTransmission(slaveAddress);
  Wire.write(d1);
  Wire.write(d2);
  Wire.endTransmission();
}


/**
* Initializes IR sensor
*/
void FireSensor::initialize(){
  slaveAddress = IRsensorAddress >> 1;  // This results in 0x21 as the address to pass to TWI
  Serial.begin(19200);

  pinMode(ledPin, OUTPUT);  // Set the LED pin as output

  Wire.begin(); // IR sensor initialize
  Write_2bytes(0x30,0x01); delay(10);
  Write_2bytes(0x30,0x08); delay(10);
  Write_2bytes(0x06,0x90); delay(10);
  Write_2bytes(0x08,0xC0); delay(10);
  Write_2bytes(0x1A,0x40); delay(10);
  Write_2bytes(0x33,0x33); delay(10);
  delay(100);

  //centerFan.setpid(0.03,0.001,0); //Work good but slow
  //centerFan.setpid(0.03,0.006,0); //Better when working alone
  centerFan.setpid(0.02,0.004,0.0001); //REVIEW: A lot better
}


/**
* Saves array of x and y from sensor reading
*/
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


/**
* Prints flame sensors readings
*/
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


/**
* Returns true if flame sensor is triggered
* @return True if flame sensor triggered
*/
bool FireSensor::isFire(){
  if((Ix[0]<1023) || (Iy[0]<1023)){
    Serial.print("Fire Seen");
    seen = true;
  }else{
    seen = false;
  }
  return seen;
}


/**
* Get z value for the flame
* @return Z value
*/
int FireSensor::getz(){
  return Iy[0];
}


/**
* Return x value for the flame
* @return X value
*/
int FireSensor::getx(){
  return Ix[0];
}


//REVIEW:
/**
* Moves servo so flame is centered with servo
*/
bool FireSensor::centerHeight(){
  if(isFire()){
    int fanError =  centerFan.calc(CENTER_VAL, getz()); //Use PID to center flame
    int newError = STARTVAL - fanError;
    if (newError>178){
      newError=178;
    }
    if(newError<2){
      newError=0;
    }
    fanServo.write(newError); //add error to initial starting position
    //Serial.println(fanError);
    if(fanError <= 2){
      return true; //When it sees the candle
    }else{
      return false;
    }
  }
}


//REVIEW:
/**
* Blows out candle
*/
void FireSensor::blowOutCandle(){
  //if(isFire()){
  fan.maxPower(true); //turns fan on
  //}

  //have fan oscillate up and down in order to be sure to extinguish flame

  /***********************************************************************************************
  * WARNING                                                                                     *
  * This code is set to go from 0 to 180, but the range of the servo is probably less           *
  * Do NOT run this code until the range is determined, otherwise the servo will try and go the *
  * full 180 degrees and make either stall the servo or break something on the robot            *
  ***********************************************************************************************/
  oscillate(3);
  fan.maxPower(false);
}

void FireSensor::oscillate(int count){
  for(int m = 0; m < count; m++){
    for(int i = 60; i < 120; i++){
      fanServo.write(i);
      delay(10);
    }
    for(i = 120; i > 60; i--){
      fanServo.write(i);
      delay(10);
    }
  }
}
