#ifndef FireSensor_h
#define FireSensor_h

#include "Arduino.h"

// Wii Remote IR sensor  test sample code  by kako http://www.kako.com
// modified output for Wii-BlobTrack program by RobotFreak http://www.letsmakerobots.com/user/1433
// modified for http://DFRobot.com by Lumi, Jan. 2014
// modified A LOT for RBE2002 by Josue Contreras, March 2018
class FireSensor{
public:
  FireSensor();
  void initialize();
  void Write_2bytes(byte,byte);
  void useSensor();
  void showAll();

private:
  int IRsensorAddress = 0xB0;
  //int IRsensorAddress = 0x58;
  int slaveAddress;
  int ledPin = 13;
  boolean ledState = false;
  byte data_buf[16];
  int i;

  int Ix[4];
  int Iy[4];
  int s;
};
#endif
