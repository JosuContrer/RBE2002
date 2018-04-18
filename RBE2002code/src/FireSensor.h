#ifndef FIRESENSOR_h
#define FIRESENSOR_h

#include "Arduino.h"

/***************************************************************************************************
 * Wii Remote IR sensor  test sample code  by kako http://www.kako.com                             *
 * modified output for Wii-BlobTrack program by RobotFreak http://www.letsmakerobots.com/user/1433 *
 * modified for http://DFRobot.com by Lumi, Jan. 2014                                              *
 * modified A LOT for RBE2002 by Josue Contreras, March 2018                                       *
 ***************************************************************************************************/


/*******************************************
 * FireSensor class to control fire sensor *
 *******************************************/

class FireSensor{
public:
  FireSensor();
  void initialize();
  void Write_2bytes(byte,byte);
  void useSensor();
  void showAll();
  bool isFire();
  int getz();
  int getx();
  void centerHeight();
  void blowOutCandle();

private:
  int IRsensorAddress = 0xB0;
  int slaveAddress;
  int ledPin = 13;
  boolean ledState = false;
  byte data_buf[16];
  int i;
  int Ix[4];
  int Iy[4];
  int s;
  bool seen;
};

#endif
