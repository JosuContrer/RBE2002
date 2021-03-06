#ifndef GLOBALPINS_H
#define GLOBALPINS_H

#include <Arduino.h>

/*********************************************************
 * Used to delcare pin numbers for all sensors/actuators *
 *********************************************************/


///////////////
//DRIVETRAIN //
///////////////
#define DLEFTMOTOR 27
#define ALEFTMOTOR 7
#define DRIGHTMOTOR 29
#define ARIGHTMOTOR 6


///////////
//BUTTON //
///////////
#define BUTTON 20


///////////////////////
//ULTRASONIC SENSORS //
///////////////////////
#define BACKLEFTULTRATRIG 22
#define BACKLEFTULTRAECHO 23

#define FRONTLEFTULTRATRIG 24
#define FRONTLEFTULTRAECHO 25

#define FRONTULTRATRIG 19
#define FRONTULTRAECHO 18

#define SIDEULTRATRIG 17
#define SIDEULTRAECHO 16


//////////////////
//LINE FOLLOWER //
//////////////////
#define NUM_SENSORS             6 // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4 // average 4 analog samples per sensor reading
#define EMITTER_PIN             2 // emitter is controlled by digital pin 2
#define addrCalibratedMinimumOn 0
#define addrCalibratedMaximumOn 100

#define LINEFOLLOWERONE A5
#define LINEFOLLOWERTWO A4


/////////
// FAN //
/////////
#define FANPIN 10
#define FANSERVOPIN 8

#define IRPIN A3

#define REDLED 14
#define BLUELED 15


#endif
