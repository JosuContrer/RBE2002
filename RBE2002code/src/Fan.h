#ifndef FAN_H
#define FAN_H

/**************************************************
 * Definition of functions for control of the fan *
 **************************************************/

//////////////
//CONSTANTS //
//////////////
#define ON 1
#define OFF 0


///////////////
// FUNCTIONS //
///////////////
void fanInitialize();
void fanState(bool);

#endif
