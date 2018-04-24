/*
 *	Sharp GP2Y0A02YK0F IR Distance Sensor 20-150 cm
 *	(Parallax Part Number: #28997)
 *	IR Distance Sensor Library
 */

#include <IRrangeFinder.h>
#include "globalPins.h"
// Constructor

GP2Y0A02YK0F::GP2Y0A02YK0F() {

}

// Default Begin method: sensorPin = A0.

void GP2Y0A02YK0F::begin() {
	begin (IRPIN);
}

// Begin method - assign sensorPin as the analog sensor input
// When you use begin() without variables the default input A0 is assumed.

void GP2Y0A02YK0F::begin(int sensorPin) {
  	pinMode(sensorPin, INPUT);
	_sensorPin = sensorPin;
}

// getDistanceRaw() Method: Returns the distance as a raw value: ADC output: 0 -> 1023

int GP2Y0A02YK0F::getDistanceRaw() {
		return (analogRead(_sensorPin));
}

// getDistanceCentimeter() Method: Returns the distance in centimeters

int GP2Y0A02YK0F::getDistanceCentimeter() {
	float sensorValue = analogRead(_sensorPin);
  	float cm = 10650.08 * pow(sensorValue,-0.935) - 10;
  	return roundf(cm);
}

// isCloser Method: check whether the distance to the detected object is smaller than a given threshold

boolean GP2Y0A02YK0F::isCloser(int threshold) {
	if (threshold>getDistanceCentimeter()) {
		return (true);
	} else {
		return (false);
	}
}

// isFarther Method: check whether the distance to the detected object is smaller than a given threshold

boolean GP2Y0A02YK0F::isFarther(int threshold) {
	if (threshold<getDistanceCentimeter()) {
		return true;
	} else {
		return false;
	}
}

int GP2Y0A02YK0F::avg(){
	int avg=0, sum=0,divider=0;

	for(int i=1; i < 9; i++){
		savedReads[9-i]=savedReads[9-i-1]; //shift array values to make room for new
	}

	savedReads[0]=getDistanceCentimeter(); //add new distance to array

	//Count number of values
	for(int i=0; i<5; i++){
		if(savedReads[i]<30 && savedReads[i]>0){
			sum+=savedReads[i];
			divider++;
		}
	}

	//Take average of values
	if(divider!=0){
		avg=sum/divider;
	}

	else{
		avg = 40;//random number that wont trigger anything
	}

	//Wrap protection
	if(!(avg<40&&avg>0)){
		avg=40;
	}

	return avg;
}

void GP2Y0A02YK0F::clear(){
	for(int i = 0; i < 10; i++){
		savedReads[i] = 39;
	}
}
