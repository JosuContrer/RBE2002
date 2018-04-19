#include <Arduino.h>


/********************************
 * Class to control ultrasonics *
 ********************************/

class Ultrasonic {
public:
	Ultrasonic(int trigPin, int echoPin);
	void initialize();
	int readDistance();
	int avg();


private:
  int trigPin;
  int echoPin;
	int savedReads[10];
};
