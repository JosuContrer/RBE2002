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
	void clear();
	int avgTwo();

private:
  int trigPin;
  int echoPin;
	int savedReads[10];
	int prevVal;
};
