
class Ultrasonic {
public:
	Ultrasonic(int trigPin, int echoPin);
	void initialize();
	int readDistance();


private:
  int trigPin;
  int echoPin;
};
