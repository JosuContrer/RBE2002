
class Ultrasonic {
public:
	Ultrasonic(int trigPin, int echoPin, int duration, int distance);
	void initialize();
	int readDistance();
  void setValues(int, int);

private:
  int trigPin;
  int echoPin;
  int duration;
  int distance;
};
