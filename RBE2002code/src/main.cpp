#include <Arduino.h>
#include "FireSensor.h"
//#include "Motor.h"
#include "drive.h"
#include "Ultrasonic.h"
#include "globalPins.h"
#include "PID.h"
#include <LiquidCrystal.h>
#include "QTRSensors.h"  //
#include "EEPROMex.h"


//State diagram control
enum State {STOP, WALLFOLLOW} state;
enum State2 {STOPROBOT, START} startStop;
float x;
float y;
unsigned long leftEncTicks = 0;
unsigned long rightEncTicks = 0;
float gyroValue;


//Function prototypes
void followWall();
void printLCD(int, int, char[]);
void calcXandY();
void LeftEncoderTicks();
void RightEncoderTicks();
void startOrStop();
void calibrateLineSensor();

//Object Creation
FireSensor fireSensor;
drive driveTrain;
Ultrasonic backLeftUltra(BACKLEFTULTRATRIG, BACKLEFTULTRAECHO);
Ultrasonic frontLeftUltra(FRONTLEFTULTRATRIG, FRONTLEFTULTRAECHO);
Ultrasonic frontUltra(FRONTULTRATRIG, FRONTULTRAECHO);
PID driveStraightPID;
LiquidCrystal lcd(40, 41, 42, 43, 44, 45);
QTRSensorsAnalog qtraSix((unsigned char[]) {0, 1, 2, 3, 4, 5}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensors[3];
//Still have to implement the revesre part
// Motor(digitalPin,analogPin,isReverse);
//  Motor leftMotor(29,7,true);
//  Motor rightMotor(28,6,false);

void setup() {
    //Fire Sensor
    // pinMode(29,OUTPUT);
    // pinMode(28,OUTPUT);
    // pinMode(6,OUTPUT);
    // pinMode(7, OUTPUT);
    state = STOP; //Robot will start being stopped
    startStop = START; //Robot will move once button is pushed

    pinMode(19, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(19), LeftEncoderTicks, RISING);
    pinMode(2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(2), RightEncoderTicks, RISING);
    pinMode(18, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(18), startOrStop, FALLING);

    fireSensor.initialize(); //this initializes the fire sensor
     // leftMotor.initialize();
     // rightMotor.initialize();
    calibrateLineSensor();
    driveTrain.initialize();
    driveStraightPID.setpid(5,.1,0);

    lcd.begin(16, 2);
    Serial.begin(9600);

}

void loop() {
    switch(state){
      case WALLFOLLOW:
        drive();
        break;
      case STOP:
        driveTrain.setPower(0, 0);
        break;
    }

    //Fire Sensor hey tye something
    //fireSensor.useSensor();
    //fireSensor.showAll();
    //-----------Works-------------
    // digitalWrite(29, LOW);
    // digitalWrite(28, LOW);
    // analogWrite(7,255);
    // analogWrite(6,255);
    // delay(1000);
    // digitalWrite(29, HIGH);
    // digitalWrite(28, HIGH);
    // analogWrite(7,255);
    // analogWrite(6,255);
    // delay(1000);
    //---------------------------
    //--------Testing------------
  // leftMotor.setPower(255);
  // rightMotor.setPower(255);
  //
  //driveTrain.setPower(-100,100);


}


//This function is the state level control for driving
void drive(){
  if (frontUltra.readDistance() <= 10){ //the trig and echo pin need to be set to correct values
    driveTrain.setPower(0, 0);
    calcXandY();
    //change to new switch case here, will need to turn now
  }
  if (qtraSix.readLine(sensors) > 100){ //have this if statement be if line follower is triggered
    driveTrain.setPower(0, 0);
    calcXandY();
    //change to new switch case here, will need to turn now
  }
  if(0){ //have this if statement be if flame sensor is triggered

  }
  else{
    followWall();
  }
}


//This function has the robot follow a wall using the PID
void followWall(){
  int baseRightSpeed = 90;
  int baseLeftSpeed = 90;

  int frontUltraVal = frontLeftUltra.readDistance();
  int backUltraVal = backLeftUltra.readDistance();

  float proportionalVal = driveStraightPID.calc(frontUltraVal, backUltraVal);

  //float proportionalValRight = driveStraightPID.calc(12, frontUltraVal);
  //float proportionalValLeft = driveStraightPID.calc(12, backUltraVal);

  int newLeftSpeed = baseLeftSpeed + proportionalVal;
  int newRightSpeed = baseRightSpeed - proportionalVal;
  // leftDrive.setPower(50);
  Serial.print("Left: ");
  Serial.println(newLeftSpeed);
  Serial.print("Right: ");
  Serial.println(newRightSpeed);

  char message[] = "Left/Right Motor Speeds";
  printLCD(newLeftSpeed, newRightSpeed, message);

  driveTrain.setPower(newLeftSpeed, newRightSpeed);
}


void calcXandY(){
  x = x + (leftEncTicks / 100) * cos(gyroValue);   //TODO BOTH OF THESE VALUES NEED TO BE CHANGED
  y = y + (leftEncTicks / 100) * sin(gyroValue);
}


void startOrStop(){
  switch(startStop){
    case STOPROBOT:
      state = STOP;
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("STOPPED");
      startStop = START;
      break;

    case START:
      state = WALLFOLLOW;
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("STARTING");
      startStop = STOPROBOT;
      break;
  }
}


void printLCD(int valOne, int valTwo, char message[]){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(valOne);
  lcd.setCursor(10, 0);
  lcd.print(valTwo);
  lcd.setCursor(0,1);
  lcd.print(message);
}

//count left encoder ticks
void LeftEncoderTicks() {
  leftEncTicks++;
}

//count right encoder ticks
void RightEncoderTicks() {
  rightEncTicks++;
}


void calibrateLineSensor() {
  Serial.println("Calibrating....");
  delay(500);

  qtraSix.emittersOn();
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode

  qtraSix.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  //qtrrc.calibrate();
  EEPROM.readBlock<unsigned int>(addrCalibratedMinimumOn, qtraSix.calibratedMinimumOn, 8);
  EEPROM.readBlock<unsigned int>(addrCalibratedMaximumOn, qtraSix.calibratedMaximumOn, 8);

  Serial.println("EEPROM Recall Complete");

  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration
  delay(1000);
}
