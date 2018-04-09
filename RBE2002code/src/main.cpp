#include <Arduino.h>
#include "FireSensor.h"
//#include "Motor.h"
#include "drive.h"
#include "Ultrasonic.h"
#include "globalPins.h"
#include "PID.h"
#include <LiquidCrystal.h>
#include "QTRSensors.h"  //not sure why this is coming up as an error
#include "EEPROMex.h"
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>


//State diagram control
enum State {STOP, WALLFOLLOW} state;
enum State2 {STOPROBOT, START} startStop;
float x;
float y;
unsigned long leftEncTicks = 0;
unsigned long rightEncTicks = 0;
float gyroValue;

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();


//Function prototypes
void followWall();
void printLCD(int, int, char[]);
void calcXandY();
void LeftEncoderTicks();
void RightEncoderTicks();
void startOrStop();
void calibrateLineSensor();
void setupIMU();

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
  //attachInterrupt(digitalPinToInterrupt(19), LeftEncoderTicks, RISING);
  pinMode(2, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(2), RightEncoderTicks, RISING);
  pinMode(20, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(20), startOrStop, RISING);

  setupIMU();
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
  lcd.print("inLoop");
  switch(state){
    case WALLFOLLOW:
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Start");
    drive();
    break;
    case STOP:
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("STOPPED");
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
  lcd.print("inDrive");
  if (frontUltra.readDistance() <= 20){ //the trig and echo pin need to be set to correct values
    driveTrain.setPower(0, 0);
    calcXandY();
    char message[] = "Left/Right Motor Speeds";
    printLCD(x,y,message);
    //change to new switch case here, will need to turn now
  }
  if (qtraSix.readLine(sensors) > 100){ //have this if statement be if line follower is triggered
    driveTrain.setPower(0, 0);
    calcXandY();
    char message[] = "Left/Right Motor Speeds";
    printLCD(x,y,message);
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
  printLCD(x, y, message);

  driveTrain.setPower(newLeftSpeed, newRightSpeed);
}


void calcXandY(){
  x = x + (leftEncTicks / 100) * cos(gyroValue);   //TODO BOTH OF THESE VALUES NEED TO BE CHANGED
  y = y + (leftEncTicks / 100) * sin(gyroValue);   //64ticks/rev max res, 2.75 in diam 5.5pi circumfrence, 17.28in/revesr
  //17.28/64 .27in/tick
}


void startOrStop(){
  switch(startStop){
    case STOPROBOT:
    state = STOP;
    startStop = START;
    break;

    case START:
    state = WALLFOLLOW;
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


//This sets up the IMU
void setupIMU()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

int gyroVal(){
  lsm.read();  /* ask it to read in the data */

  /* Get a new sensor event */
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp);

  // Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
  // Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
  // Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");
  //
  // Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" gauss");
  // Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" gauss");
  // Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" gauss");
  //
  // Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" dps");
  // Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" dps");
  // Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" dps");
  return g.gyro.z;
}
