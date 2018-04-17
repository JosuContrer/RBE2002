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
#include <SPI.h>//not used but needed?
#include <Adafruit_BNO055.h>
#include "States.h"
#include "MotorStates.h"
#include <Bounce2.h>
#include "Fan.h"

//CONSTANTS
#define baseLeftSpeed_120 120
#define baseRightSpeed_120 120

//State diagram control
enum State {STOP, WALLFOLLOW,TURN, FLAME} state;
enum State2 {STOPROBOT, START} startStop;
enum pidSelect {WALL,TURNING} pidSel;
enum turner {LEFT,RIGHT} turnDir;
float x;
float y;
unsigned long leftEncTicks = 0;
unsigned long rightEncTicks = 0;
int stop;
int desiredGyro;
int previousDistance=0;
// i2c
Adafruit_BNO055 bno = Adafruit_BNO055();
int baseRightSpeed =baseRightSpeed_120;
int baseLeftSpeed = baseLeftSpeed_120;
Bounce debouncer = Bounce();
//Function prototypes
void driveFollow();
void followWall();
void printLCD(int, int, char[]);
void calcXandY();
void LeftEncoderTicks();
void RightEncoderTicks();
void startOrStop();
void calibrateLineSensor();
void setupIMU();
void turn();
void drivePID();
void gyroVal();


//Object Creation
FireSensor fireSensor;
drive driveTrain;
Ultrasonic backLeftUltra(BACKLEFTULTRATRIG, BACKLEFTULTRAECHO);
Ultrasonic frontLeftUltra(FRONTLEFTULTRATRIG, FRONTLEFTULTRAECHO);
Ultrasonic frontUltra(FRONTULTRATRIG, FRONTULTRAECHO);
Ultrasonic sideUltra(SIDEULTRATRIG, SIDEULTRAECHO);
PID driveStraightPID;
PID turnPID;
LiquidCrystal lcd(40, 41, 42, 43, 44, 45);
QTRSensorsAnalog qtraSix((unsigned char[]) {0, 1, 2, 3, 4, 5}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensors[3];

int frontUltraVal=0;
int backUltraVal=0;

float proportionalVal;
int newLeftSpeed;
int newRightSpeed;
float gyro;

//Testing
MotorStates test1;


void turn(int turnDir){//so you can just call turn(LEFT)ezpz
  switch(turnDir){
    case LEFT:
    baseLeftSpeed=-90;
    desiredGyro=gyro-90;
    if (desiredGyro<0){
      desiredGyro+=360;
    }
    state=TURN;
    case RIGHT:
    baseRightSpeed=-90;
    desiredGyro=gyro+90;
    if (desiredGyro>360){
      desiredGyro-=360;
    }
    state=TURN;
  }
}

void setup() {

  //----------------------------------
  //Only for testing purposes
  //----------------------------------
  // pinMode(29,OUTPUT);
  // pinMode(28,OUTPUT);
  // pinMode(6,OUTPUT);
  // pinMode(7, OUTPUT);
  // leftMotor.initialize();
  // rightMotor.initialize();
  // test1.initialize();
  // ---------------------------------

  state = STOP; //Robot will start being stopped
  //state = WALLFOLLOW;
  startStop = START; //Robot will move once button is pushed

  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), LeftEncoderTicks, CHANGE);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3), RightEncoderTicks, CHANGE);
  pinMode(11, INPUT_PULLUP);
  debouncer.attach(11);
  debouncer.interval(5);

  setupIMU(); //problem
  fireSensor.initialize(); //this initializes the fire sensor
  //calibrateLineSensor();
  driveTrain.initialize();
  driveStraightPID.setpid(7,.1,.02);
  turnPID.setpid(1,.2,.02);

  fanInitialize();
  lcd.begin(16, 2);
  Serial.begin(9600);
}


void loop() {
  //----------------------------------
  //Only for testing purposes
  //----------------------------------
  //In order for testing driving directions
  //test1.motorDrive(TURNLEFTCENTER);
  //------------or---------------
  // digitalWrite(29, LOW);
  // analogWrite(7,255);
  // leftMotor.setPower(255);
  // rightMotor.setPower(255);
  //-----------or----------------
  // driveTrain.setPower(0,0);
  //-----------------------------
  //Fire Sensor hey tye something
  //fireSensor.useSensor();
  //fireSensor.showAll();
  //-----------------------------


  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyro = euler.x();

  fireSensor.useSensor();
   lcd.setCursor(0,0);
   lcd.print(frontUltraVal);
  // lcd.print(fireSensor.getx());
   lcd.setCursor(0,1);
   lcd.print(backUltraVal);
  // lcd.print(fireSensor.getz());


  debouncer.update();
  if(debouncer.risingEdge()){
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

  //---------------------
  Serial.println(state);
  //----------Caleb code--------
  switch(state){
    case WALLFOLLOW:
      lcd.setCursor(9, 1);
      //lcd.print("Start");
      // Serial.println("in switch case drive");
      driveFollow();
      calcXandY();
      break;
    case STOP:
      lcd.setCursor(9, 1);
      //lcd.print("STOPPED");
      driveTrain.setPower(0, 0);
      break;

    case TURN:
      lcd.setCursor(9, 1);
      //lcd.print("turning")

      proportionalVal = driveStraightPID.calc(frontUltraVal, backUltraVal);
      newLeftSpeed = baseLeftSpeed + proportionalVal;
      newRightSpeed = baseRightSpeed - proportionalVal;
      driveTrain.setPower(newLeftSpeed, newRightSpeed);
      if(abs(gyro-desiredGyro)<=4){//tweak or put gyro in
        state=WALLFOLLOW;
      }
      break;

    case FLAME:
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("Flame is in front");
      break;
  }
}


//This function is the state level control for driving
void driveFollow(){
  // lcd.clear();
  // lcd.print("inDrive");
  // lcd.setCursor(0,1);
  // lcd.print(gyro);
  // Serial.println(frontUltra.readDistance());
  // Serial.println("in driveFollow()");

  lcd.setCursor(7,0);
  lcd.print(frontUltra.avg());

  //prevent false read
  if (frontUltra.avg()<= 15  ){ //the trig and echo pin need to be set to correct values
    driveTrain.setPower(0, 0);

    lcd.print("front sensed");
    calcXandY();
    char message[] = "Distance travelled";
    printLCD(x,y,message);

    turn(RIGHT);   //Inside this function, state is changed to turn
  }
  // if (qtraSix.readLine(sensors) > 100){ //have this if statement be if line follower is triggered
  //   driveTrain.setPower(0, 0);
  //   calcXandY();
  //   char message[] = "Left/Right Motor Speeds";
  //   printLCD(x,y,message);
  //   //change to new switch case here, will need to turn now
  // }
  if(fireSensor.isFire()){ //have this if statement be if flame sensor is triggered
    if(sideUltra.readDistance() < 10){
      state = FLAME;
    }
  }
  followWall();
}


//This function has the robot follow a wall using the PID
void followWall(){
  // Serial.println("in followWall()");
  //ping in succession
  baseRightSpeed =baseRightSpeed_120;
  baseLeftSpeed = baseLeftSpeed_120;
  int count=millis()%2;
  if(count == 1){
    frontUltraVal = frontLeftUltra.avg();
  }
  else if(count==0){
    backUltraVal = backLeftUltra.avg();
  }
  proportionalVal = driveStraightPID.calc(frontUltraVal, backUltraVal);
  newLeftSpeed = baseLeftSpeed + proportionalVal;
  newRightSpeed = baseRightSpeed - proportionalVal;
  driveTrain.setPower(newLeftSpeed, newRightSpeed);

  // leftDrive.setPower(50);
  // Serial.print("Left: ");
  // Serial.println(newLeftSpeed);
  // Serial.print("Right: ");
  // Serial.println(newRightSpeed);
  // if (frontLeftUltra.avg()>=35){
  //   rightEncTicks=0;
  //   state=TURNLEFT;
  // }
//  char message[] = "X/Y val";
  //printLCD(x, y, message);
}


void calcXandY(){
  double temp= (leftEncTicks + rightEncTicks)/2;
  x = x + (temp *.0072*2);// * cos(gyro);   //TODO BOTH OF THESE VALUES NEED TO BE CHANGED
  y = y + (temp *.0072*2);// * sin(gyro);   // 2.75 in diam 5.5pi circumfrence, 17.28in/revesr
  leftEncTicks=0;
  rightEncTicks=0;
  //800 counts per rev for rising edge single channel 1/800 *17.28 * 1/3
}


// void startOrStop(){
//   noInterrupts();
//   delayMicroseconds(20);
//   interrupts();
//   switch(startStop){
//     case STOPROBOT:
//       state = STOP;
//       startStop = START;
//       break;
//
//     case START:
//       state = WALLFOLLOW;
//       startStop = STOPROBOT;
//       break;
//
//   }
// }


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
  if(!bno.begin()){
    lcd.print("error");
  }
  // 1.) Set the accelerometer range
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);
  bno.setExtCrystalUse(true);
  // 3.) Setup the gyroscope
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}
