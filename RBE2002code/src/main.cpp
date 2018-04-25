#include <Arduino.h>
#include "FireSensor.h"
#include "drive.h"
#include "Ultrasonic.h"
#include "globalPins.h"
#include "PID.h"
#include <LiquidCrystal.h>
#include "QTRSensors.h"
#include "EEPROMex.h"
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Adafruit_BNO055.h>
#include "States.h"
#include <Bounce2.h>
#include "Fan.h"
#include "MotorStates.h"
#include "line.h"
#include "IRrangeFinder.h"
//RYAN

//////////////
//CONSTANTS //
//////////////
#define baseLeftSpeed_120 200
#define baseRightSpeed_120 200
#define OFFSET_HEIGHT 6 //TODO: This is the number of inches the flame sensor is off the ground
#define CENTERVAL_X 511.5 //TODO: Position of flame sensor so at center of x range
/////////////////////
//GLOBAL VARIABLES //
/////////////////////
bool blowing = false; //For Flame state in order to know if the candle is out or not
bool goToFlame = false;
int turnLeft = 1;
bool cliff = false;
bool islandTurnBool = false;
bool firstTurn = false;

//////////////////////////
//State diagram control //
//////////////////////////
enum State {STOP, WALLFOLLOW,TURN, FLAME, TRAVELTOFLAME, TURNRIGHTLINE,DRIVESTRAIGHT} state;
enum State2 {STOPROBOT, START} startStop;
enum pidSelect {WALL,TURNING} pidSel;
enum turner {LEFT,RIGHT} turnDir;
float x = 0;
float y = 0;
float z = 0;
float saveX, saveY, saveZ;
unsigned long leftEncTicks = 0;
unsigned long rightEncTicks = 0;
int stop;
int desiredGyro;
int previousDistance=0;
int baseRightSpeed =baseRightSpeed_120;
int baseLeftSpeed = baseLeftSpeed_120;
unsigned int sensors[3];
int frontUltraVal=0;
int backUltraVal=0;
float pValLeft,pValRight,proportionalVal;
int newLeftSpeed;
int newRightSpeed;
float gyro;
bool returnHome;
float distTraveled;
float finalDistance;
MotorStates testMotor;
int turns=0;
int distToCandle= 0;
float heightOfRobot = 9;
int flameDistance = 8;

////////////////////////
//Function prototypes //
////////////////////////
void driveFollow();
void followWall();
void printLCD(int, int, char[]);
void calcXandY();
void LeftEncoderTicks();
void RightEncoderTicks();
void startOrStop();
void calibrateLineSensor();
void setupIMU();
int turnInitialize(int);
void drivePID();
void gyroVal();
void calculateHeight();
void displayXYZ();
void saveValues();
bool centerFlameX();
void driveToFlame();
bool driveStraight(float);
double returnDistance();
void islandTurn(int);
bool isSensorCliff();
bool wallFound = false;
double gyroLookUp(double);
void turn(int);
void calculateCandleOffset();
void stopMoving();
void lineBack();
void driveFlameSeen(int);
void calcZ();
////////////////////
//Object Creation //
////////////////////
FireSensor fireSensor;
drive driveTrain;
Ultrasonic backLeftUltra(BACKLEFTULTRATRIG, BACKLEFTULTRAECHO);
Ultrasonic frontLeftUltra(FRONTLEFTULTRATRIG, FRONTLEFTULTRAECHO);
Ultrasonic frontUltra(FRONTULTRATRIG, FRONTULTRAECHO);
Ultrasonic sideUltra(SIDEULTRATRIG, SIDEULTRAECHO);
PID driveStraightPID;
PID turnPID;
PID centerFlameXPID;
LiquidCrystal lcd(40, 41, 42, 43, 44, 45);
QTRSensorsAnalog qtraSix((unsigned char[]) {0, 1, 2, 3, 4, 5}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
Bounce debouncer = Bounce();
Adafruit_BNO055 bno = Adafruit_BNO055();
extern Servo fanServo;
PID encoderPID;
PID gyroPID;
extern Fan fan;

GP2Y0A02YK0F irSensor;

Line lineOne(LINEFOLLOWERONE,10);
Line lineTwo(LINEFOLLOWERTWO,10);

//////////////////////
//Arduino Functions //
//////////////////////
void setup() {
  fan.initialize();

  state = STOP; //Robot will start being stopped
  startStop = START; //Robot will move once button is pushed

  returnHome = false; //Not currently returning returning home

  //Interrupts
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3), LeftEncoderTicks, CHANGE);
  pinMode(18, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(18), RightEncoderTicks, CHANGE);
  pinMode(11, INPUT_PULLUP);

  //Debouncer
  debouncer.attach(11);
  debouncer.interval(5);

  //Initialize all classes
  setupIMU();
  fireSensor.initialize();
  //calibrateLineSensor(); //TODO: Record values into memory

  driveTrain.initialize();

  //PIDs
  driveStraightPID.setpid(15, 0,.05); //PID to drive straight  //was 30
  //turnPID.setpid(100,.3,.01); //PID for turning //was 13
  centerFlameXPID.setpid(.05, 0, 0); //PID for centering flame
  encoderPID.setpid(.1, .1, 0.01);
  gyroPID.setpid(4.5, 0, 0.01);

  irSensor.begin(IRPIN);
  //Displays
  lcd.begin(16, 2);
  Serial.begin(115200);

  //TESTING
  testMotor.initialize();

  bno = Adafruit_BNO055();

}


void loop() {

  // testMotor.motorDrive(STRAIGHT);
//  Serial.println(frontUltra.avgTwo());
// int val = irSensor.getDistanceCentimeter();
//   Serial.println(val);
  // Serial.print("R: ");
  // Serial.println(rightEncTicks);
  //fan.maxPower(true);
  // delay(30000);
  // fan.maxPower(false);
  // delay(30000);
// islandTurn(5);
// //turnInitialize(LEFT);
// driveTrain.setPower(0, 0);
// delay(3000);


 // driveTrain.setPower(255, 255);
 // //lcd.print(leftEncTicks);
 // int val = frontUltra.avg();
 //  Serial.println(val);
//
//
// driveStraight(10);
     imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //get vector from IMU
     gyro = euler.x(); //x value of IMU
    // lcd.setCursor(0, 1);
    // lcd.print(gyro);

  //   turnInitialize(RIGHT);
  //   lcd.setCursor(0, 0);
  //   lcd.print(x);
  //   lcd.setCursor(10, 0);
  //   lcd.print(y);
  //   // lcd.setCursor(0, 1);
  //   // lcd.print(gyro);
  //   driveStraight(10);
  //  // euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //get vector from IMU
  //  //  gyro = euler.x(); //x value of IMU
  //  //  calcXandY();
  //  turnInitialize(RIGHT);
  //   lcd.setCursor(0, 0);
  //   lcd.print(x);
  //   lcd.setCursor(10, 0);
  //   lcd.print(y);
  //   // lcd.setCursor(0, 1);
  //   // lcd.print(gyro);
  //   driveStraight(10);
  //  // euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //get vector from IMU
  //  //  gyro = euler.x(); //x value of IMU
  //  //  calcXandY();
  //  turnInitialize(RIGHT);
  //   lcd.setCursor(0, 0);
  //   lcd.print(x);
  //   lcd.setCursor(10, 0);
  //   lcd.print(y);
  //   driveStraight(10);
  //  // euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //get vector from IMU
  //  //  gyro = euler.x(); //x value of IMU
  //  //  calcXandY();
  //  turnInitialize(RIGHT);
  //   lcd.setCursor(0, 0);
  //   lcd.print(x);
  //   lcd.setCursor(10, 0);
  //   lcd.print(y);
  //   driveTrain.setPower(0, 0);
  //   delay(100000);
  //  frontUltraVal = frontLeftUltra.avg();
  // //Serial.println(gyro);




//////////////////////
// CODE STARTS HERE //
//////////////////////


  fireSensor.useSensor(); //save flame sensor values to array

  //Emergency start/stop button
   debouncer.update();
  if(debouncer.risingEdge()){
    switch(startStop){
      case STOPROBOT:
        state = STOP; //stop robot
        startStop = START; //set so next time button is pressed, robot starts
        break;

      case START:
        state = WALLFOLLOW; //start robot
        startStop = STOPROBOT; //set so next time button is pressed, robot stops
        break;
    }
  }

  //REVIEW:
  if(returnHome){
    if(x < 3 && y < 3){  // as the robot gets closer to the original starting position,
                         // x and y should get closer to 0, at this point, want to stop robot
      state = STOP;
      displayXYZ(); //print to screen coordinates of candle
    }
  }
  //Main flow control
  switch(state){
    case WALLFOLLOW:
      driveFollow();

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WALLFOLLOWING");

      break;

    case STOP:
      driveTrain.setPower(0, 0);
      //lcd.setCursor(0,0);
      //lcd.print(digitalRead(LINEFOLLOWERONE));
      //lcd.setCursor(0,1);
      //lcd.print(digitalRead(LINEFOLLOWERTWO));
      //lcd.clear();
      //lcd.setCursor(10,0);

      break;

    case TURN: //REVIEW:
      /******************************************************************************
      * IDEA                                                                       *
      * Turning should have its own PID, and it should not be based on ultrasonics *
      ******************************************************************************/
      // turn(turnLeft);
      // driveTrain.setPower(0, 0);
      // delay(3000);
      // if(goToFlame){
      //   driveStraight(400);
      // }
      // // else if(turnLeft){
      // //   islandTurn(20);
      // // }
      // else if(cliff){
      //   driveStraight(400);
      // }
      // else{
      //   state=WALLFOLLOW;
      //   wallFound=false;
      // }
      break;

    case FLAME: //REVIEW:

      lcd.clear();
      lcd.setCursor(0, 1);
      //if(!fireSensor.isFire()){
      //   break;
      // }
      lcd.print("Flame is in front");
      driveTrain.setPower(0, 0);
      // bool hVal;
      // bool xVal;

      //if(!blowing){//REVIEW: It is a global variable in main (at top). NOTE: check if this class works
        /**********************************************/
        //hVal = fireSensor.centerHeight();  //move flame sensor to be at center of flame in y/z direction
        // lcd.clear();
        // lcd.setCursor(0, 0);
        // lcd.print("Center Height");
        //delay(200);
        /**********************************************/
        //xVal = centerFlameX(); //move flame sensor to be at center of flame in x direction
        /**********************************************/
        //driveTrain.setPower(0, 0);
        //calculateCandleOffset();
        //calculateHeight(); //determine height of candle
        //saveValues(); //save x, y, and z values (will change when robot returns home)
        /**********************************************/
      // }
      // if(hVal && xVal){
        //if(fireSensor.isFire()){//REVIEW: We can also make it do this for t amount of seconds and later check if its out
        lcd.clear();
        lcd.setCursor(0, 0);
          lcd.print("Blowing");
          fireSensor.blowOutCandle(); //extinguish the candle
          blowing = true;
          lcd.clear();
          lcd.setCursor(0, 0);
          if(!fireSensor.isFire()){
          lcd.print("Blew out candle");
        //}else{
          returnHome = true; //use to have robot stop when returns to (0.0) posiion
          state = STOP; //COMBAK: Change this to WALLFOLLOW: have robot continue driving home
        //}
        //}
      }
      break;
   case TRAVELTOFLAME:
      //lcd.clear();
      //lcd.setCursor(0, 0);
      //lcd.print("TRAVELTOFLAME");
      firstTurn = false;
      distToCandle = sideUltra.avg();
      //lcd.setCursor(0, 1);
      //lcd.print(distToCandle);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Center height");
      driveTrain.setPower(0, 0);
      delay(2000);
      calcZ();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(z);
      delay(2000);
      fireSensor.centerHeight();
      centerFlameX();
      driveTrain.setPower(0, 0);
      delay(2000);

    //   // if(fireSensor.isFire()){
    //   //   driveTrain.setPower(0,0);
    //   //   state = FLAME;
    //   // }
       goToFlame = true;  //will cause robot to go to TRAVELTOFLAME switch case
       turnInitialize(RIGHT);
       //driveFlameSeen();
       //firstTurn = true;
       driveFlameSeen(1000);
       //firstTurn = false;
       goToFlame = true;
       turnInitialize(LEFT);

       if(fireSensor.isFire()){
         state = FLAME;
       }
       driveTrain.setPower(0, 0);
       delay(3000);
       break;

     case TURNRIGHTLINE:
       //TODO:
        // driveTrain.setPower(-100, -100);
        // delay(1000);
        // driveTrain.setPower(0, 0);
        // cliff = true;
        // turnInitialize(RIGHT);
        lineBack();
        //driveStraight(100);
        break;
    case DRIVESTRAIGHT:


    //delay(500);
      //lcd.clear();
      //lcd.setCursor(0,0);
     //lcd.print("DRIVESTRAIGHT");

      //}
       break;

}
}


//////////////
//Functions //
//////////////

//REVIEW:
/**
 * Initializes values for turning
 * @param turnDir 1 for right, 0 for left
 */
int turnInitialize(int turnDir){
  //lcd.clear();
  baseLeftSpeed=0;
  baseRightSpeed=0;
  //calcXandY();
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //get vector from IMU
  gyro = euler.x(); //x value of IMU
  calcXandY();
  switch(turnDir){
    case LEFT:
      desiredGyro= ((int) euler.x() +270) % 360;

      turnLeft = true;
      //state=TURN;
      //turn(turnLeft);
      break;
    case RIGHT:

      desiredGyro= ((int) euler.x() + 90) % 360;
      // if(desiredGyro < 0){
      //   desiredGyro += 360;
      //
      // }
      turnLeft=false;
      // if(desiredGyro > 360){
      //   desiredGyro =((int)gyro+90)-360;
      // }
      //state=TURN;
      //turn(turnLeft);
      break;
  }
  turn(turnLeft);
  return turnDir;
}


/**
 * Driving control
 */
void driveFollow(){

  // int count=millis()%2;
  // if(count == 1){
  //   frontUltraVal = frontLeftUltra.avg();
  // }
  // else if(count==0){
  //   backUltraVal = backLeftUltra.avg();
  // }
  //If front ultrasonic triggered (wall in front)
  //lcd.clear();

  // if(frontUltra.avg() < 15){
  //   driveTrain.setPower(0, 0); //Stop robot
  //   //lcd.clear();          //COMBAK: Remove this, for testing
  //   //lcd.setCursor(5, 1);
  //   //lcd.print("FRONT ULTRA TRIGGERED");
  //   delay(2000);
  //   //delay(400);
  //   frontUltra.clear();
  //   calcXandY(); //Calculate x and y
  //
  //
  //   turnInitialize(RIGHT);   //REVIEW: This should be moved to TURN because line followers uses it as well
  // }

  //TODO: Test out line sensor code/values
  //If line sensor triggered
  // if (qtraSix.readLine(sensors) > 100){ //have this if statement be if line follower is triggered
  //   driveTrain.setPower(0, 0);
  //   calcXandY();
  //   char message[] = "Left/Right Motor Speeds";
  //   printLCD(x,y,message);
  //   //change to new switch case here, will need to turn now
  // }

  //If flame sensor senses
  //fire
  if(fireSensor.isFire()){
    //if(sideUltra.avg() < 10){ //TODO: Test value to see distance for flame
    //state = FLAME;
    //centerFlameX();  //center flame in x direction
    //driveToFlame();
    //lcd.setCursor(5,1);
    //lcd.print("flame");
    //delay(300);
    state = TRAVELTOFLAME;
    //calcXandY();
    //return;
    //}
  }
   else{
    followWall();
   }
}


/**
 * Robot follows wall using PID between two ultrasonics
 * NOTE: Does not include set distance from wall
 */
void followWall(){
  ////lcd.clear();
  //set base speeds

  baseRightSpeed =baseLeftSpeed_120;
  baseLeftSpeed = baseRightSpeed_120;

  //ping ultrasonics in succession
  if(irSensor.avg() < 30){
    // int count1=0,count2=0;
    // unsigned int time=millis();
    // while(time+100>millis()){
    //   if(irSensor.getDistanceCentimeter() < 20){
    //     count1++;
    //   }
    //   count2++;
    // }
    // if(count1==count2){
    driveTrain.setPower(0, 0); //Stop robot
    lcd.clear();          //COMBAK: Remove this, for testing
    lcd.setCursor(5, 1);
    lcd.print("FRONT ULTRA TRIGGERED");
    delay(2000);
    //delay(400);
    frontUltra.clear();
    //calcXandY(); //Calculate x and y


    turnInitialize(RIGHT);
  // }   //REVIEW: This should be moved to TURN because line followers uses it as well
  }
  frontUltraVal = frontLeftUltra.avg();

  // else if(count==0){
  //   backUltraVal = backLeftUltra.avg();
  // }

    /////////////
    // TESTING //
    /////////////
    /////////////
  // Serial.print("Left: ");
  // Serial.println(frontUltraVal);
  // Serial.print("Right: ");
  // Serial.println(backUltraVal);

  if(frontUltraVal > 35){
    //lcd.clear();          //COMBAK: Remove this, for testing
    //lcd.setCursor(0, 1);
    //lcd.print("NO Wall");
     turnLeft = true;
     //driveStraight(10);
     //calcXandY();
     islandTurn(10);

    //if(reachedDistance){
    //turnLeft = turnInitialize(LEFT);
      //state = STOP;
      // //lcd.clear();
      // //lcd.setCursor(0, 1);
      // //lcd.print("Stopped");
  //  turnInitialize(LEFT);
    // }else{
    //   return;
    // }
  }


  if(isSensorCliff()){
    lcd.setCursor(5,1);
    lcd.print("line");
    lineBack();
    // lcd.setCursor(5,1);
    // lcd.print("line");
    //delay(400);
    //islandTurn(10);
  }

  //PID control

  pValRight = driveStraightPID.calc(10,frontLeftUltra.avg());
  //pValLeft = driveStraightPID.calc(15, backUltraVal);
  //Serial.println(frontUltraVal);
  //S
  newLeftSpeed = 105;
  newRightSpeed = 115 - pValRight;

  if (newLeftSpeed>255){
   newLeftSpeed=255;
  }
 if (newRightSpeed>255){
   newRightSpeed=255;
  }


  driveTrain.setPower(newLeftSpeed, newRightSpeed);
  //lcd.setCursor(0, 0);
  //lcd.print(frontLeftUltra.avg());
  //lcd.setCursor(0, 10);

  //lcd.setCursor(0,1);
  //lcd.print(newLeftSpeed);
  //lcd.setCursor(10,1);
  //lcd.print(newRightSpeed);

}


/**
 * Calculates the x and y position of the robot
 */
/****************************************************
 * Math:                                            *
 * 2.75 in diam 5.5pi circumfrence, 17.28in/revesre *
 * 800 counts per rev for rising edge single channel*
 *          1/800 *17.28 * 1/3                      *
 ****************************************************/
void calcXandY(){
  double temp= (leftEncTicks + rightEncTicks)/2;
  x = x + ((temp *.0072*2)/ 8) * cos(gyro);   //REVIEW: Add back in gyro code
  y = y + ((temp *.0072*2)/ 8) * sin(gyro);
  leftEncTicks=0;
  rightEncTicks=0;
  //TODO: For final distance, must use ultrasonic to get distance from robot to candle
}

/**
 * Calculate height of flame
 */
void calculateHeight(){
  int theta = fanServo.read();  //determine angle of servo, may need to offset this value depending on how servo is mounted
  int distanceToFlame = sideUltra.readDistance(); //use ultrasonic to get distance to flame
  z = tan(theta)*distanceToFlame + OFFSET_HEIGHT; //TODO: Set OFFSET_HEIGHT to be height of flame sensor off the ground
}


/**
 * ISR for Button
 */
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


/**
 * Print two values and message to LCD
 * @param valOne  First value to be printed
 * @param valTwo  Second value to be printed
 * @param message Message to be printed
 */
void printLCD(int valOne, int valTwo, char message[]){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(valOne);
  lcd.setCursor(10, 0);
  lcd.print(valTwo);
  lcd.setCursor(0,1);
  lcd.print(message);
}


/**
 * Display x, y, and z values on LCD
 */
void displayXYZ(){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("X: ");
  lcd.print(saveX);
  lcd.print("  ");
  lcd.print("Y: ");
  lcd.print(saveY);
  lcd.setCursor(0, 1);
  lcd.print("Z: ");
  lcd.print(saveZ);
}


/**
 * Record x, y, and z values
 */
void saveValues(){
  saveX = x;
  saveY = y;
  saveZ = z;
}


/**
 * ISR for left encoder ticks
 */
void LeftEncoderTicks() {
  leftEncTicks++;
  //Serial.println(leftEncTicks);
}

/**
 * ISR for right encoder ticks
 */
void RightEncoderTicks() {
  rightEncTicks++;
  //Serial.println(rightEncTicks);
}


/**
 * Calibrate line sensor by reading in values from EEPROM memory
 */
void calibrateLineSensor() {
  // Serial.println("Calibrating....");
  // delay(500);
  //
  // qtraSix.emittersOn();
  // pinMode(13, OUTPUT);
  // digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  //
  // qtraSix.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  // EEPROM.readBlock<unsigned int>(addrCalibratedMinimumOn, qtraSix.calibratedMinimumOn, 8);
  // EEPROM.readBlock<unsigned int>(addrCalibratedMaximumOn, qtraSix.calibratedMaximumOn, 8);
  //
  // Serial.println("EEPROM Recall Complete");
  //
  // digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration
  // delay(1000);
  pinMode(LINEFOLLOWERONE, INPUT);
  pinMode(LINEFOLLOWERTWO, INPUT);
}

/**
 * Initialize IMU
 */
void setupIMU()
{
  if(!bno.begin()){
    //lcd.print("error");
  }
  bno.setExtCrystalUse(true);
}


/**
 * Center flame in x direction
 */
bool centerFlameX(){
  baseLeftSpeed = 95;
  baseRightSpeed = 105;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("center flame x");
  // fireSensor.useSensor();
  // int centerXError = centerFlameXPID.calc(CENTERVAL_X, fireSensor.getx()); //PID based on flame x value and centered x value
  // int newSpeed = 90 + centerXError;
  // int encoderError = encoderPID.calc(rightEncTicks, leftEncTicks);
  // driveTrain.setPower(newSpeed+encoderError, newSpeed-encoderError); //dont want wheels to turn, so make sure both are going in same direction
  // //lcd.clear();
  // //lcd.setCursor(0, 0);
  // //lcd.print(newSpeed);
  // //lcd.setCursor(0, 1);
  // //lcd.print(fireSensor.getx());
  //  if(abs(CENTERVAL_X-fireSensor.getx()) <=20){
     driveTrain.setPower(0, 0);
     delay(1000);
  //    return true;
  //  }
  float encoderError = centerFlameXPID.calc(leftEncTicks, rightEncTicks);
  while(abs(fireSensor.getx() - CENTERVAL_X) > 5){
    fireSensor.useSensor();
   if(fireSensor.getx() < CENTERVAL_X){
      newLeftSpeed = baseLeftSpeed + encoderError;
      newRightSpeed = baseRightSpeed - encoderError;
      if (newLeftSpeed > 255){
        newLeftSpeed = 255;
      }
      if (newRightSpeed > 255){
        newRightSpeed=255;
      }
      driveTrain.setPower(newLeftSpeed, newRightSpeed);
   }
    else{
      newLeftSpeed = (baseLeftSpeed + encoderError) * -1;
      newRightSpeed = (baseRightSpeed - encoderError) * -1;

    }
    if (newLeftSpeed > 255){
      newLeftSpeed = 255;
    }
    if (newRightSpeed > 255){
      newRightSpeed=255;
    }
    driveTrain.setPower(newLeftSpeed, newRightSpeed);
    encoderError = centerFlameXPID.calc(leftEncTicks, rightEncTicks);
    lcd.setCursor(0, 1);
    lcd.print(fireSensor.getx());
    lcd.setCursor(10, 1);
    lcd.print(abs(fireSensor.getx() - CENTERVAL_X));
  }
  return true;
   // else{
   //  return false;
   // }
}


/**
 * Drive past flame by width of robot, then initialize turn
 * NOTE: This will go to TRAVELTOFLAME
 */
 void driveToFlame(){
   goToFlame = true;  //will cause robot to go to TRAVELTOFLAME switch case
   //turnInitialize(RIGHT);
   driveStraight(5);
 }


/**
 * Drive straight using complimentary filter of gyro and encoders
 */
bool driveStraight(float distToGo){
  //NOTE: These values must add to 1
  leftEncTicks = 0;
  rightEncTicks = 0;
  if (irSensor.avg() < 30){
    driveTrain.setPower(0, 0);
    delay(1000);
    if(cliff){
      turnInitialize(RIGHT);
      cliff = false;
    }
    return true;
  }

  // gyro=euler.x();


  //COMBAK: Uncomment everything below
  // baseLeftSpeed=baseLeftSpeed_120;
  // baseRightSpeed=baseRightSpeed_120;
  //
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("in Drive straight");
  frontUltra.clear();
  baseLeftSpeed = 190;
  baseRightSpeed = 210;
  distTraveled = returnDistance();
  finalDistance = distTraveled + distToGo;
   //state=DRIVESTRAIGHT;
//lcd.clear();
   while(distTraveled < finalDistance){

     //islandTurn = true;
     if(fireSensor.isFire()){ //&& firstTurn){
       driveTrain.setPower(0,0);
       state = FLAME;
       //return;
     }

     else if ((irSensor.avg() < 30) || (frontLeftUltra.avg() < 30)){
       driveTrain.setPower(0, 0);
       delay(1000);
       if(cliff){
         turnInitialize(RIGHT);
         cliff = false;
       }
       state=WALLFOLLOW;
       return true;
     }

     else if(isSensorCliff()){
       lineBack();
       break;
     }


    // else if(frontUltra.avg() < 15 && !goToFlame){
    //    goToFlame = false; //not in flame
    //    turnLeft = false;
    //    cliff = false;
    //    turnInitialize(RIGHT);
    //  }
    //  else if(frontLeftUltra.avg() < 20 && !goToFlame&&cliff){
    //    state = WALLFOLLOW;
    //    //driveFollow();
    //    break;
    //
    //  }
     //Encoder PID
     float gyroPercentage = 0;
     float encoderPercentage = .2;
     float encoderError = encoderPID.calc(leftEncTicks, rightEncTicks);
     //lcd.setCursor(0,0);
     //lcd.print(finalDistance-distTraveled);

     //Gyro PID
     imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
     int gyroError = gyroPID.calc(gyroLookUp(gyro), euler.x());
     //Complimentary Filter
     int driveCompFilter = (gyroPercentage * gyroError) + (encoderPercentage * encoderError);
     newLeftSpeed = baseLeftSpeed - driveCompFilter;
     newRightSpeed = baseRightSpeed + driveCompFilter;
     if (newLeftSpeed > 255){
       newLeftSpeed = 255;
     }
     if (newRightSpeed > 255){
       newRightSpeed=255;
     }
     // }if (newLeftSpeed < -255){
     //   newLeftSpeed = -255;
     // }
     // if (newRightSpeed < -255){
     //   newRightSpeed= -255;
     // }
     driveTrain.setPower(newLeftSpeed, newRightSpeed);
     delay(10);
     //lcd.setCursor(0,1);
     //lcd.print(newLeftSpeed);
     //lcd.setCursor(10,1);
     //lcd.print(newRightSpeed);
     distTraveled = returnDistance();
   }
  // if(goToFlame){
  //      turnInitialize(RIGHT);
  //    }
  irSensor.clear();
    if(cliff){
      return true;
    }
    if(islandTurnBool){
       //lcd.clear();
       //lcd.setCursor(0, 0);
       //lcd.print("In else if in DS");
       //delay(1000);
       return false;
     }

     // else if (turns<2){
     //   turns++;
       //turnInitialize(LEFT);//TODO

     //}
     else{
        turnLeft=false;
        turns=0;
        state=WALLFOLLOW;
      }
    return false;
     //delay(100);

  //COMBAK: New stuff added

  // driveTrain.setPower(baseLeftSpeed, baseRightSpeed);
  // Serial.println(gyroError);
  //
  //
  // float gyroPercentage = .1;
  // float encoderPercentage = .9;
  // float encoderError = encoderPID.calc(leftEncTicks, rightEncTicks);
  // //lcd.setCursor(0,0);
  // //lcd.print(leftEncTicks);
  //
  // //Gyro PID
  // imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  // int gyroError = gyroPID.calc(gyroLookUp(gyro), euler.x());
  // //Complimentary Filter
  // int driveCompFilter = (gyroPercentage * gyroError) + (encoderPercentage * encoderError);
  // newLeftSpeed = baseLeftSpeed - driveCompFilter;
  // newRightSpeed = baseRightSpeed + driveCompFilter;
  // driveTrain.setPower(newLeftSpeed, newRightSpeed);
}

double returnDistance(){
  double temp= (leftEncTicks + rightEncTicks)/2;
  double distance = distance + ((temp *.0072*2)) / 8;
  Serial.println(distance);
  return distance;
  //TODO: For final distance, must use ultrasonic to get distance from robot to candle
}

void islandTurn(int distance){
  bool resumeWallFollowing = false;
  //turnLeft = true;
  islandTurnBool = true;
  resumeWallFollowing = driveStraight(distance);
  stopMoving();
  delay(500);

  if(resumeWallFollowing){
    return;
  }
  turnInitialize(LEFT);
  stopMoving();
  delay(500);

  // driveStraight(25);
  // stopMoving();
  // delay(500);
  //
  // turnInitialize(LEFT);
  // stopMoving();
  // delay(500);
  //
  // driveStraight(distance);
  // stopMoving();
  // delay(500);
  state = WALLFOLLOW;
  turnLeft=false;
  islandTurnBool = false;




  //turnInitialize(LEFT);
  //turn(LEFT);
  //turnLeft = false;
  //driveStraight(20);
  // turnLeft = true;
  // turnInitialize(LEFT);
  // driveStraight(distance);
  // turnInitialize(LEFT);
  // driveStraight(20);
  // turnLeft = false;
}

bool isSensorCliff(){

  return ((analogRead(LINEFOLLOWERONE) > 900) && (analogRead(LINEFOLLOWERTWO > 900))) ? 1 : 0;

  //return lineOne.limit() && lineTwo.limit();


}

double gyroLookUp(double gyroVal){
  if(gyroVal >=295 && gyroVal < 45){
    gyroVal = 0;
  }
  if(gyroVal >= 45 && gyroVal < 125){
    gyroVal = 90;
  }
  if(gyroVal >= 125 && gyroVal < 225){
    gyroVal = 180;
  }
  if(gyroVal >= 225 && gyroVal < 295){
    gyroVal = 270;
  }
  return gyroVal;
}

void turn(int turnLeft){
  //lcd.setCursor(0, 0);
  //displayXYZ();
  lcd.setCursor(0, 0);
  lcd.print(x);
  lcd.setCursor(10, 0);
  lcd.print(y);


  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float currentAngle =  euler.x();
  //desiredGyro *= (0.5 * cliff);

  while(abs(desiredGyro - currentAngle)>=13){ //was 3
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    currentAngle =  euler.x();

    irSensor.avg();
    // if(isSensorCliff()){
    //   state = TURNRIGHTLINE;
    //   //lcd.setCursor(5,1);
    //   //lcd.print("line");
    //   //delay(400);
    //   cliff = true;
    // }
    //frontUltraVal = frontLeftUltra.avg();
    //proportionalVal = turnPID.calc(currentAngle, desiredGyro);
    proportionalVal = 0;
    // newLeftSpeed = 0 - proportionalVal;
    // newRightSpeed = 0 + proportionalVal;
    //
  if(!turnLeft){
      newLeftSpeed = 115 + (-1 * proportionalVal);
      newRightSpeed = -125 + proportionalVal;
    }
    else{
      newLeftSpeed = -115 + proportionalVal;
      newRightSpeed = 125 + (-1* proportionalVal);
    }

    driveTrain.setPower(newLeftSpeed, newRightSpeed);
    frontUltra.clear();
    //lcd.setCursor(0, 0);
    //lcd.print("TURNING");
    //lcd.setCursor(0, 1);
    //lcd.print(currentAngle);
    //lcd.setCursor(10, 1);
    //lcd.print(desiredGyro);
    //lcd.setCursor(10, 0);
    //lcd.print(abs(desiredGyro - currentAngle));


  }

  // driveTrain.setPower(0, 0); //COMBAK: Remove these lines
  // delay(5000);
  //lcd.clear();
  //lcd.setCursor(0, 0);
  //lcd.print("Finished turning");

  driveTrain.setPower(0, 0);
  irSensor.avg();
  leftEncTicks = 0;
  rightEncTicks = 0;
  delay(1000);
  if(goToFlame || islandTurnBool){
    //driveStraight(400);
    //lcd.clear();
    //lcd.setCursor(0, 0);
    //lcd.print("in if in turn");
    return;
  }

  // else if(turnLeft){
  //   islandTurn(20);
  // }

  else{
    state=WALLFOLLOW;
    wallFound=false;
  }
}

void calculateCandleOffset(){
  int offset = sideUltra.avg();
  x = x + offset*cos(gyro);
  y = y + offset*sin(gyro);
}

void stopMoving(){
  driveTrain.setPower(0, 0);
}

void lineBack(){
  //lcd.print("in cliff");
  driveTrain.setPower(0, 0);
  delay(500);
  driveTrain.setPower(-175, -210);
  delay(2000);
  driveTrain.setPower(0, 0);
  delay(500);
  cliff = true;
  turnInitialize(RIGHT);
  driveStraight(40);
}


void driveFlameSeen(int distToGo){
  //NOTE: These values must add to 1
  leftEncTicks = 0;
  rightEncTicks = 0;
  if (irSensor.avg() < 30){
    driveTrain.setPower(0, 0);
    delay(3000);
    return;
  }
  // gyro=euler.x();


  //COMBAK: Uncomment everything below
  // baseLeftSpeed=baseLeftSpeed_120;
  // baseRightSpeed=baseRightSpeed_120;
  //
  // //lcd.setCursor(0,0);
  // //lcd.print("in Drive straight");
  baseLeftSpeed = 200;
  baseRightSpeed = 200;
  distTraveled = returnDistance();
  finalDistance = distTraveled + distToGo;
   //state=DRIVESTRAIGHT;
//lcd.clear();
   while((distTraveled < finalDistance) && (irSensor.avg() > 20)){

     //islandTurn = true;
     if(fireSensor.isFire()){ //&& firstTurn){
       driveTrain.setPower(0,0);
       state = FLAME;
       //return;
     }
     else if(isSensorCliff()){
       lineBack();
       break;
     }


    // else if(frontUltra.avg() < 15 && !goToFlame){
    //    goToFlame = false; //not in flame
    //    turnLeft = false;
    //    cliff = false;
    //    turnInitialize(RIGHT);
    //  }
    //  else if(frontLeftUltra.avg() < 20 && !goToFlame&&cliff){
    //    state = WALLFOLLOW;
    //    //driveFollow();
    //    break;
    //
    //  }
     //Encoder PID
     float gyroPercentage = 0;
     float encoderPercentage = .2;
     float encoderError = encoderPID.calc(leftEncTicks, rightEncTicks);
     //lcd.setCursor(0,0);
     //lcd.print(finalDistance-distTraveled);

     //Gyro PID
     imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
     int gyroError = gyroPID.calc(gyroLookUp(gyro), euler.x());
     //Complimentary Filter
     int driveCompFilter = (gyroPercentage * gyroError) + (encoderPercentage * encoderError);
     newLeftSpeed = baseLeftSpeed - driveCompFilter;
     newRightSpeed = baseRightSpeed + driveCompFilter;
     if (newLeftSpeed > 255){
       newLeftSpeed = 255;
     }
     if (newRightSpeed > 255){
       newRightSpeed=255;
     }
     // }if (newLeftSpeed < -255){
     //   newLeftSpeed = -255;
     // }
     // if (newRightSpeed < -255){
     //   newRightSpeed= -255;
     // }
     driveTrain.setPower(newLeftSpeed, newRightSpeed);
     delay(10);
     //lcd.setCursor(0,1);
     //lcd.print(newLeftSpeed);
     //lcd.setCursor(10,1);
     //lcd.print(newRightSpeed);
     distTraveled = returnDistance();
   }
  // if(goToFlame){
  //      turnInitialize(RIGHT);
  //    }
  irSensor.clear();
    if(islandTurnBool){
       //lcd.clear();
       //lcd.setCursor(0, 0);
       //lcd.print("In else if in DS");
       //delay(1000);
       return;
     }

     // else if (turns<2){
     //   turns++;
       //turnInitialize(LEFT);//TODO

     //}
     else{
        turnLeft=false;
        turns=0;
        state=WALLFOLLOW;
      }

     //delay(100);

  //COMBAK: New stuff added

  // driveTrain.setPower(baseLeftSpeed, baseRightSpeed);
  // Serial.println(gyroError);
  //
  //
  // float gyroPercentage = .1;
  // float encoderPercentage = .9;
  // float encoderError = encoderPID.calc(leftEncTicks, rightEncTicks);
  // //lcd.setCursor(0,0);
  // //lcd.print(leftEncTicks);
  //
  // //Gyro PID
  // imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  // int gyroError = gyroPID.calc(gyroLookUp(gyro), euler.x());
  // //Complimentary Filter
  // int driveCompFilter = (gyroPercentage * gyroError) + (encoderPercentage * encoderError);
  // newLeftSpeed = baseLeftSpeed - driveCompFilter;
  // newRightSpeed = baseRightSpeed + driveCompFilter;
  // driveTrain.setPower(newLeftSpeed, newRightSpeed);

}


void calcZ(){
  float angle;
  flameDistance = (sideUltra.avg())/2.54;
  lcd.print(flameDistance);
  delay(2000);
  if(fireSensor.getz() > 511){
    angle = ((0.575959/1022)*fireSensor.getz())-0.2879793;

  }
  else{
    angle = 0.2879793 - ((0.575959/1022)*fireSensor.getz());

  }
  z = (heightOfRobot - tan(angle)*flameDistance) * 1.37; //flameDistance is set to 8
}
