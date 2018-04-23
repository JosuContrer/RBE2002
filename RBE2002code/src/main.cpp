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

//////////////////////////
//State diagram control //
//////////////////////////
enum State {STOP, WALLFOLLOW,TURN, FLAME, TRAVELTOFLAME, TURNRIGHTLINE,DRIVESTRAIGHT} state;
enum State2 {STOPROBOT, START} startStop;
enum pidSelect {WALL,TURNING} pidSel;
enum turner {LEFT,RIGHT} turnDir;
float x;
float y;
float z;
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

//////////////////////
//Arduino Functions //
//////////////////////
void setup() {
  fan.initialize();

  state = WALLFOLLOW; //Robot will start being stopped
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
  driveStraightPID.setpid(50, 0,.05); //PID to drive straight  //was 30
  turnPID.setpid(100,.3,.01); //PID for turning //was 13
  centerFlameXPID.setpid(.8, .1, .01); //PID for centering flame
  encoderPID.setpid(.1, 0, 0);
  gyroPID.setpid(4.5, 0, 0.01);

  //Displays
  lcd.begin(16, 2);
  Serial.begin(115200);

  //TESTING
  testMotor.initialize();



}


void loop() {
  //testMotor.motorDrive(STOP2);
  fan.maxPower(true);
  // delay(30000);
  // fan.maxPower(false);
  // delay(30000);



 // driveTrain.setPower(255, 255);
 // lcd.print(leftEncTicks);
  //Serial.println(frontLeftUltra.avg());


//    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //get vector from IMU
//    gyro = euler.x(); //x value of IMU
//    frontUltraVal = frontLeftUltra.avg();
//   //Serial.println(gyro);
//   fireSensor.useSensor(); //save flame sensor values to array
//
//   //Emergency start/stop button
//    debouncer.update();
//   if(debouncer.risingEdge()){
//     switch(startStop){
//       case STOPROBOT:
//         state = STOP; //stop robot
//         startStop = START; //set so next time button is pressed, robot starts
//         break;
//
//       case START:
//         state = WALLFOLLOW; //start robot
//         startStop = STOPROBOT; //set so next time button is pressed, robot stops
//         break;
//     }
//   }
//
//   //REVIEW:
//   if(returnHome){
//     if(x < 3 && y < 3){  // as the robot gets closer to the original starting position,
//                          // x and y should get closer to 0, at this point, want to stop robot
//       state = STOP;
//       displayXYZ(); //print to screen coordinates of candle
//     }
//   }
//   //Main flow control
//   switch(state){
//     case WALLFOLLOW:
//       driveFollow();
//
//       lcd.clear();
//       lcd.setCursor(0, 0);
//       lcd.print("WALLFOLLOWING");
//
//       break;
//
//     case STOP:
//       driveTrain.setPower(0, 0);
//       break;
//
//     case TURN: //REVIEW:
//       /******************************************************************************
//       * IDEA                                                                       *
//       * Turning should have its own PID, and it should not be based on ultrasonics *
//       ******************************************************************************/
//       turn(turnLeft);
//       if(goToFlame){
//         driveStraight(400);
//       }
//       else if(turnLeft){
//         islandTurn(20);
//       }
//       else if(cliff){
//         driveStraight(400);
//       }
//       else{
//         state=WALLFOLLOW;
//         wallFound=false;
//       }
//       break;
//
//     case FLAME: //REVIEW:
//
//       lcd.clear();
//       lcd.setCursor(0, 1);
//       //if(!fireSensor.isFire()){
//       //   break;
//       // }
//       lcd.print("Flame is in front");
//       driveTrain.setPower(0, 0);
//       bool hVal;
//       bool xVal;
//
//       if(!blowing){//REVIEW: It is a global variable in main (at top). NOTE: check if this class works
//         /**********************************************/
//         hVal = fireSensor.centerHeight();  //move flame sensor to be at center of flame in y/z direction
//         lcd.clear();
//         lcd.setCursor(0, 0);
//         lcd.print("Center Height");
//         //delay(200);
//         /**********************************************/
//         xVal = centerFlameX(); //move flame sensor to be at center of flame in x direction
//         /**********************************************/
//         //driveTrain.setPower(0, 0);
//         calculateHeight(); //determine height of candle
//         saveValues(); //save x, y, and z values (will change when robot returns home)
//         /**********************************************/
//       }
//       if(hVal && xVal){
//         //if(fireSensor.isFire()){//REVIEW: We can also make it do this for t amount of seconds and later check if its out
//         lcd.clear();
//         lcd.setCursor(0, 0);
//           lcd.print("Blowing");
//           fireSensor.blowOutCandle(); //extinguish the candle
//           blowing = true;
//           lcd.clear();
//           lcd.setCursor(0, 0);
//           if(!fireSensor.isFire()){
//           lcd.print("Blew out candle");
//         //}else{
//           returnHome = true; //use to have robot stop when returns to (0.0) posiion
//           state = STOP; //COMBAK: Change this to WALLFOLLOW: have robot continue driving home
//         }
//         //}
//       }
//       break;
//     // case TRAVELTOFLAME:
//     //   // //driveStraight(10);
//     //   // if(fireSensor.isFire()){
//     //   //   driveTrain.setPower(0,0);
//     //   //   state = FLAME;
//     //   // }
//     //   goToFlame = true;  //will cause robot to go to TRAVELTOFLAME switch case
//     //   //turnInitialize(RIGHT);
//     //   driveStraight(10);
//     //   break;
//
//      // case TURNRIGHTLINE:
//      //  //TODO:
//      //    driveTrain.setPower(-100, -100);
//      //    delay(1000);
//      //    driveTrain.setPower(0, 0);
//      //    cliff = true;
//      //    turnInitialize(RIGHT);
//      //    //driveStraight(100);
//      //    break;
//     case DRIVESTRAIGHT:
//
//
//     //delay(500);
//
//       // lcd.setCursor(0,0);
//       // lcd.print("DRIVESTRAIGHT");
//       while(distTraveled < finalDistance){
//         //islandTurn = true;
//         // if(fireSensor.isFire()){
//         //   driveTrain.setPower(0,0);
//         //   state = FLAME;
//         // }
//          if(isSensorCliff()){
//           driveTrain.setPower(0,0);
//           state = TURNRIGHTLINE;
//         }
//         else if(frontUltra.avg() < 15 && !goToFlame){
//           goToFlame = false; //not in flame
//           turnLeft = false;
//           cliff = false;
//           turnInitialize(RIGHT);
//         }
//         else if(frontLeftUltra.avg() < 25 && !goToFlame){
//           state = WALLFOLLOW;
//         }
//         //Encoder PID
//         float gyroPercentage = .1;
//         float encoderPercentage = .9;
//         float encoderError = encoderPID.calc(leftEncTicks, rightEncTicks);
//         lcd.setCursor(0,0);
//         lcd.print(leftEncTicks);
//
//         //Gyro PID
//         imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//         int gyroError = gyroPID.calc(gyroLookUp(gyro), euler.x());
//         //Complimentary Filter
//         int driveCompFilter = (gyroPercentage * gyroError) + (encoderPercentage * encoderError);
//         newLeftSpeed = baseLeftSpeed - driveCompFilter;
//         newRightSpeed = baseRightSpeed + driveCompFilter;
//         driveTrain.setPower(newLeftSpeed, newRightSpeed);
//         lcd.setCursor(0,1);
//         lcd.print(newLeftSpeed);
//         lcd.setCursor(10,1);
//         lcd.print(newRightSpeed);
//         distTraveled = returnDistance();
//
//         //delay(100);
//       }
//
//
//          if(goToFlame){
//            turnInitialize(RIGHT);
//          }
//
//          else if (turns<2){
//            turns++;
//            turnInitialize(LEFT);//TODO
//          }
//         else{
//             turnLeft=false;
//             turns=0;
//             state=WALLFOLLOW;
//           }
//
//        break;
//
// }
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
  lcd.clear();
  baseLeftSpeed=0;
  baseRightSpeed=0;
  calcXandY();
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  switch(turnDir){
    case LEFT:
      desiredGyro= ((int) euler.x() + 270) % 360;

      turnLeft = true;
      //state=TURN;
      turn(turnLeft);
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
      turn(turnLeft);
      break;
  }
  return turnDir;
}


/**
 * Driving control
 */
void driveFollow(){
  if(isSensorCliff()){
    state = TURNRIGHTLINE;
    lcd.setCursor(5,1);
    lcd.print("line");
    delay(400);
    cliff = true;
  }
  // int count=millis()%2;
  // if(count == 1){
  //   frontUltraVal = frontLeftUltra.avg();
  // }
  // else if(count==0){
  //   backUltraVal = backLeftUltra.avg();
  // }
  //If front ultrasonic triggered (wall in front)
  //Serial.println(frontUltra.avg());
  if(frontUltra.avg() < 15){
    driveTrain.setPower(0, 0); //Stop robot
    lcd.clear();          //COMBAK: Remove this, for testing
    lcd.setCursor(5, 1);
    lcd.print("FRONT ULTRA TRIGGERED");
    //delay(400);
    frontUltra.clear();
    calcXandY(); //Calculate x and y

    //FOR TESTING: Print x and y value
    char message[] = "Distance travelled";
    printLCD(x,y,message);

    turnInitialize(RIGHT);   //REVIEW: This should be moved to TURN because line followers uses it as well
  }

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
  else if(fireSensor.isFire()){
    //if(sideUltra.avg() < 10){ //TODO: Test value to see distance for flame
    //state = FLAME;
    //centerFlameX();  //center flame in x direction
    //driveToFlame();
    lcd.setCursor(5,1);
    lcd.print("flame");
    //delay(300);
    state = TRAVELTOFLAME;
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
  //lcd.clear();
  //set base speeds
  baseRightSpeed =baseLeftSpeed_120;
  baseLeftSpeed = baseRightSpeed_120;

  //ping ultrasonics in succession
  int count=millis()%2;
  if(count == 1){
    frontUltraVal = frontLeftUltra.avg();
  }
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

  if(frontUltraVal > 25){
    lcd.clear();          //COMBAK: Remove this, for testing
    lcd.setCursor(0, 1);
    lcd.print("NO Wall");
     turnLeft = true;
     //driveStraight(10);
    islandTurn(10);

    //if(reachedDistance){
    //turnLeft = turnInitialize(LEFT);
      //state = STOP;
      // lcd.clear();
      // lcd.setCursor(0, 1);
      // lcd.print("Stopped");
  //  turnInitialize(LEFT);
    // }else{
    //   return;
    // }
  }

  //PID control

  pValRight = driveStraightPID.calc(10,frontLeftUltra.avg());
  //pValLeft = driveStraightPID.calc(15, backUltraVal);
  Serial.println(frontUltraVal);
  newLeftSpeed = 120;
  newRightSpeed = 120 - pValRight;

  if (newLeftSpeed>255){
   newLeftSpeed=255;
  }
 if (newRightSpeed>255){
   newRightSpeed=255;
  }


  driveTrain.setPower(newLeftSpeed, newRightSpeed);
  lcd.setCursor(0,1);
  lcd.print(newLeftSpeed);
  lcd.setCursor(10,1);
  lcd.print(newRightSpeed);

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
  x = x + (temp *.0072*2)/ 8 * cos(gyro);   //REVIEW: Add back in gyro code
  y = y + (temp *.0072*2)/ 8 * sin(gyro);
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
    lcd.print("error");
  }
  bno.setExtCrystalUse(true);
}


/**
 * Center flame in x direction
 */
bool centerFlameX(){
  fireSensor.useSensor();
  int centerXError = centerFlameXPID.calc(CENTERVAL_X, fireSensor.getx()); //PID based on flame x value and centered x value
  int newSpeed = 90 + centerXError;
  int encoderError = encoderPID.calc(rightEncTicks, leftEncTicks);
  driveTrain.setPower(newSpeed+encoderError, newSpeed-encoderError); //dont want wheels to turn, so make sure both are going in same direction
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(newSpeed);
  lcd.setCursor(0, 1);
  lcd.print(fireSensor.getx());
   if(abs(CENTERVAL_X-fireSensor.getx()) <=20){
     driveTrain.setPower(0, 0);
     return true;
   }
   else{
    return false;
   }
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

  // gyro=euler.x();


  //COMBAK: Uncomment everything below
  // baseLeftSpeed=baseLeftSpeed_120;
  // baseRightSpeed=baseRightSpeed_120;
  //
  // lcd.setCursor(0,0);
  // lcd.print("in Drive straight");
  baseLeftSpeed = 200;
  baseRightSpeed = 200;
   distTraveled = returnDistance();
   finalDistance = distTraveled + distToGo;
   state=DRIVESTRAIGHT;

  //COMBAK: New stuff added

  // driveTrain.setPower(baseLeftSpeed, baseRightSpeed);
  // Serial.println(gyroError);
  //
  //
  // float gyroPercentage = .1;
  // float encoderPercentage = .9;
  // float encoderError = encoderPID.calc(leftEncTicks, rightEncTicks);
  // lcd.setCursor(0,0);
  // lcd.print(leftEncTicks);
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

  turnLeft = true;
  driveStraight(distance);
  // turnLeft = true;
  // turnInitialize(LEFT);
  // driveStraight(distance);
  // turnInitialize(LEFT);
  // driveStraight(20);
  // turnLeft = false;
}

bool isSensorCliff(){

  return digitalRead(LINEFOLLOWERONE) && digitalRead(LINEFOLLOWERTWO);


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
  leftEncTicks = 0;
  rightEncTicks = 0;
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  int currentAngle = (int) euler.x() % 360;
  while(currentAngle != desiredGyro){
    frontUltraVal = frontLeftUltra.avg();
    //proportionalVal = turnPID.calc(currentAngle, desiredGyro);
    proportionalVal = 0;
    // newLeftSpeed = 0 - proportionalVal;
    // newRightSpeed = 0 + proportionalVal;
    //
  if(!turnLeft){
      newLeftSpeed = 120 + (-1 * proportionalVal);
      newRightSpeed = -120 + proportionalVal;
    }
    else{
      newLeftSpeed = -120 + proportionalVal;
      newRightSpeed = 120 + (-1* proportionalVal);
    }

    driveTrain.setPower(newLeftSpeed, newRightSpeed);

    lcd.setCursor(0, 0);
    lcd.print("TURNING");
    lcd.setCursor(0, 1);
    lcd.print(currentAngle);
    lcd.setCursor(10, 1);
    lcd.print(desiredGyro);
    lcd.setCursor(10, 0);
    lcd.print(abs(desiredGyro - currentAngle));
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    currentAngle = (int) euler.x() % 360;
  }

  // driveTrain.setPower(0, 0); //COMBAK: Remove these lines
  // delay(5000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Finished turning");
  driveTrain.setPower(0, 0);
}
