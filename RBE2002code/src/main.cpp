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
float proportionalVal;
int newLeftSpeed;
int newRightSpeed;
float gyro;
bool returnHome;
float distTraveled;
float finalDistance;
MotorStates testMotor;

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
void islandTurn();
bool isSensorCliff();

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


//////////////////////
//Arduino Functions //
//////////////////////
void setup() {
  state = WALLFOLLOW; //Robot will start being stopped
  startStop = START; //Robot will move once button is pushed

  returnHome = false; //Not currently returning returning home

  //Interrupts
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), LeftEncoderTicks, CHANGE);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3), RightEncoderTicks, CHANGE);
  pinMode(11, INPUT_PULLUP);

  //Debouncer
  debouncer.attach(11);
  debouncer.interval(5);

  //Initialize all classes
  setupIMU();
  fireSensor.initialize();
  //calibrateLineSensor(); //TODO: Record values into memory
  fanInitialize();
  driveTrain.initialize();

  //PIDs
  driveStraightPID.setpid(30,.1,.02); //PID to drive straight  //was 30
  turnPID.setpid(13,.2,.02); //PID for turning
  centerFlameXPID.setpid(.8, .1, .01); //PID for centering flame
  encoderPID.setpid(.1, 0, 0);
  gyroPID.setpid(10, 0, 0);

  //Displays
  lcd.begin(16, 2);
  Serial.begin(115200);

  //TESTING
  testMotor.initialize();
}


void loop() {
 //testMotor.motorDrive(STRAIGHT);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //get vector from IMU
  gyro = euler.x(); //x value of IMU
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
      //lcd.print(fireSensor.getx());
      break;

    case TURN: //REVIEW:
      /******************************************************************************
      * IDEA                                                                       *
      * Turning should have its own PID, and it should not be based on ultrasonics *
      ******************************************************************************/
      leftEncTicks = 0;
      rightEncTicks = 0;

      gyro = (int) euler.x();
      proportionalVal = turnPID.calc(gyro, desiredGyro);
      newLeftSpeed = 0 - proportionalVal;
      newRightSpeed = 0 + proportionalVal;
      driveTrain.setPower(newLeftSpeed, newRightSpeed);

      lcd.setCursor(0, 0);
      lcd.print("TURNING");
      lcd.setCursor(0, 1);
      lcd.print(gyro);
      lcd.setCursor(10, 1);
      lcd.print(desiredGyro);
      lcd.setCursor(10, 0);
      lcd.print(abs(desiredGyro - gyro));

      // Serial.print("Gyro: ");
      // Serial.println(gyro);
      // Serial.print("desiredGyro: ");
      // Serial.println(desiredGyro);
      // Serial.print("Difference: ");
      // Serial.println(abs(gyro - desiredGyro));

      if(abs(desiredGyro - gyro)<5){

        if(goToFlame){
          state = TRAVELTOFLAME;
        }
        if(turnLeft){
          islandTurn();
        }
        if(cliff){
          driveStraight(400);
        }
        else{
          state=WALLFOLLOW;
        }
      }
      break;

    case FLAME: //REVIEW:

      lcd.clear();
      lcd.setCursor(0, 1);
      //if(!fireSensor.isFire()){
      //   break;
      // }
      lcd.print("Flame is in front");
      driveTrain.setPower(0, 0);
      bool hVal;
      bool xVal;

      if(!blowing){//REVIEW: It is a global variable in main (at top). NOTE: check if this class works
        /**********************************************/
        hVal = fireSensor.centerHeight();  //move flame sensor to be at center of flame in y/z direction
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Center Height");
        //delay(200);
        /**********************************************/
        xVal = centerFlameX(); //move flame sensor to be at center of flame in x direction
        /**********************************************/
        //driveTrain.setPower(0, 0);
        calculateHeight(); //determine height of candle
        saveValues(); //save x, y, and z values (will change when robot returns home)
        /**********************************************/
      }
      if(hVal && xVal){
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
        }
        //}
      }
      break;
    case TRAVELTOFLAME:
      driveStraight(10);
      if(fireSensor.isFire()){
        driveTrain.setPower(0,0);
        state = FLAME;
      }
      break;

     case TURNRIGHTLINE:
      //TODO:
        turnInitialize(RIGHT);
        driveStraight(100);
        if(frontUltra.avg() < 15){
          goToFlame = false; //not in flame
          turnLeft = false;
          cliff = false;
          turnInitialize(RIGHT);
        }
        break;
    case DRIVESTRAIGHT:

      if(distTraveled < finalDistance){

        //Encoder PID
        int encoderError = encoderPID.calc(leftEncTicks, rightEncTicks);


        //Complimentary Filter
        //int driveCompFilter = (gyroPercentage * gyroError) + (encoderPercentage * encoderError);
        newLeftSpeed = baseLeftSpeed_120 - encoderError;
        newRightSpeed = baseRightSpeed_120 + encoderError;
        driveTrain.setPower(newLeftSpeed, newRightSpeed);

        lcd.print(newLeftSpeed);

        //delay(100);
      }
       else{
       turnInitialize(LEFT);//TODO
       }

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
  lcd.clear();
  baseLeftSpeed=0;
  baseRightSpeed=0;
  switch(turnDir){
    case LEFT:
      desiredGyro=((int) gyro - 90) % 360;
      if(desiredGyro < 0){
        desiredGyro += 360;

      }
      turnLeft = true;
      state=TURN;
      break;
    case RIGHT:
      desiredGyro=((int)gyro + 90) % 360;
      // if(desiredGyro > 360){
      //   desiredGyro =((int)gyro+90)-360;
      // }
      state=TURN;
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
    lcd.setCursor(0, 0);
    lcd.print("FRONT ULTRA TRIGGERED");
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
   state = FLAME;
    //centerFlameX();  //center flame in x direction
    //driveToFlame();
    return;
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
  //set base speeds
  baseRightSpeed =150;
  baseLeftSpeed = 150;

  //ping ultrasonics in succession
  int count=millis()%2;
  if(count == 1){
    frontUltraVal = frontLeftUltra.avg();
  }
  else if(count==0){
    backUltraVal = backLeftUltra.avg();
  }

    /////////////
    // TESTING //
    /////////////
    /////////////
  // Serial.print("Left: ");
  // Serial.println(frontUltraVal);
  // Serial.print("Right: ");
  // Serial.println(backUltraVal);

  if(frontUltraVal > 15){
    lcd.clear();          //COMBAK: Remove this, for testing
    lcd.setCursor(0, 1);
    lcd.print("NO Wall");
    driveStraight(10);
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
  proportionalVal = driveStraightPID.calc(frontUltraVal, backUltraVal);
  newLeftSpeed = baseLeftSpeed - proportionalVal;
  newRightSpeed = baseRightSpeed + proportionalVal;
  driveTrain.setPower(newLeftSpeed, newRightSpeed);
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
  Serial.println("Calibrating....");
  delay(500);

  qtraSix.emittersOn();
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode

  qtraSix.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  EEPROM.readBlock<unsigned int>(addrCalibratedMinimumOn, qtraSix.calibratedMinimumOn, 8);
  EEPROM.readBlock<unsigned int>(addrCalibratedMaximumOn, qtraSix.calibratedMaximumOn, 8);

  Serial.println("EEPROM Recall Complete");

  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration
  delay(1000);
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
  unsigned long drivePast = leftEncTicks + 100;

  //Drive past flame
  if(leftEncTicks < drivePast){
    driveTrain.setPower(200, 200);
  }
  else{
    goToFlame = true;  //will cause robot to go to TRAVELTOFLAME switch case
    turnInitialize(RIGHT);
  }
}


/**
 * Drive straight using complimentary filter of gyro and encoders
 */
bool driveStraight(float distToGo){
  //NOTE: These values must add to 1
  // float gyroPercentage = 0;
  // float encoderPercentage = 1;
  //delay(500);
  //Gyro PID
  // imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  // int gyroError = gyroPID.calc(gyro, euler.x());
  // gyro=euler.x();
  lcd.setCursor(0,1);
  lcd.print("in Drive straight");
  distTraveled = returnDistance();
  finalDistance = distTraveled + distToGo;
  state=DRIVESTRAIGHT;

}

double returnDistance(){
  double temp= (leftEncTicks + rightEncTicks)/2;
  double distance = distance + ((temp *.0072*2)) / 8;
  Serial.println(distance);
  return distance;
  //TODO: For final distance, must use ultrasonic to get distance from robot to candle
}

void islandTurn(){
  driveStraight(10);
  turnLeft = true;
  turnInitialize(LEFT);
  driveStraight(6);
  turnLeft = false;
}

bool isSensorCliff(){
  if(qtraSix.readLine(sensors) > 100){
    driveTrain.setPower(0, 0);
    calcXandY();
    return true;
  }else{
    return false;
  }
}
