// TODO: Check if the encoder pins are wrong or the encoder values are set wrongly
#include "RunningMedian.h"
#include <DualVNH5019MotorShield.h>
#include <EnableInterrupt.h>


#define FRONT_LEFT_SENSOR 2
#define FRONT_CENTER_SENSOR 1
#define FRONT_RIGHT_SENSOR 0
#define FRONT_LONG_RANGE 4
#define RIGHT_RIGHT_SENSOR 3
#define RIGHT_BACK_SENSOR 5

#define ENCODER_LEFT1 11
#define ENCODER_LEFT2 13
#define ENCODER_RIGHT1 3
#define ENCODER_RIGHT2 5

boolean CONSTANT_CALIB = true;
int ONE_BLOCK_TICKS = 306;
int ANGLE_LEFT_90 = 388;
int ANGLE_RIGHT_90 = 388;
int MIN_SPEED = 150;
int MAX_SPEED = 280;
int TURN_SPEED = 240;
int ALIGN_DIST_FRONT = 475;
int DIST_FROM_WALL_FRONT = 100;
int BRAKE_DELAY = 0;

int TWO_BLOCK_TICKS = 612;
int THREE_BLOCK_TICKS = 918;
int FOUR_BLOCK_TICKS = 1224;
int FIVE_BLOCK_TICKS = 1530;

// Constants used in align()
int ALIGN_RIGHT = 0; // Adjust Rotation based on right sensors
int ALIGN_FRONT = 1; // Adjust Rotation based on front sensors
int ALIGN_DIST = 2;  // Adjust distance based on front sensors
int ALIGN_THRES = 5;
int ALIGN_CALIB_SPEED = 75;

// Constants used in calibration()
double P_STEER = 0.1;
int CALIB_SENSOR_MIN = 340;
int MIN_STEER = 0;
int MAX_STEER = 15;
int STEER_DELAY = 80; // 500ms
int RPM_PRINT_THRES = 300;

// Variables used by the program
long prevTime = 0;
int currSpeed = 250;
int prevSpeed = 0;
int dirL = 1;
int dirR = 1;

long prevCalibTime = 0;

// Calibration to steer the robot straight
boolean constCalib = true;
boolean moving = false;
boolean printed = false;

long numTicks = 0;
long ticksLeft = 0;
long ticksRight = 0;
DualVNH5019MotorShield md;

void setup() {
  Serial.begin(115200);
  // Initialisation for the encoders used 
  pinMode(ENCODER_LEFT1, INPUT);
  pinMode(ENCODER_LEFT2, INPUT);
  pinMode(ENCODER_RIGHT1, INPUT);
  pinMode(ENCODER_RIGHT2, INPUT);

  enableInterrupt(ENCODER_LEFT1, encoderLeft, RISING);
  enableInterrupt(ENCODER_RIGHT1, encoderRight, RISING);

  // Initialisation of motors
  md.init();
}

void encoderRight() {
  ticksLeft++;
}

void encoderLeft() {
  ticksRight++;
}

// Aligns the robot to obstacle or wall
// A0 - Aligns rotation of robot based on right sensors
// A1 - Aligns rotation of robot based on front sensors
// A2 - Aligns distance of robot to wall based on front sensors

void alignFront(){
  int err = 0;
  long maxTime = millis() + 800; // 1 second to align
  
  while (millis() < maxTime) {
    err = readSensor(FRONT_LEFT_SENSOR) - readSensor(FRONT_RIGHT_SENSOR);

    // Once the err is below the threshold (ALIGN_THRES), it means the robot is calibrated
    if (abs(err) <= ALIGN_THRES) {
      md.setBrakes(400, 400);
      break;
    } else if (err > ALIGN_THRES) {
      // Align Distance. Too close to wall
        md.setSpeeds(-ALIGN_CALIB_SPEED, -ALIGN_CALIB_SPEED);
    }
  }
  
  // Ensures that robot is stopped
  md.setBrakes(400, 400);
  // Reset Values
  resetTicks();
}

void alignRight() {
  int err = 0;
  long maxTime = millis() + 800; // 1 second to align
  
  while (millis() < maxTime) {
    err = readSensor(RIGHT_BACK_SENSOR) - readSensor(RIGHT_RIGHT_SENSOR);

    // Once the err is below the threshold (ALIGN_THRES), it means the robot is calibrated
    if (abs(err) <= ALIGN_THRES) {
      md.setBrakes(400, 400);
      break;
    } else if (err > ALIGN_THRES) {
      // Align Distance. Too close to wall
        md.setSpeeds(-ALIGN_CALIB_SPEED, ALIGN_CALIB_SPEED);
    }
  }
  
  // Ensures that robot is stopped
  md.setBrakes(400, 400);
  // Reset Values
  resetTicks();
}

void alignLeft() {
  int err = 0;
  long maxTime = millis() + 800; // 1 second to align
  
  while (millis() < maxTime) {
    err = readSensor(RIGHT_BACK_SENSOR) - readSensor(RIGHT_RIGHT_SENSOR);

    // Once the err is below the threshold (ALIGN_THRES), it means the robot is calibrated
    if (abs(err) <= ALIGN_THRES) {
      md.setBrakes(400, 400);
      break;
    } else if (err > ALIGN_THRES) {
      // Align Left. Too close to wall
      md.setSpeeds(ALIGN_CALIB_SPEED, -ALIGN_CALIB_SPEED);
    } 
  }
  
  // Ensures that robot is stopped
  md.setBrakes(400, 400);
  // Reset Values
  resetTicks();
}

// Sets numTicks, ticksLeft, ticksRight back to its original values;
void resetTicks() {
//  numTicks = ticksLeft;
//  ticksRight = numTicks;
  // TODO: Test with the following values. Should work
   numTicks = 0;
   ticksLeft = 0;
   ticksRight = 0;
}

// Moves the robot forward and backward based on number of blocks
// F2; - Moves the robot 2 blocks forward
// B3; - Moves the robot 3 blocks backward
// Note: Txx,xx,xx,xx; Serial command sets the ticks for the different blocks.
void forward(int blocks) {
  boolean isForward = blocks > 0;
  blocks = abs(blocks);
  int ticks = 0;
  switch (blocks) {
    case 1:
      ticks = ONE_BLOCK_TICKS;
      break;
    case 2:
      ticks = TWO_BLOCK_TICKS;
      break;
    case 3:
      ticks = THREE_BLOCK_TICKS;
      break;
    case 4:
      ticks = FOUR_BLOCK_TICKS;
      break;
    case 5:
      ticks = FIVE_BLOCK_TICKS;
      break;
    default:
      ticks = blocks * FIVE_BLOCK_TICKS / 5.0;
      break;
  }
  
  currSpeed = MAX_SPEED;
  dirL = isForward ? 1 : -1;
  dirR = isForward ? 1 : -1;
  numTicks += ticks;
}

// Turning based on angle
// L90; - Turn left 90 degrees
// R180; - Turn right 180 degrees
void turn(int angle) {
  boolean leftTurn = angle < 0;
  dirL = leftTurn ? -1 :  1;
  dirR = leftTurn ?  1 : -1;
  currSpeed = 240;
  if (angle == 90) {
    numTicks += ANGLE_RIGHT_90;
    return;
  } else if (angle == -90) {
    numTicks += ANGLE_LEFT_90;
  }
}

// When the robot is moving, it constantly checks its right sensors to ensure that the angle to aligned to the wall
void calibration() {
  if (!constCalib) {
    return;
  }
  
  // -X = Left, +X = Right
  if (moving && millis() > prevCalibTime + STEER_DELAY) {
    // Calibrate with right sensor
    int rightFrontReading = readSensor(RIGHT_RIGHT_SENSOR);
    int rightBackReading = readSensor(RIGHT_BACK_SENSOR);
    
    if (rightFrontReading < CALIB_SENSOR_MIN || rightBackReading < CALIB_SENSOR_MIN) {
      return;
    }

    int err = rightFrontReading - rightBackReading;
    double steerError = P_STEER * abs(err);
    steerError = steerError > MAX_STEER ? MAX_STEER : steerError > MIN_STEER ? steerError : MIN_STEER;
    steerError = err > 0 ? -1 * dirL * steerError : dirL * steerError;
    
    ticksRight += steerError;
    prevCalibTime = millis();
  }
}

long prevRPMTime = 0;
float currRPM = 0;
int measureTime = 20; // Every 40ms
long prevTicks = 0;

// For Testing
float prevRPM = 0;

void calculateRPM() {
  long currTime = millis();
  if (currTime - prevRPMTime > measureTime) {
    currRPM = (330 * (ticksLeft - prevTicks)) / (1.0 * (currTime - prevRPMTime));
    prevTicks = ticksLeft;
    prevRPMTime = currTime;
  }
}

float prevSpeedTime = 0;
int prevError = 0;
int calibLeft = 0;
int calibRight = 0;

// This is for fastest path where if it detects an obstacle, it stops before it hits.
// Return true when constant calibration is on
boolean fastestObsStop() {
  if (constCalib || dirL != 1 || dirR != 1) {
    return true;
  }

  int fastestThres = 100;
  if (readSensor(FRONT_LEFT_SENSOR) > ALIGN_DIST_FRONT - fastestThres ||
      readSensor(FRONT_CENTER_SENSOR) > ALIGN_DIST_FRONT - fastestThres ||
      readSensor(FRONT_RIGHT_SENSOR) > ALIGN_DIST_FRONT - fastestThres) {
    numTicks = ticksLeft + 42;
    return false;
  } else {
    return true;
  }
}


void loop() {
  calculateRPM();
  int ticksError = ticksRight - ticksLeft;
  
  // Run for 10000 ticks
  if (ticksLeft <= numTicks - 42 && calibLeft == 2 && fastestObsStop()) {
    moving = true;
    calibration();
    
    int error = 6 * ticksError;
    error = error > 20 ? 20 : error < -20 ? -20 : error;
//    if (prevError != error) {
//      Serial.println(error);
//      prevError = error;
//    }
    if (constCalib) {
      if (currSpeed != prevSpeed) {
        md.setM1Speed(dirR * (currSpeed - error));
        delay(36);
        md.setM2Speed(dirL * (currSpeed + error));
        prevSpeed = currSpeed;
      } else {
        md.setM1Speed(dirR * (currSpeed - error));
//        delay(6);
        md.setM2Speed(dirL * (currSpeed + error));
      }
    } else {
      md.setSpeeds(dirR * (currSpeed - error), dirL * (currSpeed + error));
    }
//  } else if (moving) {
  } else if (currRPM > 0) {
    if (currRPM <= 250) {
      md.setBrakes(400, 400);
    } else {
      md.setBrakes(400, 400);
    }

  // This sets the initial calibration on whether the robot moves too much or too little
  } else if (moving && calibLeft == 2) {
    calibLeft = numTicks - ticksLeft > 0 ? 1 : -1;
    calibRight = numTicks - ticksRight > 0 ? 1 : -1;

//    Serial.print("Current: ");
//    Serial.print(ticksLeft);
//    Serial.print(" ");
//    Serial.println(ticksRight);

    ticksLeft -= 2 * abs(ticksLeft - numTicks);
    ticksRight -= 2 * abs(ticksRight - numTicks);
  // Ensure the robot reverse back to the correct ticks
  } else if (moving && (calibLeft != 0 || calibRight != 0)) {
    
    if (calibLeft != 0) {
      if (ticksLeft >= numTicks - 1) {
        md.setBrakes(400, 400);
        calibLeft = 0;
      } else {
//        Serial.print("Left Speed");
//        Serial.println(dirL * calibLeft * 100);
        md.setM2Speed(dirL * calibLeft * 130);
      }
    }

    if (calibRight != 0) {
      if (ticksRight >= numTicks - 1) {
        md.setBrakes(400, 400);
        calibRight = 0;
      } else {
//        Serial.print("Right Speed");
//        Serial.println(dirR * calibRight * 100);
        md.setM1Speed(dirR * calibRight * 130);
      }
    }

//  } else if (turning && abs(ticksError) > 1) {
//    // Set as a single value to use
//    // error = leftTicks - (2 * numTicks)
//    // dirL * -1 * speed - Too much ticks
//    // dirL * speed - Too little ticks
//    
//    // Slow down when turning
//    if (ticksError > 0) {
//      md.setM2Speed(100);
//    } else {
//      md.setM1Speed(100);
//    }
//  } else if (moving) {
//    Serial.print(numTicks);
//    Serial.print(", ");
//    Serial.print(ticksLeft);
//    Serial.print(", ");
//    Serial.print(ticksRight);
//    Serial.print(", ");
//    Serial.println(ticksError);
//    moving = false;
  } else {
    delay(BRAKE_DELAY);
    resetTicks();
    constCalib = CONSTANT_CALIB;
    // This ensures the robot recalibrates
    moving = true;
    calibLeft = 2;
    calibRight = 2;
    serialIn();
  }
}

// RPi -> Arduino: Data that are sent from the RPi
// Following are the opcodes
// F / B  - Move the robot forward / backward number of blocks
// L / R  - Turn the robot left / right based on angle specified
// P0;    - Print out the sensor values
// A1/2/3 - Aligns robot to wall
// T / S  - Update constant values
void serialIn() {
  if (Serial.available()) {
    char code = Serial.read();
    char code2 = Serial.read();

    switch (code) {
      // Move forward by number of blocks
      case 'F':
        forward(1);
        if (code2) {
          code = code2;
          code2 = "";
        }
        break;
      // Used for backing out of holes
      case 'B':
        forward(-1);
        if (code2) {
          code = code2;
          code2 = "";
        }
        break;
        // Left by angles. L90, turn left
      case 'L':
        turn(90);
        if (code2) {
          code = code2;
          code2 = "";
        }
        break;
      // Right by angles. R90, turn right
      case 'R':
        turn(-90);
        if (code2) {
          code = code2;
          code2 = "";
        }
        break;
      // Print out sensor values
      case 'P':
        Serial.print(readSensor(FRONT_LEFT_SENSOR));
        Serial.print(",");
        Serial.print(readSensor(FRONT_CENTER_SENSOR));
        Serial.print(",");
        Serial.print(readSensor(FRONT_RIGHT_SENSOR));
        Serial.print(",");
        Serial.print(readSensor(RIGHT_RIGHT_SENSOR));
        Serial.print(",");
        Serial.print(readSensor(RIGHT_BACK_SENSOR));
        Serial.print(",");
        Serial.println(readSensor(FRONT_LONG_RANGE));
        break;
      // Three types of align
      case 'X':
        alignFront();
        Serial.println("DONE");
        if (code2) {
          code = code2;
          code2 = "";
        }
        break;
      // Left
      case 'Z':
        alignLeft();
        Serial.println("DONE");
        break;
      // Right
      case 'C':
        alignRight();
        Serial.println("DONE");
        break;
//      case 'S':
//        ONE_BLOCK_TICKS = 305;
//        ANGLE_LEFT_90 = Serial.parseInt();
//        ANGLE_RIGHT_90 = Serial.parseInt();
//        MIN_SPEED = Serial.parseInt();
//        MAX_SPEED = Serial.parseInt();
//        TURN_SPEED = Serial.parseInt();
//        ALIGN_DIST_FRONT = Serial.parseInt();
//        DIST_FROM_WALL_FRONT = Serial.parseInt();
//        BRAKE_DELAY = Serial.parseInt();
//        RPM_PRINT_THRES = Serial.parseInt();
//
//        delim = Serial.read();
//        Serial.print(ONE_BLOCK_TICKS);
//        Serial.print(',');
//        Serial.print(ANGLE_LEFT_90);
//        Serial.print(',');
//        Serial.print(ANGLE_RIGHT_90);
//        Serial.print(',');
////        Serial.print(ANGLE_LEFT_180);
////        Serial.print(',');
////        Serial.print(ANGLE_RIGHT_180);
////        Serial.print(',');
//        Serial.print(MIN_SPEED);
//        Serial.print(',');
//        Serial.print(MAX_SPEED);
//        Serial.print(',');
//        Serial.print(TURN_SPEED);
//        Serial.print(',');
//        Serial.print(ALIGN_DIST_FRONT);
//        Serial.print(',');
//        Serial.print(DIST_FROM_WALL_FRONT);
//        Serial.print(',');
//        Serial.print(BRAKE_DELAY);
//        Serial.print(',');
//        Serial.println(RPM_PRINT_THRES);
//        break;
    }
    if (isdigit(code)){
      int val1 = code - '0';
      forward(val1);
    }
  }
}

// read each sensor ~5msec
float readSensor(int IRpin) {
  RunningMedian samples = RunningMedian(9);
  for (int i = 0; i < 9; i ++) {
    samples.add(analogRead(IRpin));
  }
  return samples.getMedian();
}
