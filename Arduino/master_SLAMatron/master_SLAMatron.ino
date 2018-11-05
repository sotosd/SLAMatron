/*
 * @file master_SLAMatron.ino
 *
 * Sotirios S. Dimitras 2018
 * 
 * Electrical and Computer Engineering Department at Technical University of Crete (TUC)
 * Thesis : Design and Implementaion of an Autonomous Robotic Vehicle for Mapping
 * 
 * SLAMatron / An Autonomous Robotic Vehicle for Mapping .
 * (Simultaneous Localization And Mapping)
 * 
 * A robot using MPU 6050 IMU, BT, 2 Ultrasonic Sensors and 2 uln2003 for controling the 28byj-48 steppers    
 * The robot can be programmed as desplayed at the commented section in loop() in order to execute SLAM.
 * The robot follows the left sided wall, as long there is one. It steps for 10cm each time that drives forward (can be changed).  
 * If any obstacle is found it can turn left or right (-90 and +90 degrees). 
 * It scans with the front sided USensor, which is attached on a servo motor, and stores it the distances measured. 
 * The measured distances correspond to the  angles [0,180] (stepping 10 degrees at a time => 19 values)
 * The IMU prevents drift and allows for precise direction when the robot turns without encoders. 
 * The measured distances, together with robot's heading and the robot steps, are being sent to base/station (pc)
 * where a python script processes them and produces the wanted map (Occupancy Grid Mapping).
 *
 */

// ================================================================
// ===                      MPU/IMU SECTION                     ===
// ================================================================
/* ============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2012 Jeff Rowberg

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required
#include "Wire.h"


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   ========================================================================= */
// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_EULER

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container

VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float hed; //is being used in order to check whether the IMU is allready settled and can provide us with stabilised values.

// ================================================================
// ===                    SERVO'S LIBRARY                       ===
// ================================================================

#include <Servo.h>              // Add library
Servo _servo;               // Define any servo name

int servo_position_init = 90 ; // Define servo's initial position


// ================================================================
// ===         STEPPER MOTOR'S LIBRARY / PIN Layout             ===
// ================================================================

// For controling the two stepper motors we are going to use thw AccelStepper library.
// It has many interersting and realy helpfull functions which make our lifes easier to use to the steppers.
#include <AccelStepper.h>
#define HALFSTEP 8

// Right Motor pin definitions
#define RmotorPin1  6     // IN1 on the ULN2003 driver 1
#define RmotorPin2  5     // IN2 on the ULN2003 driver 1
#define RmotorPin3  4     // IN3 on the ULN2003 driver 1
#define RmotorPin4  3     // IN4 on the ULN2003 driver 1

// Left Motor pin definitions
#define LmotorPin1  7      // IN1 on the ULN2003 driver 2
#define LmotorPin2  8      // IN2 on the ULN2003 driver 2
#define LmotorPin3  10     // IN3 on the ULN2003 driver 2
#define LmotorPin4  11     // IN4 on the ULN2003 driver 2

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper with 28BYJ-48
AccelStepper stepperR(HALFSTEP, RmotorPin1, RmotorPin3, RmotorPin2, RmotorPin4); // Rigth wheel
AccelStepper stepperL(HALFSTEP, LmotorPin1, LmotorPin3, LmotorPin2, LmotorPin4); // Left wheel


/*The following variables will be needed in order to calculate how many steps the steppers have to run
  in order to drive a specified distance */
// Constant for wheel diameter
const float wheeldiameter = 67.70; // Wheel diameter in millimeters, change if different

// Float number of steps for a stepper motor 's full revolution
const float stepcount = 4096.00;  // Change to match value of your own stepper motor


// ================================================================
// ===                  DEFINE USONARS' PINS                    ===
// ================================================================
#define Front_Sensor_Trig A3
#define Front_Sensor_Echo A2

#define Side_Sensor_Trig 12
#define Side_Sensor_Echo 13

//float number that describes the distance between the left side wall and the robot
float side_dist = 0.0;
//an array of all measured distances of objects located for 0 to 180 degrees round the robot's front side
float meas[37];

// ================================================================
// ===                 BLUETOOTH PIN LAYOYT                     ===
// ================================================================
#include "SoftwareSerial.h"

#define BT_RX A0
#define BT_TX A1

SoftwareSerial serial_connection(BT_TX, BT_RX); //Create a serial connection with TX and RX on these pins

/*The following variables will be needed in order to create the comunication between SLAMatron and python
  script.
  SLAMatron will send to PC station all the stored claculated distances of the objects (if found any) around it
  and its possition. Then, theese values will be processed by a python script in order to create the desired map.*/
volatile float AHeading = 0.0; //aka Artificial Heading. By design we only +-90 degrees turn.So we know that the heading will either be one of the -180,-90, 0,90,180
//This variable is being set properly to each new value whenever the agent has to turn.
//That information is being send at the base station for further processing.
float DistanceToDrive = 10.0; //Distance that the robot has to drive in forward


// ================================================================
// ===           IMU'S INTERRUPT DETECTION ROUTINE              ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  Wire.begin();
  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  serial_connection.begin(9600);


  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //  while (Serial.available() && Serial.read()); // empty buffer
  //  while (!Serial.available());                 // wait for data
  //  while (Serial.available() && Serial.read()); // empty buffer again


  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  // those gyro offsets is being collected after callibrating your gyro
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(42);
  mpu.setYGyroOffset(-37);
  mpu.setZGyroOffset(26);
  mpu.setXAccelOffset(-3092);
  mpu.setYAccelOffset(1578);
  mpu.setZAccelOffset(1183);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  delay(3000); // Wait 15 - 40 seconds to allow setup of MPU6050.


  //Attach Servo Pin and initialize its position to look straight (90` degrees).
  //Detach it when the process is done in order to avoid the buzzing sound.
  _servo.attach (9);          // Define the servo signal pins
  _servo.write(servo_position_init);
  delay(500);
  _servo.detach();

  //Set all needed information for Left and Right Stepper Motors
  stepperR.setMaxSpeed(2000.0);
  stepperR.setAcceleration(200.0);
  stepperR.setSpeed(650.0);


  stepperL.setMaxSpeed(2000.0);
  stepperL.setAcceleration(200.0);
  stepperL.setSpeed(650.0);


  //Initialize all Sensors' pins
  pinMode(Front_Sensor_Trig, OUTPUT);
  pinMode(Front_Sensor_Echo, INPUT);

  pinMode(Side_Sensor_Trig, OUTPUT);
  pinMode(Side_Sensor_Echo, INPUT);

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
int state = 6;
int tcount = 0;
int i;
void loop() {
  //1st test
  //  if (state == 0) {
  //    CheckIMU( &state);
  //  }
  //  if (state == 1) {
  //    driveStraightDistance(CMtoSteps(10.0));
  //    turnAbsolute(90.0);
  //    //      get_heading(&hed);
  //    //      Serial.println(hed);
  //    tcount += 1;
  //    if (tcount == 3) {
  //      state = 2;
  //    }
  //  }
  // if (state == 2) {
  // Serial.println("done");
  // state = 3;
  //}

  //2nd test
  //    if (state == 0) {
  //      CheckIMU( &state);
  //    }
  //    if (state == 1) {
  //      Front_Sonar_Read();
  //      if (meas[10] > 25.00) {
  //        state = 2;
  //      }
  //      else {
  //        state = 3;
  //      }
  //      delay(50);
  //    }
  //    if (state == 2) {
  //      driveStraightDistance(CMtoSteps(DistanceToDrive));
  //      state = 10;
  //    }
  //    if (state == 3) {
  //      DistanceToDrive = meas[10] - 5.00;
  //      driveStraightDistance(CMtoSteps(DistanceToDrive));
  //      state = 4;
  //      DistanceToDrive = 25.00;
  //    }
  //    if (state == 4) {
  //      tcount += 1;
  //      turnAbsolute(90.0);
  //      state = 10;
  //    }
  //    if (state == 10) {
  //
  //      send_measurments();
  //      state = 1;
  //      if (tcount == 2) {
  //        state = 11;
  //      }
  //    }
  //    if (state == 11) {
  //      end();
  //      state = 12;
  //    }

}

// ================================================================
// ===                        FUNCTIONS                         ===
// ================================================================

//Function that sends measurments of the robot's surroundings.
//We send AHeading (aka Artificial Heading) and distance that robot traveled in order to compute its coordinates on the grid
//We also send all measured distances of the obgects around the robot, in order to process them and create the corresponding map.
//The format that the information is being sent, has to be as the following as it has to match while being read by the python script
void send_measurments() {
  int i = 0;
  serial_connection.print("Pose " + String(AHeading) + " " + String(DistanceToDrive));
  delay(4);
  serial_connection.print( " UMeas ");
  for (i = 0; i < 19; i++) {
    serial_connection.print( String(meas[i]) + " ");
  }
  serial_connection.print( " EndMeas");
  delay(4);
  serial_connection.println();
  delay(4);

}

//End() Function is helps to terminate the python script and show the final mapping when the robot stops moving and
//collecting new information
void end() {
  serial_connection.println("end");
}


//The following function is being used in order to get the distance between the robot and the wall on its left
//Thus, we are able to understand if there is a wall so we can follow it.
//The sensor responsible for this task is located on the back left side of the robot.
//This distance information, plus the distance information measured from the next function when the servo pose equals to 180, will also help
//us to alligne the robot to the wall next to it.
void Side_Sonar_Read() {
  float duration, temp = 0;
  int count = 0;
  while (count < 5) {
    digitalWrite(Side_Sensor_Trig, LOW);
    delayMicroseconds(2);

    digitalWrite(Side_Sensor_Trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(Side_Sensor_Trig, LOW);

    duration = pulseIn(Side_Sensor_Echo, HIGH);
    temp = temp + ((duration / 2) * 0.0344) - 1;
    count++;
  }
  side_dist = temp / count ;

}//End Side_Sonar_Read

//The following function helps us measure distance between the robot and the objects,if there are any.
//It collects 19 measurments which are being collected from an ultrasonic sensor attached on top of a servo motor.
//The 19 measurments correspond to the range [0-180](degrees) with a step of 10 degrees.
//The 0 degree possition is on the right side of the robot, the 90 degrees possition facing at the front of the robot and the 180 degrees
//possition is at the left side of the robot
//It is omportand to be mentioned that in order to get more accurate measurments, we repeat twice the procedure and store the average of the two results.
void Front_Sonar_Read() {
  int i = 0, servo_position;
  float duration;
  _servo.attach (9);
  for (servo_position = 0; servo_position <= 180; servo_position += 10) {

    _servo.write(servo_position);
    delay(30);
    digitalWrite(Front_Sensor_Trig, LOW);
    delay(4); // delays are required for a succesful sensor operation.
    digitalWrite(Front_Sensor_Trig, HIGH);

    delay(10); //this delay is required as well!
    digitalWrite(Front_Sensor_Trig, LOW);
    duration = pulseIn(Front_Sensor_Echo, HIGH);
    meas[i] = (duration * 0.0344 / 2); // convert the distance to centimeters
    //Serial.println(String(servo_position)+" "+String(meas[i])); //Debug MSG
    i++;
    delay(300);

  }
  for (servo_position = 180; servo_position >= 0; servo_position -= 10) {
    _servo.write(servo_position);
    delay(30);
    i--;
    digitalWrite(Front_Sensor_Trig, LOW);
    delay(4); // delays are required for a succesful sensor operation.
    digitalWrite(Front_Sensor_Trig, HIGH);

    delay(10); //this delay is required as well!
    digitalWrite(Front_Sensor_Trig, LOW);
    duration = pulseIn(Front_Sensor_Echo, HIGH);
    //distance = (duration / 2) / 29.1; // convert the distance to centimeters
    meas[i] = (meas[i] + (duration * 0.034 / 2)) / 2.0;
    //Serial.println(String(servo_position)+" "+String(duration * 0.034 / 2)); //Debug MSG
    delay(300);

  }
  //We constantly attach and detach the servo motoro, in order to avoid the annoying buzzing sound
  //and safe some energy to the batteries
  _servo.write(servo_position_init);
  delay(500);
  _servo.detach();

}// End Front_Sonar_Read


// Function to convert from centimeters to steps
int CMtoSteps(float cm) {

  int result;  // Final calculation result
  float circumference = (wheeldiameter * 3.14) / 10; // Calculate wheel circumference in cm
  float cm_step = circumference / stepcount;  // CM per Step

  float f_result = cm / cm_step;  // Calculate result as a float
  result = (int) f_result; // Convert to an integer (note this is NOT rounded)

  return result;  // End and return result

}//End CM to Steps

// Function that drives SLAMatron forward for a specified number of steps
void driveStraightDistance(int steps) {
  stepperR.move(steps);
  stepperL.move(steps);
  //The run() function must be called frequently until the motor is in the
  //desired position, after which time run() will do nothing.
  while (stepperR.distanceToGo() != 0 || stepperL.distanceToGo() != 0) {
    stepperL.setMaxSpeed(2000.0);
    stepperR.setMaxSpeed(2000.0);
    stepperL.run();
    stepperR.run();
  }
}//End Drive Straight

// Function that turns SLAMatron in a specified angle.
// We have an offset within 0.5 degrees of the desired angle
void turnAbsolute(float target) {
  static float Heading, targ;
  byte Turn; //0 if turning Left, 1 if turning Right
  //condition ? expression-true : expression-false
  (target > 0) ? Turn = 1 : Turn = 0;

  AHeading += target;
  if (AHeading >= 180) {
    AHeading = -180.0 + (AHeading - 180.0);
  }
  else if (AHeading <= -180) {
    AHeading = 180.0 - (AHeading + 180.0);
  }

  get_heading(&Heading);
  // Calculate the target (Heading now + how many degrees we need to turn (+-90))
  targ = Heading + target;
  // The IMU, as I 've set it up, acknowledges values in the range [-179.9, +179.9]
  //So we need to translate properly the target every time that the robot has to make a turn.
  if (targ > 180) {
    targ = -180.0 + (targ - 180.0);
  }
  else if (targ < -180) {
    targ = 180.0 - (targ + 180.0);
  }
  else if (targ == 180) {
    targ = 179.9;
  }
  else if (targ == -180) {
    targ = -179.9;
  }
  //Serial.println("Heading " + String(Heading) + " - Target " + String(targ)); //Debug MSG
  //Turning Right
  if (Turn == 0) {
    while (abs(Heading - targ) >= 0.5) {
      stepperR.setSpeed(-650);
      stepperL.setSpeed(650);
      stepperL.runSpeed();
      stepperR.runSpeed();
      get_heading(&Heading);
    }
  }
  //Turning Left
  if (Turn == 1) {
    while (abs(Heading - targ) >= 0.5) {
      stepperR.setSpeed(650);
      stepperL.setSpeed(-650);
      stepperL.runSpeed();
      stepperR.runSpeed();
      get_heading(&Heading);
    }
  }
}//End Turn Absolute

//This function is being called every time we need to calculate the robot's heading
//Is being used in order the robot to turn left or right by 90 degrees with an error in +-0.5 degrees.
void get_heading(float *head) {
  //    while (!mpuInterrupt) {
  //    } //End While. This is necessary during non-interrupt time periods.


  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;



    // display angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    *head = (ypr[0] * 180 / M_PI);
  }
}//End Get Heading


//
// Check to see if the IMU has settled down and is giving a steady heading.
// If it hasn't disable the go button.

void CheckIMU(int *state)
{
  static int Init = 1, Count;
  static float oHeading = 0.0;
  get_heading(&hed);

  if (Init)
  {
    //      //If IMU not stable don't allow the robot to start navigating.
    //      if(*state == 1)
    //        *state = 0;

    Count++;
    if (Count == 100)
    {
      Count = 0;
      if (abs(hed - oHeading) < 0.01)
      {
        Init = 0;
        *state = 1;
      }
      else
        oHeading = hed;
    }
  }
}//END Check IMU
