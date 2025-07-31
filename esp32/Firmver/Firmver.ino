#include <Stepper.h>
#include <NewPing.h>
#include <TFMPlus.h>  // Include TFMini Plus Library v1.5.0
#include <Adafruit_VL53L0X.h>
#include <Wire.h>
 
const int pinMotorIn1 = 19;
const int pinMotorIn2 = 5;
const int pinMotorIn3 = 18;
const int pinMotorIn4 = 17;
const uint8_t pinSonarTrigger = 33;
const uint8_t pinSonarEcho = 32;
const int pinTFMiniPlusTX = 27;
const int pinTFMiniPlusRX = 16;
 
const int motorRevolutionSteps = 2048; // change this to fit the number of steps per revolution
// Revolutions per minute.
const int rpm = 8;
 
// 180 degree is 1/2 of motorRevolutionSteps.
int deg180 = motorRevolutionSteps / 2;
// 90 degree is 1/4 of motorRevolutionSteps or 1/2 of deg180.
int deg90 = deg180 / 2;

const unsigned int sonarMaxDistanceCm = 450;
// markerDistanceCm is distance in centimeters to marker.
// Marker used to find front.
const unsigned long markerDistanceCm = 5;

const int devHCSR04 = 1;
const int devTFMiniPlus = 2;
const int devVL53L0X = 3;
 
// Set-up shift of sensors.
const int devVL53L0XShiftMm = 10;
const int devHCSR04ShiftMm = 0;
const int devTFMiniPlusShiftMm = 0;
 
// initialize the stepper library
 
NewPing sonar(pinSonarTrigger, pinSonarEcho, sonarMaxDistanceCm);
Stepper stepperMotor(motorRevolutionSteps, pinMotorIn1, pinMotorIn2, pinMotorIn3, pinMotorIn4);
TFMPlus tfmP;
// Константы для VL53L0X
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
 
void setup() {
   // initialize the serial port
   Serial.begin(115200);
   Serial2.begin(115200, SERIAL_8N1, pinTFMiniPlusRX, pinTFMiniPlusTX);
   
   Serial.printf("set motor speed");
 
   stepperMotor.setSpeed(rpm);
   Serial.println("init VL53L0X");
    if (!lox.begin()) {
        Serial.println("Failed to initialize 88 VL53L0X");
        // while (1);
    }
   
   Serial.println("init VL53L0X done");
// stepperMotor.step(400);
// return;
 
   tfmP.begin(&Serial2);
   printf( "Soft reset: ");
   if( tfmP.sendCommand( SOFT_RESET, 0))
   {
       printf( "passed.\r\n");
   }
   else tfmP.printReply();
 
   if( tfmP.sendCommand( SET_FRAME_RATE, FRAME_200))
   {
       printf( "%2uHz.\r\n", FRAME_200);
   }
   else tfmP.printReply();
}
 
// Application state.
int state = 0;
 
const int stateUninitialized = 0;
const int stateMarkerDetected = 1;
const int statePositionAdjusting = 2;
const int statePositionAdjusted = 3;
const int statePositionStartScan = 4;
 
int markerSteps = 0;
int markerFullSteps = 0;
const int adjustingStepsAtOnce = 10;
 
const int stepsPerScan = 5;
int stepsScanned = 0;
// 1 - left to right.
// -1 - right to left.
int scanDirection = 1;
int stepsForScan = deg180;

// Variables for LiDAR
int dist; /* Actual distance measurements of LiDAR */
float temperature;
int strength; /* Signal strength of LiDAR */
unsigned char check; /* Save check value */
unsigned char uart[9];  /* Save data measured by LiDAR */
int rec_debug_state = 0x01; // Receive state for frame
 
void writeInfo(float angle, int distanceCm, int dev) {
   const char *model = "Unknown device";
   switch (dev) {
       case devHCSR04:
           model = "HC-SR04";
           break;
       case devTFMiniPlus:
           model = "TF Mini Plus";
           break;
       case devVL53L0X:
           model = "VL53L0X";
           break;
       default:
           // We not support this device.
           return;
   }
 
   Serial.printf("data,%s,%.2f,%d\n", model, angle, distanceCm);
}
 
void writeInfoAll(float angle, int distance1, int distance2, int distance3) {
  Serial.printf("data,%.2f,%d,%d,%d\n", angle, distance1, distance2, distance3);
}
 
int measureTFMiniPlusNew() {
   int16_t tfDist = 0;    // Distance to object in centimeters
   int16_t tfFlux = 0;    // Strength or quality of return signal
   int16_t tfTemp = 0;    // Internal temperature of Lidar sensor chip
 
   if( tfmP.getData( tfDist, tfFlux, tfTemp)) // Get data from the device.
   {
       return int(tfDist);
   }
   else                  // If the command fails...
   {
       tfmP.printFrame();  // display the error and HEX dataa
   }
}
 
int measureTFMiniPlus() {
   if (Serial2.available()) {
       if (rec_debug_state == 0x01) {
           uart[0] = Serial2.read();
           if (uart[0] == 0x59) {
               check = uart[0];
               rec_debug_state = 0x02;
           }
       } else if (rec_debug_state == 0x02) {
           uart[1] = Serial2.read();
           if (uart[1] == 0x59) {
               check += uart[1];
               rec_debug_state = 0x03;
           } else {
               rec_debug_state = 0x01;
           }
       } else if (rec_debug_state == 0x03) {
           uart[2] = Serial2.read();
           check += uart[2];
           rec_debug_state = 0x04;
       } else if (rec_debug_state == 0x04) {
           uart[3] = Serial2.read();
           check += uart[3];
           rec_debug_state = 0x05;
       } else if (rec_debug_state == 0x05) {
           uart[4] = Serial2.read();
           check += uart[4];
           rec_debug_state = 0x06;
       } else if (rec_debug_state == 0x06) {
           uart[5] = Serial2.read();
           check += uart[5];
           rec_debug_state = 0x07;
       } else if (rec_debug_state == 0x07) {
           uart[6] = Serial2.read();
           check += uart[6];
           rec_debug_state = 0x08;
       } else if (rec_debug_state == 0x08) {
           uart[7] = Serial2.read();
           check += uart[7];
           rec_debug_state = 0x09;
       } else if (rec_debug_state == 0x09) {
           uart[8] = Serial2.read();
           if (uart[8] == check) {
               dist = uart[2] + uart[3] * 256;  // Calculate distance
               strength = uart[4] + uart[5] * 256;  // Calculate strength
               temperature = uart[6] + uart[7] * 256;  // Calculate temperature
               temperature = temperature / 8 - 256;
 
               while (Serial2.available()) {
                   Serial2.read();
               }
           }
           rec_debug_state = 0x01;
       }
   }
 
   return dist+devTFMiniPlusShiftMm;
}

int measureHCSR04() {
   return (sonar.ping_cm() * 10)+devHCSR04ShiftMm;
}
 
int measureVL53L0X() {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) {
        return measure.RangeMilliMeter + devVL53L0XShiftMm;
    } else {
        return -1; // Если вне диапазона
    }
}
 
void scanAndRotate() {
   int hcSR04Distance = measureHCSR04();
   int tfMiniPlusDistance = measureTFMiniPlusNew();
   int vl53Dist = measureVL53L0X();
   float stepToAngle = float(360) / float(motorRevolutionSteps);
   float currentAngle = float(stepToAngle) * float(stepsScanned);
   //writeInfo(currentAngle, hcSR04Distance, devHCSR04);
   //writeInfo(currentAngle, tfMiniPlusDistance, devTFMiniPlus);
   //writeInfo(currentAngle, vl53Dist, devVL53L0X);
  
   writeInfoAll(currentAngle, hcSR04Distance, tfMiniPlusDistance, vl53Dist);
 
   int stepsToRotate = stepsPerScan * scanDirection;
   stepsScanned += stepsToRotate;
 
   if (stepsScanned >= stepsForScan) {
       scanDirection = -1;
   }
 
   if (stepsScanned <= 0) {
       scanDirection = 1;
   }
 
   stepperMotor.step(stepsToRotate);
}
 
void loop() {
  // return;
   if (state == stateUninitialized || state == stateMarkerDetected || state == statePositionAdjusting) {
       int distance = sonar.ping_cm();
 
       Serial.println(distance);
 
       if (distance == markerDistanceCm || distance == (markerDistanceCm + 1)) {
           if (state == stateUninitialized) {
               state = stateMarkerDetected;
           }
 
           markerSteps += adjustingStepsAtOnce;
 
           if (state == statePositionAdjusting) {
               markerSteps -= adjustingStepsAtOnce;
               markerFullSteps += adjustingStepsAtOnce;
           }
       } else {
           if (state == statePositionAdjusting) {
               stepperMotor.step(deg180 - (markerFullSteps / 2));
               state = statePositionAdjusted;
               delay(1000);
               Serial.println("front found");
               return;
           }
           if (state == stateMarkerDetected) {
               state = statePositionAdjusting;
           }
 
       }
 
       int step = -adjustingStepsAtOnce;
       if (state == statePositionAdjusting) {
           step = step * -1;
 
           if (markerSteps <= 0) {
               return;
           }
       }
 
       stepperMotor.step(step);
   }
 
   // Rotate motor on start point for scanning.
   if (state == statePositionAdjusted) {
       stepperMotor.step(deg90 * -1);
       state = statePositionStartScan;
       Serial.println("scanner is on start position");
   }
 
   // Now scanner is on right place and we can start scanning area.
   if (state == statePositionStartScan) {
       scanAndRotate();
   }
}