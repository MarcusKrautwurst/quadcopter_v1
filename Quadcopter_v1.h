// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _Quadcopter_v1_H_
#define _Quadcopter_v2_H_

#if defined(ARDUINO) && ARDUINO >= 100
      #include "Arduino.h"
    #else
      #include "WProgram.h"
    #endif
//add your includes for the project test here

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "PID_v1.h"
//#include "RC.h"
#include "config.h"
#include "SFE_BMP180.h"

//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project test here
void blinkLED();
void initMPU();
void initBarometer();
void initMotors();
void initPID();
void updateControllerInput();
void updateYPR();
void updatePID();
void updateMotors();
void updateDebugView();
void updateAltittude();
void updateTemperature();

//Do not add code below this line
#endif /* _test_H_ */
