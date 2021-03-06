#ifndef _config_H_
#define _config_H_

///////////// IMPORTANT NOTES /////////////

// It is very important to not use PIN 10 for anything, 
// its unfortunately reserved for the SD library

///////////// I2C  CONFIG  ///////////////////
#define I2C_ADRESS_MPU 0x69

/////////////// MISC  CONFIG  /////////////////
#define RC_ROUNDING_BASE 50
const uint16_t DELAY_ARM = 2000; // delay for arming the motors : default = 2000


///////////////// PIN CONFIG ///////////////////
#define LED_BLUE 13
#define LED_RED 14
#define LED_GREEN 15
#define SD_PIN 53
#define SD_TMP_PIN 10

/////////////// MOTOR CONFIG ////////////////////
#define MOTOR_FWD_L_PIN 3
#define MOTOR_FWD_R_PIN 4
#define MOTOR_BCK_L_PIN 5
#define MOTOR_BCK_R_PIN 6

///////////// ALTIMETER CONFIG //////////////////
#define ULTRASONIC_TRIGGER_PIN 38
#define ULTRASONIC_ECHO_PIN 40
#define ULTRASONIC_TIMEOUT 3000		// 10000 is around 1.72 meters
float ULTRASONIC_MAX_DISTANCE=ULTRASONIC_TIMEOUT/58;

#define BAROMETER_RESOLUTION 2		// 1=ultra low power | 2=standard | 3= high resolution | 4=ultra high resolution

///////////// FLIGHT PARAMETERS /////////////
#define MAX_PITCH 30
#define MAX_ROLL 30
#define MAX_YAW 180

#define MINSPEED 20
#define TAKEOFF_SPEED 35
#define MAXSPEED 180


//////////////// PID CONFIG /////////////
#define PITCH_P_VAL 0.168  //0.168 // 0.5
#define PITCH_I_VAL 0.654  //0.654 // 0
#define PITCH_D_VAL 0.008  //0.008 // 1

#define ROLL_P_VAL 0.168    //0.168 // 2
#define ROLL_I_VAL 0.654    //0.654 // 5
#define ROLL_D_VAL 0.008    //0.008 // 1

#define ALTITUDE_P_VAL 1    //0.168 // 2
#define ALTITUDE_I_VAL 1    //0.654 // 5
#define ALTITUDE_D_VAL 1    //0.008 // 1

#define YAW_P_VAL 0.00    //2
#define YAW_I_VAL 0.00    //5
#define YAW_D_VAL 0.00    //1

///////////// RC CONFIG /////////////
const int RC1_PIN = 36;			// remote channel 1
const int RC2_PIN = 36;			// remote channel 2
const int RC3_PIN = 32;			// remote channel 3

const int RXLo=920;
const int RXHi=1640;
const int RXDeadLo=1265;
const int RXDeadHi=1295;
const int RXMid=1280;


#endif
