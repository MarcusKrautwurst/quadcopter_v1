///////////// I2C  CONFIG  //////////////
#define I2C_ADRESS_MPU 0x69

///////////// PIN CONFIG /////////////
#define LED_PIN 13
#define MOTOR_FWD_L_PIN 10
#define MOTOR_FWD_R_PIN 11
#define MOTOR_BCK_L_PIN 6
#define MOTOR_BCK_R_PIN 9
#define MOTOR_TEST_PIN 5

///////////// FLIGHT CONFIG /////////////
#define MAX_PITCH 30
#define MAX_ROLL 30
#define MAX_YAW 180

// Speed values
#define MINSPEED 20
#define MAXSPEED 255

//// PID CONFIG
#define PITCH_P_VAL 0.168  //0.168 // 0.5
#define PITCH_I_VAL 0.654  //0.654 // 0
#define PITCH_D_VAL 0.008  //0.008 // 1

#define ROLL_P_VAL 0.168    //0.168 // 2
#define ROLL_I_VAL 0.654    //0.654 // 5
#define ROLL_D_VAL 0.008    //0.008 // 1

#define YAW_P_VAL 0.00    //2
#define YAW_I_VAL 0.00    //5
#define YAW_D_VAL 0.00    //1
