#include "Quadcopter_v1.h"
#include "config.h"

#define DEBUG


MPU6050 mpu(I2C_ADRESS_MPU);	// IMU
SFE_BMP180 pressure;			// Barometric sensor

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;


double Setpoint_Pitch, Input_Pitch, Output_Pitch;
double Setpoint_Roll, Input_Roll, Output_Roll;
double Setpoint_Altitude, Input_Altitude, Output_Altitude;
         // motor balances can vary between -100 & 100

PID myPID_P(&Input_Pitch, &Output_Pitch, &Setpoint_Pitch,PITCH_P_VAL,PITCH_I_VAL,PITCH_D_VAL, DIRECT);
PID myPID_R(&Input_Roll, &Output_Roll, &Setpoint_Roll,ROLL_P_VAL,ROLL_I_VAL,ROLL_D_VAL, DIRECT);
PID myPID_A(&Input_Altitude, &Output_Altitude, &Setpoint_Altitude,ALTITUDE_P_VAL,ALTITUDE_I_VAL,ALTITUDE_D_VAL, DIRECT);



//Log mylog(SD_PIN);	// SD Card logging

// MPU control/status vars
bool blinkState = false;
bool dmpReady = false;  // set true if DMP init was successful
volatile bool mpuInterrupt = false;	// indicates whether MPU interrupt pin has gone high
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yprLast[3] = {0.0f, 0.0f, 0.0f};

float ch1, ch2, ch3, ch4, ch5;         			// RC channel inputs
												// ch1: pitch | ch2: roll | ch3: speed
float velocity,velocityLast;

uint16_t temp;									// temperature
uint16_t alt_timer;								// altitude  measurement timer
uint16_t alt_interval = 300;        			// the interval we want to get an updated altitude in milliseconds

uint16_t test_timer;
uint16_t test_length = 13000;

uint16_t takeoff_test_timer;
uint16_t takeoff_test_increments = 2000;
bool takeoff_test_reverse = false;

long ultrasonic_duration, ultrasonic_distance;
int64_t baseline; 								// baseline pressure
int64_t default_elevation = 13.93;				// elevation of the location
double altitude,absolute_altitude,altitude_comp;// relative and absolute altitude
bool hold_altitude_status = false;				// is hold altitude selected?
float altitude_status;							// debug variable to determine what instrument is being used for altitude detection

int16_t	mFL,mFR,mBL,mBR;                     	// Our 4 motor variables


//////////////////////////////////////////////
///////////// UTILITY  FUNCTIONS  ////////////
//////////////////////////////////////////////

void blinkLED(byte targetPin, int numBlinks, int blinkRate) {
  for (int i=0; i < numBlinks; i++) {
    digitalWrite(targetPin, HIGH);   	  // sets the LED on
    delay(blinkRate);                     // waits for blinkRate milliseconds
    digitalWrite(targetPin, LOW);    	  // sets the LED off
    delay(blinkRate);
  }
}

void fatalError(String error_message){
	while(1){
		#ifdef DEBUG
		Serial.println("FATAL ERROR:");
		Serial.print(error_message);
		#endif
		digitalWrite(LED_RED,HIGH);
	}
}

void dmpDataReady() {
  mpuInterrupt = true;
}

double getPressure()
{
  char status;
  double T,P;

  status = pressure.startTemperature();
  if (status != 0){
    delay(status);
    status = pressure.getTemperature(T);
    if (status != 0){
      status = pressure.startPressure(BAROMETER_RESOLUTION);
      if (status != 0){
        delay(status);
        status = pressure.getPressure(P,T);
        if (status != 0){
        	temp = T;
        	return(P);
        } else return 0;}
      else return 0;}
    else return 0;}
  else return 0;
}

typedef struct {
  double q; //process noise covariance
  double r; //measurement noise covariance
  double x; //value
  double p; //estimation error covariance
  double k; //kalman gain
} kalman_state;


double kalman_init(double q, double r, double p, double initial_value){
	kalman_state result;
	result.q = q;
	result.r = r;
	result.p = p;
	result.x = initial_value;
	return result;
}

void
kalman_update(kalman_state* state, double measurement)
{
  //prediction update
  //omit x = x
  state->p = state->p + state->q;

  //measurement update
  state->k = state->p / (state->p + state->r);
  state->x = state->x + state->k * (measurement - state->x);
  state->p = (1 - state->k) * state->p;
}

//////////////////////////////////////////////
///////////// INIT  FUNCTIONS  ///////////////
//////////////////////////////////////////////

void initESCs(){
	// hardcoded, remove once we have the RC
	motor1.write(180);
	motor2.write(180);
	motor3.write(180);
	motor4.write(180);
	delay(DELAY_ARM);
	motor1.write(0);
	motor2.write(0);
	motor3.write(0);
	motor4.write(0);
	delay(DELAY_ARM);
}

void initMPU(){
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
//  mpu.setXGyroOffset(220); // 220
//  mpu.setYGyroOffset(-1057);  // 76 // -1057
//  mpu.setZGyroOffset(31); // -85 // 31
//  mpu.setZAccelOffset(1281); // 1281 // 1688 factory default for my test chip
  if(devStatus == 0){
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }


}

void initAltimeter(){
  if (pressure.begin()){
  } else {
	digitalWrite(LED_RED,HIGH);
	while(1); // Pause forever.
  }
  baseline = getPressure();

  pinMode(ULTRASONIC_TRIGGER_PIN,OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN,INPUT);
}

void initMotors(){
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN,OUTPUT);

  motor1.attach(MOTOR_FWD_L_PIN);
  motor2.attach(MOTOR_BCK_L_PIN);
  motor3.attach(MOTOR_FWD_R_PIN);
  motor4.attach(MOTOR_BCK_R_PIN);

  blinkLED(LED_GREEN,3,100);
  digitalWrite(LED_GREEN,HIGH);
}

void initPID(){
  Input_Pitch = 0;
  Input_Roll = 0;
  myPID_P.SetMode(REVERSE);
  myPID_R.SetMode(REVERSE);
  myPID_A.SetMode(REVERSE);

  myPID_P.SetOutputLimits(-100,100);
  myPID_R.SetOutputLimits(-100,100);
}

//void initLogging(){
//  // make sure that the default chip select pin is set to
//  // output, even if you don't use it:
//  pinMode(SD_TMP_PIN, OUTPUT);
//
//  // see if the card is present and can be initialized:
//  while (!SD.begin(SD_PIN)) {
//	Serial.println("Card failed, or not present");
//	// don't do anything more:
//	return;
//  }
//}

void motorStop(){
	analogWrite(MOTOR_FWD_L_PIN,0);
	analogWrite(MOTOR_FWD_R_PIN,0);
	analogWrite(MOTOR_BCK_L_PIN,0);
	analogWrite(MOTOR_BCK_R_PIN,0);
	digitalWrite(LED_RED,HIGH);
}

//////////////////////////////////////////////
///////////// UPDATE  FUNCTIONS  /////////////
//////////////////////////////////////////////

void updateControllerInput(){
	ch1 = pulseIn(RC1_PIN,HIGH,25000);
	ch2 = pulseIn(RC2_PIN,HIGH,25000);
	ch3 = pulseIn(RC3_PIN,HIGH,25000);
	blinkLED(LED_BLUE,1,1);	// with this we can visualise RC signals


	 //Input Signal trimming
//	for (int i=0; i<=8; i++) {
//		if (ch[i] <= RXLo){
//			ch[i] = RXLo;
//		}
//
//		if (ch[i] <= RXDeadHi && ch[i] >= RXDeadLo) {
//		ch[i] = RXMid;
//		}
//
//		if (ch[i] >= RXHi) {
//		 ch[i] = RXHi;
//		}
//	}
	// once we have the RC, uncomment this 
	//  Setpoint_Pitch = map(ch[0],RXLo,RXHi,MAX_PITCH*-1,MAX_PITCH);
	//  Setpoint_Roll = map(ch[1],RXLo,RXHi,MAX_ROLL*-1,MAX_ROLL);
	Setpoint_Pitch = 0;
	Setpoint_Roll = 0;
}

void updateYPR(){
    if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize) {
    	blinkLED(LED_RED,1,3);
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    blinkState = !blinkState;
    digitalWrite(LED_BLUE, blinkState);
  }
}

void updateAltimeter(){
//	if (altitude_comp>0){
//		kalman_init(altitude_comp)
//	}

	digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
	delayMicroseconds(2);
	digitalWrite(ULTRASONIC_TRIGGER_PIN, HIGH);
	delayMicroseconds(10);
	digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
	ultrasonic_duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH,ULTRASONIC_TIMEOUT);
	ultrasonic_distance = ((ultrasonic_duration/2) / 29.1);
	if (ultrasonic_distance<=ULTRASONIC_MAX_DISTANCE&&ultrasonic_distance!=0){
		// If we are within range on the ultrasonic sensor, use a complementary filter of ultrasonic sensor and barometric pressure
		altitude_comp = ultrasonic_distance*0.01;
		altitude_status = 1.0;

	} else {
		double P;
		double *temp_altitude;
		P = getPressure();
		temp_altitude = pressure.altitude(P,baseline);
		altitude_comp = kalman_update(*temp_altitude)


		if (altitude_comp <= 0.1){		// this might be a bad idea, since we dont know if we get lower than start height
			altitude_comp=0.0;
		}
		altitude_status = 2.0;
	}

	absolute_altitude=default_elevation+altitude_comp;

	if (hold_altitude_status){
		Setpoint_Altitude = altitude;

	}
}

//void updateAltimeter(){
//	double P;
//	P = getPressure();
//	altitude = pressure.altitude(P,baseline);
//
//	if (altitude <= 0.1){		// this might be a bad idea, since we dont know if we get lower than start height
//		altitude=0.0;
//	}
//	absolute_altitude=default_elevation+altitude;
//
//	if (hold_altitude_status){
//		Input_Altitude = altitude;
//	}
//}

void updatePID(){
  Input_Roll = ypr[2] * 180/M_PI;
  Input_Pitch = ypr[1] * 180/M_PI;

  // Compute PID values
  myPID_P.Compute();
  myPID_R.Compute();


	if (hold_altitude_status){
		Input_Altitude = ch3;
		myPID_A.Compute();
	}

  float posPitch = map(Output_Pitch,MAX_PITCH*-1,MAX_PITCH,0,100);
  float posRoll = map(Output_Roll,MAX_ROLL*-1,MAX_ROLL,0,100);
  float negPitch = map(Output_Pitch,MAX_PITCH*-1,MAX_PITCH,100,0);
  float negRoll = map(Output_Roll,MAX_ROLL*-1,MAX_ROLL,100,0);


  mFL=(posRoll+negPitch)*0.5;
  mFR=(posRoll+posPitch)*0.5;
  mBL=(negRoll+negPitch)*0.5;
  mBR=(negRoll+posPitch)*0.5;

  mFL = constrain(mFL,0,100);
  mFR = constrain(mFR,0,100);
  mBL = constrain(mBL,0,100);
  mBR = constrain(mBR,0,100);
}

void updateVelocity(){
	ch3=1640;	// once we have RC set this through the rc input
	ch3 = floor(ch3/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
	velocity = map(ch3, RXLo, RXHi, MINSPEED, MAXSPEED);
	if((velocity < MINSPEED) || (velocity > MAXSPEED)) velocity = velocityLast;
	  velocityLast = velocity;

	  mFL=mFL*(velocity*0.01);
	  mFR=mFR*(velocity*0.01);
	  mBL=mBL*(velocity*0.01);
	  mBR=mBR*(velocity*0.01);
}

void updateMotors(){
	motor1.write(mFL);
	motor2.write(mFR);
	motor3.write(mBL);
	motor4.write(mBR);
}

void updateDebugView(){
  int setpoint_p_debug = map(Setpoint_Pitch,MAX_PITCH*-1,MAX_PITCH,0,100);
  int setpoint_r_debug = map(Setpoint_Roll,MAX_ROLL*-1,MAX_ROLL,0,100);
  Serial.print(mFL,DEC);
  Serial.print(",");
  Serial.print(mFR,DEC);
  Serial.print(",");
  Serial.print(mBL,DEC);
  Serial.print(",");
  Serial.print(mBR,DEC);
  Serial.print(",");
  Serial.print(temp,DEC);
  Serial.print(",");
  Serial.print(setpoint_p_debug,DEC);
  Serial.print(",");
  Serial.print(setpoint_r_debug,DEC);
  Serial.print(",");
  Serial.print(PITCH_P_VAL);
  Serial.print(",");
  Serial.print(PITCH_I_VAL);
  Serial.print(",");
  Serial.print(PITCH_D_VAL);
  Serial.print(",");
  Serial.print(altitude_comp);
  Serial.print(",");
  Serial.print(absolute_altitude);
  Serial.print(",");
  Serial.print(freeMemory());
  Serial.print(",");
  Serial.print(altitude_status,DEC);
  Serial.print("-");
}

//void updateLog(){
//	String dataPitch = String((int)Input_Pitch, (unsigned char)DEC);
//	String dataRoll = String((int)Input_Roll, (unsigned char)DEC);
//	File mylog = SD.open("data.csv", O_WRITE | O_CREAT);
//	if (mylog) {
//		mylog.print(dataPitch);
//		mylog.print(",");
//		while ( mylog.read() != '\n' );
//		mylog.print(dataRoll);
//		mylog.print(",");
//		mylog.close();
//	}
//}

//////////////////////////////////////////////
///////////// TEST  FUNCTIONS  ///////////////
//////////////////////////////////////////////

void test_takeoff(){

	if ((unsigned long)(millis()-takeoff_test_timer)>=takeoff_test_increments){
		takeoff_test_timer = millis();
		if (mFL>=50||mFR>=50||mBL>=50||mBR>=50){
			takeoff_test_reverse = true;
		}
	}

	if(takeoff_test_reverse){
		velocity-=0.1;
	} else {
		velocity+=0.1;
	}


	mFL=(mFL*velocity);
	mFR=(mFR*velocity);
	mBL=(mBL*velocity);
	mBR=(mBR*velocity);



}

void test_propeller_idle(){
	if ((unsigned long)millis()>=test_length){
		mFL=0;
		mFR=0;
		mBL=0;
		mBR=0;
	} else {
		mFL=4;
		mFR=4;
		mBL=4;
		mBR=4;
	}
}


//////////////////////////////////////////////
/////////////// MAIN  FUNCTIONS  /////////////
//////////////////////////////////////////////

void setup() {
	#ifdef DEBUG
	Serial.begin(115200);
	Serial.flush();
	#endif

	initAltimeter();
	initMPU();
	initMotors();
	initESCs();
	initPID();
}

void loop() {
//	updateControllerInput();
  if ((unsigned long)(millis()-alt_timer)>=alt_interval){
	  alt_timer = millis();
	  updateAltimeter();
  }
	updateYPR();
	updatePID();
	updateVelocity();
	updateMotors();

	#ifdef DEBUG
	updateDebugView();
	//  updateLog();
	#endif
}


