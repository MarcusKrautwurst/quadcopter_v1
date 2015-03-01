#include "Quadcopter_v1.h"
#include "config.h"


MPU6050 mpu(I2C_ADRESS_MPU);	// IMU
SFE_BMP180 pressure;			// Barometric sensor

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;


double Setpoint_Pitch, Input_Pitch, Output_Pitch;
double Setpoint_Roll, Input_Roll, Output_Roll;
double Setpoint_Altitude, Input_Altitude, Output_Altitude;

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

int ch[4];  //Rx channels

uint16_t boot_delay = 2000;                  	// delay for arming the motors : default = 2000
uint16_t acceleration;							// this is the speed multiplier
float velocity;
uint16_t temp;									// temperature

uint16_t alt_timer;								// altitude  measurement timer
uint16_t alt_interval = 500;        			// the interval we want to get an updated altitude in milliseconds

uint16_t takeoff_test_timer;
uint16_t takeoff_test_increments = 2000;
bool takeoff_test_reverse = false;

double baseline; 								// baseline pressure
double default_elevation = 13.93;				// elevation of the location
double altitude;								// relative altitude
double absolute_altitude;						// absolute altitude
bool holdAltitude = false;						// is hold altitude selected?


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
      status = pressure.startPressure(3);
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

//////////////////////////////////////////////
///////////// INIT  FUNCTIONS  ///////////////
//////////////////////////////////////////////

void initESCs(){
	// hardcoded, remove once we have the RC
	motor1.write(180);
	motor2.write(180);
	motor3.write(180);
	motor4.write(180);
	delay(2000);
	motor1.write(0);
	motor2.write(0);
	motor3.write(0);
	motor4.write(0);
	delay(2000);
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
	while(1); // Pause forever.
  }
  baseline = getPressure();
}

void initMotors(){
  pinMode(LED_PIN, OUTPUT);

  motor1.attach(MOTOR_FWD_L_PIN);
  motor2.attach(MOTOR_BCK_L_PIN);
  motor3.attach(MOTOR_FWD_R_PIN);
  motor4.attach(MOTOR_BCK_R_PIN);

  delay(boot_delay*0.5);
  blinkLED(LED_PIN,3,100);
  delay(boot_delay*0.5);
}

void initPID(){
  Input_Pitch = 0;
  Input_Roll = 0;
  myPID_P.SetMode(REVERSE);
  myPID_R.SetMode(REVERSE);
  myPID_A.SetMode(REVERSE);

  myPID_P.SetOutputLimits(MAX_PITCH*-1,MAX_PITCH);
  myPID_R.SetOutputLimits(MAX_ROLL*-1,MAX_ROLL);
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
}

//////////////////////////////////////////////
///////////// UPDATE  FUNCTIONS  /////////////
//////////////////////////////////////////////

void updateControllerInput(){
	ch[0] = pulseIn(RC1_PIN,HIGH,25000);
	ch[1] = pulseIn(RC2_PIN,HIGH,25000);
	ch[2] = pulseIn(RC3_PIN,HIGH,25000);
	blinkLED(LED_PIN,1,1);	// with this we can visualise RC signals

	 //Input Signal trimming
	for (int i=0; i<=8; i++) {
		if (ch[i] <= RXLo){
			ch[i] = RXLo;
		}

		if (ch[i] <= RXDeadHi && ch[i] >= RXDeadLo) {
		ch[i] = RXMid;
		}

		if (ch[i] >= RXHi) {
		 ch[i] = RXHi;
		}
	}
	// once we have the RC, uncomment this 
	//  Setpoint_Pitch = map(ch[0],RXLo,RXHi,MAX_PITCH*-1,MAX_PITCH);
	//  Setpoint_Roll = map(ch[1],RXLo,RXHi,MAX_ROLL*-1,MAX_ROLL);
	Setpoint_Pitch = 0;
	Setpoint_Roll = 0;
}

void updateYPR(){
    if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize) {}
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
    digitalWrite(LED_PIN, blinkState);
  }
}

void updateAltimeter(){
	double P;
	P = getPressure();
	altitude = pressure.altitude(P,baseline);
	
	if (altitude <= 0.1){		// this might be a bad idea, since we dont know if we get lower than start height
		altitude=0.0;
	}
	absolute_altitude=default_elevation+altitude;

	if (holdAltitude){
		Input_Altitude = altitude;
	}
}

void updatePID(){
  Input_Roll = ypr[2] * 180/M_PI;
  Input_Pitch = ypr[1] * 180/M_PI;

  // Compute PID values
  myPID_P.Compute();
  myPID_R.Compute();


	if (holdAltitude){
		myPID_A.Compute();
	}

  float posPitch = map(Output_Pitch,MAX_PITCH*-1,MAX_PITCH,MINSPEED,MAXSPEED);
  float posRoll = map(Output_Roll,MAX_ROLL*-1,MAX_ROLL,MINSPEED,MAXSPEED);
  float negPitch = map(Output_Pitch,MAX_PITCH*-1,MAX_PITCH,MAXSPEED,MINSPEED);
  float negRoll = map(Output_Roll,MAX_ROLL*-1,MAX_ROLL,MAXSPEED,MINSPEED);


  mFL=(posRoll+negPitch)*0.5;
  mFR=(posRoll+posPitch)*0.5;
  mBL=(negRoll+negPitch)*0.5;
  mBR=(negRoll+posPitch)*0.5;

  mFL = constrain(mFL,MINSPEED,MAXSPEED);
  mFR = constrain(mFR,MINSPEED,MAXSPEED);
  mBL = constrain(mBL,MINSPEED,MAXSPEED);
  mBR = constrain(mBR,MINSPEED,MAXSPEED);
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
  Serial.print(altitude);
  Serial.print(",");
  Serial.print(absolute_altitude);
  Serial.print(",");
  Serial.print(freeMemory());
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


//////////////////////////////////////////////
/////////////// MAIN  FUNCTIONS  /////////////
//////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  initAltimeter();
  initMPU();
  initMotors();
  initESCs();
  initPID();
//  initLogging();
}

void loop() {
	while(!mpuInterrupt && fifoCount < packetSize){

	/* Do nothing while MPU is not working
	 * This should be a VERY short period
	 */

	}
//	updateControllerInput();
  if ((unsigned long)(millis()-alt_timer)>=alt_interval){
	  alt_timer = millis();
	  updateAltimeter();
  }
	updateYPR();
	updatePID();
//	test_takeoff();
	updateMotors();
	//  updateLog();
	updateDebugView();
}


