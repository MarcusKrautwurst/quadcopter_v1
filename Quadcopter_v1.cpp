#include "Quadcopter_v1.h"
#include "config.h"

// IMU config
MPU6050 mpu(I2C_ADRESS_MPU);
SFE_BMP180 pressure;

double altittude_baseline; // baseline pressure

double Setpoint_Pitch, Input_Pitch, Output_Pitch;
double Setpoint_Roll, Input_Roll, Output_Roll;

PID myPID_P(&Input_Pitch, &Output_Pitch, &Setpoint_Pitch,PITCH_P_VAL,PITCH_I_VAL,PITCH_D_VAL, DIRECT);
PID myPID_R(&Input_Roll, &Output_Roll, &Setpoint_Roll,ROLL_P_VAL,ROLL_I_VAL,ROLL_D_VAL, DIRECT);

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

int16_t boot_delay = 2000;                  	// delay for arming the motors : default = 2000
int16_t temp;									// temperature
uint16_t temp_timer;							// temperature measurement timer
uint16_t temp_interval = 10000;        			// the interval we want to get an updated temperature in milliseconds
int16_t rel_alltitude;

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

double getPressure() {
  char status;
  double T,P;
  status = pressure.startTemperature();
  if (status != 0) {
    delay(status);
    status = pressure.getTemperature(T);
    if (status != 0) {
      status = pressure.startPressure(3);
      if (status != 0) {
        delay(status);
        status = pressure.getPressure(P,T);
        if (status != 0) {
          return(P);
        }
      }
    }
  }
}

//////////////////////////////////////////////
///////////// INIT  FUNCTIONS  ///////////////
//////////////////////////////////////////////


void initMPU(){
  Wire.begin();
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

void initBarometer(){
  if (!pressure.begin()){
    Serial.println("BMP180 init fail");
    //while(1); at the moment we simply ignore this, because we dont have the device yet
  }
  altittude_baseline = getPressure();
  Serial.println("baseline pressure: ");
  Serial.print(altittude_baseline);
}

void initMotors(){
  pinMode(LED_PIN, OUTPUT);
  pinMode(MOTOR_TEST_PIN, OUTPUT);
  pinMode(MOTOR_FWD_L_PIN, OUTPUT);
  pinMode(MOTOR_FWD_R_PIN, OUTPUT);
  pinMode(MOTOR_BCK_L_PIN, OUTPUT);
  pinMode(MOTOR_BCK_R_PIN, OUTPUT);
  delay(boot_delay*0.5);
  blinkLED(LED_PIN,3,100);
  delay(boot_delay*0.5);
}

void initPID(){
  Input_Pitch = 0;
  Input_Roll = 0;
  myPID_P.SetMode(AUTOMATIC);
  myPID_R.SetMode(AUTOMATIC);
  myPID_P.SetOutputLimits(MAX_PITCH*-1,MAX_PITCH);
  myPID_R.SetOutputLimits(MAX_ROLL*-1,MAX_ROLL);
}

void initLogging(){
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(SD_TMP_PIN, OUTPUT);

  // see if the card is present and can be initialized:
  while (!SD.begin(SD_PIN)) {
	Serial.println("Card failed, or not present");
	// don't do anything more:
	return;
  }
}


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

	//  Setpoint_Pitch = map(ch[0],0,1023,MAX_PITCH*-1,MAX_PITCH);
	//  Setpoint_Roll = map(ch[1],0,1023,MAX_ROLL*-1,MAX_ROLL);
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
//    digitalWrite(LED_PIN, blinkState);
  }
}

void updateAltittude(){
  double P;
  P = getPressure();
  rel_alltitude = pressure.altitude(P,altittude_baseline);
}

void updatePID(){
  Input_Roll = ypr[2] * 180/M_PI;
  Input_Pitch = ypr[1] * 180/M_PI;

  // Compute PID values
  myPID_P.Compute();
  myPID_R.Compute();

  float posPitch = map(Output_Pitch,MAX_PITCH*-1,MAX_PITCH,0,255);
  float posRoll = map(Output_Roll,MAX_ROLL*-1,MAX_ROLL,0,255);
  float negPitch = map(Output_Pitch,MAX_PITCH*-1,MAX_PITCH,255,0);
  float negRoll = map(Output_Roll,MAX_ROLL*-1,MAX_ROLL,255,0);


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
  analogWrite(MOTOR_FWD_L_PIN,mFL);
  analogWrite(MOTOR_FWD_R_PIN,mFR);
  analogWrite(MOTOR_BCK_L_PIN,mBL);
  analogWrite(MOTOR_BCK_R_PIN,mBR);
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
  Serial.print(PITCH_P_VAL,DEC);
  Serial.print(",");
  Serial.print(PITCH_I_VAL,DEC);
  Serial.print(",");
  Serial.print(PITCH_D_VAL,DEC);
  Serial.print("-");
//  Serial.print("\n");	// only for readability in serial monitor, turn this off for running the external debug application
}

void updateLog(){
	String dataPitch = String((int)Input_Pitch, (unsigned char)DEC);
	String dataRoll = String((int)Input_Roll, (unsigned char)DEC);
	String dataTemp = String((int)temp, (unsigned char)DEC);
	File logPitch = SD.open("pitch.csv", O_WRITE | O_CREAT);
	File logRoll = SD.open("roll.csv", O_WRITE | O_CREAT);
	if (logPitch) {
		logPitch.print(dataPitch);
		logPitch.print(",");
		logPitch.close();
	}

	if (logRoll) {
		logRoll.print(dataRoll);
		logRoll.print(",");
		logRoll.close();
	}
}

//void updateLog(){
//	mylog.addEntry(0,Input_Pitch);
//	mylog.addEntry(1,Input_Roll);
//	mylog.writeMasterFile();
//}


void updateTemperature(){
  Wire.requestFrom(I2C_ADRESS_MPU,14,true);
  Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  temp = (Wire.read()<<8|Wire.read())/340.00+36.53;  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  temp = (temp-32)*0.56;
}

//////////////////////////////////////////////
/////////////// MAIN  FUNCTIONS  /////////////
//////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  initMPU();
  initMotors();
  initPID();
  initLogging();
}

void loop() {
  updateYPR();
  updateAltittude();
  updateControllerInput();
  updatePID();
  updateLog();
  if ((unsigned long)(millis()-temp_timer)>=temp_interval){
	  temp_timer = millis();
	  updateTemperature();
  }
  updateMotors();
  updateLog();
//  updateDebugView();
}


