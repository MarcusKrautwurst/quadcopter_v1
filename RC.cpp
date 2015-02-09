#if defined(ARDUINO) && ARDUINO >= 100
      #include "Arduino.h"
    #else
      #include "WProgram.h"
    #endif


#include "RC.h"

RC_Channel::RC_Channel()
{
}


int RC_Channel::setPin(int inputPin) {
	lastReadTime = micros();
	if(globalPinCounter<6)
	{
		pin[globalPinCounter++] = inputPin;
		 pinMode(inputPin, INPUT);
		return (globalPinCounter - 1);
	}
	else
		return -1;
}


float RC_Channel::getValue(int pinIndex)
{
	return pulseIn(pin[pinIndex], HIGH, 25000);
}
