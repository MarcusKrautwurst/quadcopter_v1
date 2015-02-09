#ifndef _RC_H_
#define _RC_H_
#if defined(ARDUINO) && ARDUINO >= 100
      #include "Arduino.h"
    #else
      #include "WProgram.h"
    #endif


class RC_Channel
{
  public:

	RC_Channel();
	int setPin(int pin);
	float getValue(int pinIndex);


  private:
	int globalPinCounter;
	long int lastReadTime;
	int pin[7];
	int lastValue;

};

#endif
