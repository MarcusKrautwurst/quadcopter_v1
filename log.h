#ifndef _LOG_H_
#define _LOG_H_
#if defined(ARDUINO) && ARDUINO >= 100
      #include "Arduino.h"
    #else
      #include "WProgram.h"
    #endif

#include "SD.h"


class Log
{
  public:

	Log(int sd_chipselect_pin);
	int addEntry(int field,int val);
	void clear();
	void writeMasterFile();

  private:

	char*fields[2];
	char* path_logfile;
};

#endif
