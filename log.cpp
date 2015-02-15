#if defined(ARDUINO) && ARDUINO >= 100
      #include "Arduino.h"
    #else
      #include "WProgram.h"
    #endif


#include "Log.h"


Log::Log(int sd_chipselect_pin)
{
	path_logfile  = "log.txt";
	fields[0] = "Pitch";
	fields[1] = "Roll";
	pinMode(sd_chipselect_pin, OUTPUT);
	// see if the card is present and can be initialized:
	while (!SD.begin(sd_chipselect_pin)) {
		Serial.println("Card failed, or not present");
		return;
	}

	// remove old log file
	SD.remove(path_logfile);
}

void Log::writeMasterFile(){
	File myMasterfile = SD.open(path_logfile, FILE_WRITE);
	if (myMasterfile) {
		// add header to master
		myMasterfile.print(*fields[0],*fields[1]);

		// write PITCH
		File myLogfile_Pitch = SD.open(fields[0], FILE_WRITE);
		while (myLogfile_Pitch.available()) {
			myMasterfile.write(myLogfile_Pitch.read());
		    }

		// write ROLL
		File myLogfile_Roll = SD.open(fields[1], FILE_WRITE);
		while (myLogfile_Roll.available()) {
			myMasterfile.write(myLogfile_Roll.read());
		    }
		myMasterfile.close();
		SD.remove(fields[0]);
		SD.remove(fields[1]);
	}
}

int Log::addEntry(int field,int val) {
	String myVal = String((int)val, (unsigned char)DEC);
	File myLogfile = SD.open(fields[field], FILE_WRITE);
	if (myLogfile) {
		myLogfile.print(myVal);
		myLogfile.print(",");
		myLogfile.close();
		return true;
	}
	else {
		return false;
	}

}


	//RC_Channel.fields



