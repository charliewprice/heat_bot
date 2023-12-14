/*
 * 10/18/2023 go live in Farrowing Room #1, with no-data
 *
 */
#include <Arduino.h>
#include <ErriezSerialTerminal.h>
#include <DS18B20.h>
#include <Wire.h>
#include "HeatPadControl.h"
#include "Eeprom.h"

//#define SERIAL_DEBUG
//#define BARE_METAL_BUILD

#define _EEPROMUPDATE_MILLIS 	30000
boolean updateEeprom;
Eeprom eeprom;

#define _PROCESS_TIME_MILLIS 1000
long upTimeSecs;
long lastProcessMillis;
long heatingTimeSecs;

#define _DEVICE_PROFILE 41

DS18B20 ds(A0);

HeatPadControl heatPadController;

struct moterecord {
	float batteryvoltage;
    float temperature;
    uint8_t  lastackreceived;
};

struct ctl_parms {
	int16_t channel;
	int16_t zone;
	double setPoint;
	double idleband;
};

typedef struct ctl_parms Config;
Config cfg;

void bareMetal(byte* b) {
  Serial.println("Bare Metal");
  Config cfg;

  cfg.channel = 63;
  cfg.zone = 100;
  cfg.setPoint = 100.0;
  cfg.idleband = 6.0;

  memcpy(b, &cfg, sizeof(cfg));
}

void initialize(byte* b) {
  Serial.println("initializing...");
  memcpy(&cfg, b, sizeof(cfg));
}

#define _WATCHDOG_DONE 12

char newlineChar = '\n';
char delimiterChar = ' ';
SerialTerminal term(newlineChar, delimiterChar);

void sitUbu()
{
  #if not defined(_TEST_WATCHDOG)
  digitalWrite(_WATCHDOG_DONE, HIGH);				// rising edge pulse on DONE to keep the watchdog happy!
  digitalWrite(_WATCHDOG_DONE, LOW);
  #endif
}

float getTemperature() {
	float tempF;
	while (ds.selectNext()) {
      //Serial.print("DS18B20 ");
	  tempF = ds.getTempF();
	}
	return tempF;
}

void unknownCommand(const char *command) {
    Serial1.print(F("Unknown command: "));
    Serial1.println(command);
}

void cmdGetInfo() {
	 char durations[64];
	 long seconds;
	 double heat_pct;
     seconds = upTimeSecs;
     heat_pct=(100.0*heatingTimeSecs)/upTimeSecs;
     Serial1.println(F("\n~~Heat Pad Status~~~~~~~~~~~~~~~~"));
     Serial1.print("upTime   : ");
	 int hrs = seconds/3600;                                                        //Number of seconds in an hour
	 int mins = (seconds-hrs*3600)/60;                                     //Remove the number of hours and calculate the minutes.
	 int secs = seconds-hrs*3600-mins*60;                                //Remove the number of hours and minutes, leaving only seconds.
	 sprintf(durations, "%ih %im %is  [%i%%] ON", hrs, mins, secs, (int)heat_pct);
     Serial1.println(durations);
     Serial1.print("Current temperature "); Serial1.print(heatPadController.getLastTemp()); Serial1.println(" F");
     if (heatPadController.getStatus()== HEAT_ON)
       Serial1.println("The heat pads are ON now.");
     else
       Serial1.println("The heat pads are OFF now.");
     Serial1.println("==================");
}


#define MAX_TEMP 120.0
#define MIN_TEMP 75.0
#define MIN_IDLEBAND 4.0
#define MAX_IDLEBAND 16.0

void cmdSet() {
  float setTemp = atof(term.getNext());
  float idleBand = atof(term.getNext());
  if ((setTemp>=MIN_TEMP)
		  && (setTemp<=MAX_TEMP)
		  && (idleBand>=MIN_IDLEBAND)
		  && (idleBand<=MAX_IDLEBAND)) {
    cfg.setPoint = setTemp;
    cfg.idleband = idleBand;
    heatPadController.set(cfg.setPoint, cfg.idleband);
    Serial1.println("Temperature is set.");
  } else {
	Serial1.println("Temperature could not be set.");
  }
}

void cmdHelp() {
	Serial1.println(F("\n~~HeatBot Commands~~~~~~~~~~~~"));
	Serial1.println(F("  help     Print usage info"));
	Serial1.println(F("  set      <center> <idleband>"));
	Serial1.println(F("  config   Show configuration"));
	Serial1.println(F("  save     Save configuration"));
	Serial1.println(F("  reset    Reset controller"));
	Serial1.println(F("  info     Current status"));
	Serial1.println("==================");
}

void cmdSave() {
	uint8_t size = sizeof(ctl_parms);
	byte b[size];
	memcpy(b, &cfg, size);
	eeprom.saveDeviceConfig(b, EEPROM_CONFIG_ADDRESS,size);
	Serial1.println("Configuration saved.");
}

void cmdReboot() {
	Serial1.println(F("Resetting the hardware..."));
	delay(500);
	NVIC_SystemReset();
}

void cmdShow() {
	float tempF = getTemperature();
	Serial1.println(F("\n~~Heat Pad Configuration~~~~~~~~~"));
	Serial1.print("Set point: "); Serial1.print(cfg.setPoint); Serial1.println(" F");
	Serial1.print("Idleband : "); Serial1.print(cfg.idleband);  Serial1.println(" F");
	Serial1.print("Channel  : "); Serial1.println(cfg.channel);
	Serial1.print("Zone     : "); Serial1.println(cfg.zone);
    Serial1.println("-----------------");
	Serial1.print("On  below "); Serial1.print(cfg.setPoint-cfg.idleband/2); Serial1.println(" F");
	Serial1.print("Off above "); Serial1.print(cfg.setPoint+cfg.idleband/2); Serial1.println(" F");
	Serial1.print("Current T "); Serial1.print(tempF); Serial1.println(" F");
	Serial1.println("==================");
}

void setup() {
	pinMode(_WATCHDOG_DONE, OUTPUT);
	Serial1.begin(9600);
    Serial.begin(115200);

#if defined(SERIAL_DEBUG)
    Serial1.print("waiting for serial debugger");
	uint8_t n = 0;
	while (!Serial) {
	  delay(1000);
	  n += 1;
	  Serial1.print(".");
	  if (n>=10)
	    break;
	}
	Serial1.println("");
#endif

	Wire.begin();
	Wire.setClock(100000UL);

	uint8_t size = sizeof(ctl_parms);
	byte b[size];
#if defined(BARE_METAL_BUILD)
    bareMetal(b);
    //Serial.println("Bare metal is configured.");
    eeprom.saveDeviceConfig(b, EEPROM_CONFIG_ADDRESS, size);
    //Serial.println("Bare metal config saved to EEPROM");
#endif
    eeprom.loadDeviceConfig(b, EEPROM_CONFIG_ADDRESS, size);
    initialize(b);
    heatPadController.set(cfg.setPoint, cfg.idleband);
    //Serial.println("Started");

	term.addCommand("help", cmdHelp);
	term.addCommand("set", cmdSet);
	term.addCommand("save", cmdSave);
	term.addCommand("show", cmdShow);
	term.addCommand("reset", cmdReboot);
	term.addCommand("info", cmdGetInfo);

	cmdHelp();
	lastProcessMillis = - _PROCESS_TIME_MILLIS;
}

void loop() {
	sitUbu();
	term.readSerial();
	if ( (millis() - lastProcessMillis) > _PROCESS_TIME_MILLIS ) {
	  lastProcessMillis = millis();
      float tempF = getTemperature();
      heatPadController.poll(tempF);
      upTimeSecs +=  _PROCESS_TIME_MILLIS/1000;
      if  (heatPadController.getStatus()== HEAT_ON)
        heatingTimeSecs +=  _PROCESS_TIME_MILLIS/1000;
	}
}
