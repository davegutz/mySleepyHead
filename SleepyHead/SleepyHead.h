// Include libraries
#pragma once
#include "constants.h"
#ifdef USE_ARDUINO
  #include <Arduino.h> //needed for Serial.println
#else
  #include "application.h"  // Particle
#endif
#include "src/Sync/Sync.h"
#include "Sequence.h"
#include "src/Filters/myFilters.h"
#include "src/Time/TimeLib.h"
#include "src/Tones/Tones.h"  // depends on some things above
#include <HardwareSerial.h>
#include "Sensors.h"
#include "command.h"
#include "src/Tones/Tones.h"
#include "ProcessInput.h"
String serial_str;
String unit;
boolean string_cpt = false;
time_t time_initial = ARBITRARY_TIME;
unsigned long long millis_flip = millis(); // Timekeeping
unsigned long long last_sync = millis();   // Timekeeping
int debug = 0;
