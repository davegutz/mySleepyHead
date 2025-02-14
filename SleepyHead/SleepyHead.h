//
// MIT License
//
// Copyright (C) 2025 - Dave Gutz
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

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
#include <HardwareSerial.h>
#include "Sensors.h"
#include "command.h"
#include "ProcessInput.h"
String serial_str;
String unit;
boolean string_cpt = false;
time_t time_initial = ARBITRARY_TIME;
unsigned long long millis_flip = millis(); // Timekeeping
unsigned long long last_sync = millis();   // Timekeeping
int debug = 0;
