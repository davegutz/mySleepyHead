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

/***************************************************
  A class that detects a head wag
 ****************************************************/

#pragma once

#define USE_ARDUINO

#ifdef USE_ARDUINO
  #include <Arduino.h> //needed for Serial.println
#else
  #include "application.h"
#endif
#include "myFilters.h"

class Wag
{
public:
  Wag();
  Wag(const boolean init, const float nom_dt, const float set_all, const float hold_1, const float hold_2, const float hold_3);
  ~Wag();
  // operators
  // functions
  bool calculate(const boolean reset, const float T, const float in_1, const float in_2);
  void repr();
protected:
  bool reset_;
  float T_;
  float in_1_;
  float in_2_;
  boolean output_;
  float time_set_;
  float time_reset_1_;
  float time_reset_2_;
  float time_reset_3_;
  TFDelay *timer_1;
  TFDelay *timer_2;
  TFDelay *timer_3;
  boolean state_1_;
  boolean state_2_;
};
