//
// MIT License
//
// Copyright (C) 2025 - Dave Gutz
//
// Permission is hereby granted, free of charge, to any person obtaining a copy// of this software and associated documentation files (the "Software"), to deal
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
#include "Wag.h"
#include "math.h"

extern int debug;


// class Wag
// constructors
Wag::Wag()
    : reset_(false), T_(0), in_1_(0), in_2_(0), output_(false), time_set_(0), time_reset_1_(0), time_reset_2_(0), time_reset_3_(0),
    state_1_(false), state_2_(false)
{}
Wag::Wag(const boolean init, const float nom_dt, const float set_all, const float hold_1, const float hold_2, const float hold_3)
    : reset_(false), T_(nom_dt), in_1_(false), in_2_(false), output_(init), time_set_(set_all), time_reset_1_(hold_1), time_reset_2_(hold_2),
     time_reset_3_(hold_3), state_1_(false), state_2_(false)
{
    timer_1 = new TFDelay(init, time_set_, time_reset_1_, T_);
    timer_2 = new TFDelay(init, time_set_, time_reset_2_, T_);
    timer_3 = new TFDelay(init, time_set_, time_reset_3_, T_);
}
Wag::~Wag() {}

// operators
// functions

void Wag::repr()
{
    Serial.print("Wag:  reset:"); Serial.print(reset_);
    Serial.print("\tT:"); Serial.print(T_);
    Serial.print("\ttime_set:"); Serial.print(time_set_);
    Serial.print("\ttime_reset_1:"); Serial.print(time_reset_1_);
    Serial.print("\ttime_reset_2:"); Serial.print(time_reset_2_);
    Serial.print("\ttime_reset_3:"); Serial.print(time_reset_3_);
    Serial.print("\tinputx2:"); Serial.print(in_1_);
    Serial.print("\tinputx1:"); Serial.print(in_2_);
    Serial.print("\tstate_1:"); Serial.print(state_1_);
    Serial.print("\tstate_2:"); Serial.print(state_2_);
    Serial.print("\toutput:"); Serial.print(output_);
    Serial.println("");
  }

boolean Wag::calculate(const boolean reset, const float T, const float in_1, const float in_2)
{
    reset_ = reset;
    T_ = T;
    in_1_ = in_1;
    in_2_ = in_2;
    state_1_ = timer_1->calculate( in_1_, time_set_, time_reset_1_, T_, reset_);
    state_2_ = timer_2->calculate( (state_1_ && in_2_), time_set_, time_reset_2_, T_, reset_);
    output_ = timer_3->calculate(  (state_2_ && in_1_), time_set_, time_reset_3_, T_, reset_);
    return ( output_ );
}
