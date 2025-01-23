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

#include "Sequence.h"
#include "src/Time/TimeLib.h"

// Synchronize
void Sequence::calculate(unsigned long long *last_sync, unsigned long long *millis_flip)
{
    now_ms_ = (unsigned long long) millis();
    if ( now_ms_ - *last_sync > ONE_DAY_MILLIS || reset_ )  sync_time(last_sync, millis_flip); 
    read_eye_ = ReadEye->update(millis(), reset_);
    read_head_ = ReadHead->update(millis(), reset_);
    chitchat_ = Talk->update(millis(), reset_);
    elapsed_ = ReadHead->now() - time_start_;
    control_ = ControlSync->update(millis(), reset_);
    blink_ = BlinkSync->update(millis(), reset_);
    active_ = ActiveSync->update(millis(), reset_);
    publishing_ = Plotting->update(millis(), reset_);
    reset_ = false;
}  

// Time synchro so printed decimal times align with hms rolls
void Sequence::sync_time(unsigned long long *last_sync, unsigned long long *millis_flip)
{
  *last_sync = millis();

  // Refresh millis() at turn of Time.now
  int count = 0;
  long time_begin = now();  // Seconds since start of epoch
  while ( now()==time_begin && ++count<1100 )  // Time.now() truncates to seconds
  {
    delay(1);
    *millis_flip = millis()%1000;
  }
}

