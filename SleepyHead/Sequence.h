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

#pragma once
#include "constants.h"
#ifdef USE_ARDUINO
  #include <Arduino.h> //needed for Serial.println
#else
  #include "application.h"  // Particle
#endif
#include "src/Sync/Sync.h"

// Timekeeping class
class Sequence
{
public:
    Sequence(void): chitchat_(false), control_(false), elapsed_(0ULL), last_sync_(true), now_ms_((unsigned long long) millis()),
      publishing_(false), read_eye_(false), read_head_(false), time_start_(millis())      
    {
        BlinkSync = new Sync(BLINK_DELAY);
        ControlSync = new Sync(CONTROL_DELAY);
        Plotting = new Sync(PUBLISH_DELAY);
        ReadEye = new Sync(EYE_DELAY);
        ReadHead = new Sync(HEAD_DELAY);
        Talk = new Sync(TALK_DELAY);
    }
    void calculate(unsigned long long *last_sync, unsigned long long *millis_flip, const boolean reset);
    void sync_time(unsigned long long *last_sync, unsigned long long *millis_flip);
    Sync *BlinkSync;
    Sync *ControlSync;
    Sync *Plotting;
    Sync *ReadEye;
    Sync *ReadHead;
    Sync *Talk;
    boolean chitchat() { return chitchat_; }
    boolean control() { return control_; }
    float elapsed() { return float(elapsed_)/1000.; }
    boolean last_sync() { return last_sync_; }
    unsigned long long now_ms() { return now_ms_; }
    boolean publishing() { return publishing_; }
    boolean read_eye() { return read_eye_; }
    boolean read_head() { return read_head_; }
    unsigned long long time_start() { return time_start_; }
protected:
    boolean chitchat_;
    boolean control_;
    unsigned long long elapsed_;
    boolean last_sync_;
    unsigned long long now_ms_;
    boolean publishing_;
    boolean read_eye_;
    boolean read_head_;
    unsigned long long time_start_;
};
