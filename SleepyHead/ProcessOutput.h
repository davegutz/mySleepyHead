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
#include "SleepyHead.h"
#include "constants.h"

extern CommandPars cp;

// Class to manage and coordinate multiple output devices
// Duty cycle in increments of 10 0 - 90 for 10 x 0.1 seconds = 1 second cycle
// Capable of changing tones on the fly (results in ugly repeated code)
class ProcessOutput
{
public:
    ProcessOutput(const unsigned int pin, boolean *token, String name): pin_(pin),have_tokens_(false), duty_count_(0),
        token_(token), name_(name) {};
protected:
    int pin_;
    boolean have_tokens_;
    uint8_t duty_count_;
    boolean *token_;
    String name_;
};


// Tone output
class ProcessToneOutput : public ProcessOutput
{
public:
    ProcessToneOutput(const unsigned int pin,  boolean *token, String name) : ProcessOutput(pin, token, name) {};
    void turn_off();
    void write_duty(const int freq, const uint8_t duty);
};


// Tone off
void ProcessToneOutput::turn_off()
{

    // Wrap.  Allow calling procedure to decide how to renew cycling by giving up tokens each cycle
    duty_count_ = 0;
    *token_ = false;
    if ( have_tokens_ )
    {
        have_tokens_ = false;
        // Serial.print("released "); Serial.println(name_);
    }
    noTone(pin_);

}

// Tone duty cycle control.  Set duty = 0 to turn everything off for sure.
void ProcessToneOutput::write_duty(const int freq, const uint8_t duty)
{

  // Wrap.  Allow calling procedure to decide how to renew cycling by giving up tokens each cycle
  if ( duty_count_ > 90 || duty == 0 )
  {
    duty_count_ = 0;
    *token_ = false;
    have_tokens_ = false;
    noTone(pin_);
    // if ( duty != 0 )
    // {
    //     Serial.print("released "); Serial.println(name_);
    // }
}

  // Grab tokens and initialize duty
  if ( !*token_  && duty != 0 )
  {
    *token_ = true;
    have_tokens_ = true;
    duty_count_ = 0;
    noTone(pin_);
    // Serial.print("grabbed "); Serial.println(name_);
}

  // Play
  if ( have_tokens_ )
  {
    if ( duty_count_ < duty )
      tone(pin_, freq);
    else
      noTone(pin_);
    duty_count_ += 10;
  }

}


// Pin output
class ProcessPinOutput : public ProcessOutput
{
public:
    ProcessPinOutput(const unsigned int pin, boolean *token, String name) : ProcessOutput(pin, token, name) {};
    void turn_off();
    void write_duty(const uint8_t duty);
};

// Pin off
void ProcessPinOutput::turn_off()
{

    // Wrap.  Allow calling procedure to decide how to renew cycling by giving up tokens each cycle
    duty_count_ = 0;
    *token_ = false;
    if ( have_tokens_ )
    {
        have_tokens_ = false;
        // Serial.print("released "); Serial.println(name_);
    }
    digitalWrite(pin_, LOW);

}

// Pin duty cycle control.  Set duty = 0 to turn everything off for sure.
void ProcessPinOutput::write_duty(const uint8_t duty)
{

  // Wrap.  Allow calling procedure to decide how to renew cycling by giving up tokens each cycle
  if ( duty_count_ > 90 || duty == 0 )
  {
    duty_count_ = 0;
    *token_ = false;
    have_tokens_ = false;
    digitalWrite(pin_, LOW);
    // if ( duty != 0 )
    // {
        // Serial.print("released "); Serial.println(name_);
    // }
  }

  // Grab tokens and initialize duty
  if ( !*token_  && duty != 0 )
  {
    *token_ = true;
    have_tokens_ = true;
    duty_count_ = 0;
    digitalWrite(pin_, LOW);
    // Serial.print("grabbed "); Serial.println(name_);
 }

  // Play
  if ( have_tokens_ )
  {
    if ( duty_count_ < duty )
      digitalWrite(pin_, HIGH);
    else
      digitalWrite(pin_, LOW);
    duty_count_ += 10;
  }

}

