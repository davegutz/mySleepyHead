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
#include "../../constants.h"


// Generate tones
class Tone
{
  public:
    Tone(const int pin): buzzerPin_(pin), buzz_freq_grav_(buzz_freq_grav), buzz_freq_ir_(buzz_freq_ir), isPlaying_(false) {}
    void begin()
    {
      pinMode(buzzerPin_, OUTPUT);
      digitalWrite(buzzerPin_, LOW);
    }
    int gravityFreq() { return buzz_freq_grav_; }
    void gravityFreq(const int inp) { buzz_freq_grav_ = inp; }
    int irFreq() { return buzz_freq_ir_; }
    void irFreq(const int inp) { buzz_freq_ir_ = inp; }
    boolean isPlaying() { return isPlaying_; }
    void play_eye_ready_chirp(const boolean play);
    void play_eye_reset_chirp(const boolean play);
    void play_grav() { tone(buzzerPin_, buzz_freq_grav_); Serial.println("grav tone played"); isPlaying_ = true; }
    void play_ir() { tone(buzzerPin_, buzz_freq_ir_); Serial.println("ir tone played"); isPlaying_ = true; }
    void stop() { noTone(buzzerPin_); Serial.println("tone stopped"); isPlaying_ = false; }

  private:
    int buzzerPin_;
    int buzz_freq_grav_;
    int buzz_freq_ir_;
    boolean isPlaying_;
};
