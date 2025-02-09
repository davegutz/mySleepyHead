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

#include "Tones.h"

void Tone::play_eye_ready_chirp(const boolean play)
{
  static boolean last_play = false;
  static uint8_t count = 0;
  if ( count > 90 ) count = 0;
  if (play)
  {
    Serial.println("playing_ready_chirp");
    if ( count < DUTY_EYE_READY )
        tone(buzzerPin_, buzz_freq_ir); //tone(buzzerPin_, buzz_freq_ready_chirp_); 
    else
        tone(buzzerPin_, 0); //tone(buzzerPin_, buzz_freq_ready_chirp_); 
    count += 10;
  }
  else
  {
      count = 0;
  }
}

void Tone::play_eye_reset_chirp(const boolean play)
{
  static boolean last_play = false;
  static uint8_t count = 0;
  if ( count > 90 ) count = 0;
  if (play)
  {
    Serial.println("playing_reset_chirp");
    if ( count < DUTY_EYE_RESET )
        tone(buzzerPin_, buzz_freq_ir_reset); //tone(buzzerPin_, buzz_freq_ready_chirp_); 
    else
        tone(buzzerPin_, 0); //tone(buzzerPin_, buzz_freq_ready_chirp_); 
    count += 10;
  }
  else
  {
      count = 0;
  }
}
  