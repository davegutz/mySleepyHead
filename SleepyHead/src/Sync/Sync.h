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
//
// 17-Feb-2021  Dave Gutz   Create

#pragma once

// Duct Sim Class
class Sync
{
public:
  // Constructors
  Sync(void);
  Sync(unsigned long long delay);
  // Functions
  boolean update(boolean reset, unsigned long long now, boolean andCheck);
  boolean update(unsigned long long now, boolean reset, boolean andCheck);
  boolean update(unsigned long long now, boolean reset);
  boolean updateN(unsigned long long now, boolean reset, boolean orCheck);
  unsigned long long delay() { return(delay_); }
  void delay(unsigned long long new_delay) { delay_ = new_delay; updateTimeInput_ = float(delay_)/1000.; }
  void delay(unsigned long long new_delay, unsigned long long now) { delay_ = new_delay; updateTimeInput_ = float(delay_)/1000.; last_ = now; }
  unsigned long long last() { return(last_); }
  boolean stat() { return(stat_); }
  unsigned long long updateDiff() { return(updateDiff_); }
  double updateTime() { return(updateTime_); }
  double updateTimeInput() { return(updateTimeInput_); }
  unsigned long long now() { return(now_); }
private:
  unsigned long long delay_;
  unsigned long long last_;
  unsigned long long now_;
  boolean stat_;
  unsigned long long updateDiff_;
  double updateTime_;
  double updateTimeInput_;
};
