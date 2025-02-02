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
    : reset_(false), T_(0), in_1_(0), in_2_(0), output_(false), time_set(0), time_reset_1(0), time_reset_2(0), time_reset_3(0),
    state_1(false), state_2(false)
{}
Wag::Wag(const boolean init, const float nom_dt, const float set_all, const float hold_1, const float hold_2, const float hold_3)
    : reset_(false), T_(nom_dt), in_1_(false), in_2_(false), output_(init), time_set(set_all), time_reset_1(hold_1), time_reset_2(hold_2),
     time_reset_3(hold_3), state_1(false), state_2(false)
{
    timer_1 = new TFDelay(init, time_set, time_reset_1, T_);
    timer_2 = new TFDelay(init, time_set, time_reset_1, T_);
    timer_3 = new TFDelay(init, time_set, time_reset_1, T_);
}
Wag::~Wag() {}

// operators
// functions

        s = "Wag:"
        s += "  reset =  {:2d}".format(self.reset)
        s += "  T   =  {:5.3f}".format(self.T)
        s += "  time_set = {:7.3f}".format(self.time_set)
        s += "  time_reset_1 = {:7.3f} time_reset_2 = {:7.3f}".format(self.time_reset_1, self.time_reset_2, self.time_reset_3)
        s += "  inputx2 = {:7.3f} inputx1 = {:7.3f}".format(self.in_by_2, self.in_by_1)
        s += "  state_1 = {:7.3f} state_2 = {:7.3f} state_3= {:7.3f}".format(self.state_1, self.state_2, self.output)
        return s


  String Wag::repr();

  boolean Wag::calculate(const boolean reset, const float T, const float in_1, const float in_2);
{

  return ( output_ );
}
