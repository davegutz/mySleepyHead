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

class CommandPars
{
public:
  CommandPars() : inp_token(false), tone_token(false), motor_token(false), led_token(false)
  {
    inp_str = "";
  }
  ~CommandPars();
  // Small static value area for 'retained'
  String inp_str;           // Hold incoming data queue
  boolean inp_token;        // Whether inp_str is complete
  boolean tone_token;       // Permission to buzz
  boolean motor_token;      // Permission to run motor
  boolean led_token;        // Permission to run led

  void pretty_print(void)
  {
    #ifndef SOFT_DEPLOY_PHOTON
      Serial.println("command parameters(cp):");
    #endif
  }

};            

CommandPars::~CommandPars() {}
