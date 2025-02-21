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
#include "version.h"
// #include "command.h"

// Hardware pins
const int sensorPin = 20;    // Pin connected to the IR sensor (or eye detection sensor)
const int tonePin = A3;      // Pin connected to the buzzer
const int motorPin = 21;     // Pin connected to the motor
const int configPin0 = 15;   // Pin connected to the config bit 0
const int configPin1 = 16;   // Pin connected to the config bit 1

// Built-in configs
const String HDWE_UNIT[4]       = {"wearDn33iot",   "prototype",    "prototype",    "prototype"};
const float delta_pitch[4]  = {13.,             0.,             0.,             0.};
const float delta_roll[4]   = {5.,              0.,             0.,             0.};

#define USE_ARDUINO
