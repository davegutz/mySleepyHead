/*  Collision constants

15-Aug-2024 	DA Gutz 	Created from SOC_Particle code.

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

*/

#pragma once

#define USE_ARDUINO
#define SAVE_RAW

// Setup
#include "local_config.h"
const String unit_key = version + "_" + HDWE_UNIT;
const uint8_t plot_num_def = 10;

const double MAX_DT_EYE = 0.1;      // Maximum filter update time in call to prevent aliasing, sec (0.1)
const double MAX_DT_HEAD = 0.2;     // Maximum filter update time in call to prevent aliasing, sec (0.2)
const float deg_to_rps = 0.0174533;

// Hardware pins
const int sensorPin = 20;     // Pin connected to the IR sensor (or eye detection sensor)
const int buzzerPin = A3;     // Pin connected to the buzzer
const int motorPin = 21;     // Pin connected to the buzzer


// Constants; anything numeric (adjustable)
#define ONE_DAY_MILLIS  86400000UL      // Number of milliseconds in one day (24*60*60*1000)
#define TALK_DELAY           313UL      // Talk wait, ms (313UL = 0.313 sec)
#define EYE_DELAY             25UL      // Sensor read wait, ms (25UL = 0.025 sec)
#define HEAD_DELAY           100UL      // Sensor read wait, ms (100UL = 0.1 sec)
#define CONTROL_DELAY        100UL      // Output read wait, ms (100UL = 0.1 sec)
#define PUBLISH_DELAY        100UL      // Publish wait, ms (100UL = 0.1 sec)
#define BLINK_DELAY           80UL      // Blink wait, ms (80UL = 0.08 sec)
#define ACTIVE_DELAY         200UL      // Active wait, ms (200UL = 0.2 sec)
#define W_MAX                 34.9      // Max rotational value, rps (34.9) limit of hardware
#define G_MAX                   4.      // Max G value, g's (4.) limit of hardware 
#define D_MAX                2000.      // Max rotational value, deg/s (34.9*180/pi) limit of hardware
#define SERIAL_BAUD         115200      // Serial baud rate (115200).
#define TAU_E_FILT             0.1      // Eye filter time constant, sec (0.1)
#define TAU_FILT               0.1      // Tau filter, sec (0.1)
#define TAU_Q_FILT             0.1      // Quiet rate time constant, sec (0.1)
#define WN_Q_FILT               5.      // Quiet filter-2 natural frequency, r/s (5.)
#define ZETA_Q_FILT            0.9      // Quiet fiter-2 damping factor (0.9)
#define QUIET_A                0.1      // Quiet set threshold, sec (0.1)
#define O_QUIET_THR           45.8      // deg/s quiet detection threshold, small is more sensitive (4.)
#define G_QUIET_THR           0.06      // g's quiet detection threshold, small is more sensitive (0.3)
#define NDATUM                 560      // Number of datum entries to store (560)  varies depending on program size
#define NHOLD                    5      // Number of precursor entries to store (5)
#define ARBITRARY_TIME  1704067196      // 1/1/2024 at ~12:00:00 AM
#define D_EYE_VOLTAGE_D_VCC 0.7614      // Sensitivity of eye voltage to VCC, V/V (0.7614)
const float QUIET_S = 0.4;              // Quiet set persistence, sec (0.4)
const float R_SCL = 10.;                // Quiet reset persistence scalar on QUIET_S ('up 1 down 10')
const float QUIET_R = (QUIET_S/R_SCL);  // Quiet reset persistence, sec

const float two_kp_def = 10.0;             // Proportional gain Kp (10.0)
const float two_ki_def = 2.0;              // Integral gain Ki (2.0)
const float pitch_thr_def_forte = 17.;   // Threshold sleep detect screech (17.), deg
const float roll_thr_def_forte = 17.;    // Threshold sleep detect screech (17.), deg
const float pitch_thr_def_piano = 12.;   // Threshold sleep detect buzz only (12.), deg 
const float roll_thr_def_piano = 12.;    // Threshold sleep detect buzz only (12.), deg
const float EYE_S = 1.5;               // Persistence eye closed IR sense set, sec (1.5)
const float EYE_R = 0.5;               // Persistence eye closed IR sense reset, sec (0.5)
const float TAU_LT = 20.;                // Long term filter time constant, sec (20)
const float TAU_ST = 0.1;                // Short term filter time constant, sec (0.1)
const float FLT_THR_POS = 0.04;          // LTST filter positive dltst fault threshold, v (0.04)
const float FRZ_THR_POS = 0.01;          // LTST filter positive dltst freeze threshold, v (0.01)
const float OFF_S = 0.04;                // Persistence glasses off IR sense set, sec (0.04)
const float OFF_R = 3.0;                 // Persistence glasses off IR sense reset, sec (3.0)
const float GLASSES_OFF_VOLTAGE = 2.5;   // Glasses off voltage, V (2.5) above this value assumed off and reset until clear for 3 seconds (user reset)
const float HEAD_S = 0.04;               // Persistence head sense set, sec (0.04)  Head needs little; heavily filtered by Mahony
const float HEAD_R = 0.04;               // Persistence head sense reset, sec (0.04)
const float SHAKE_S = 0.2;               // Persistence head shake motion sense set, sec (0.2) update time is 0.1
const float SHAKE_R = 4.0;               // Persistence head shake motion sense reset, sec (4.0)

// Buzzer Pui 24a
// const int   buzz_freq_grav = 3500;       // Buzzer frequency when gravity detected, Hz (2000)
// const int   buzz_freq_ir = 3800;        // Buzzer frequency when IR detected, Hz (3000) 

// Buzzer Pui x4033
const int   buzz_freq_grav = 2800;       // Buzzer frequency when gravity detected, Hz (2800)
const int   buzz_freq_ir = 3300;        // Buzzer frequency when IR detected, Hz (3300) 

// Analog sensor definitions
const float v3v3_nom = 3.3;                 // IR detector power supply, v (3.3)
const int v3v3_units = 4095;            // A/D range, units (1023)
