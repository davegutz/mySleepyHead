/*  Collision constants

15-Aug-2024 	DA Gutz 	Created from SOC_Particle code.

// MIT License
//
// Copyright (C) 2024 - Dave Gutz
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

#undef USE_ARDUINO
#define SAVE_RAW

// Setup
#include "local_config.h"
#include <SafeString.h>
#include "local_config.h"
const String unit_key = version + "_" + HDWE_UNIT;

const double NOM_DT_EYE = 0.01;
const double NOM_DT_HEAD = 0.1;
const float deg_to_rps = 0.0174533;

// Constants; anything numeric (adjustable)
#define ONE_DAY_MILLIS        86400000UL// Number of milliseconds in one day (24*60*60*1000)
#define TALK_DELAY           313UL      // Talk wait, ms (313UL = 0.313 sec)
#define EYE_DELAY             20UL      // Sensor read wait, ms (20UL = 0.02 sec) Dr
#define HEAD_DELAY           100UL      // Sensor read wait, ms (100UL = 0.1 sec) Dr
#define CONTROL_DELAY         20UL      // Control read wait, ms (20UL = 0.02 sec)
#define LOG_DELAY             20UL      // Register wait, ms (20UL = 0.01 sec)
#define PUBLISH_DELAY        100UL      // Publish wait, ms (100UL = 0.1 sec)
#define BLINK_DELAY           80UL      // Blink wait, ms (80UL = 0.08 sec)
#define ACTIVE_DELAY         200UL      // Active wait, ms (200UL = 0.2 sec)
#define G_MAX                  20.      // Max G value, g's (20.) 
#define W_MAX                  20.      // Max rotational value, rps (20.)
#define T_MAX                 0.02      // Max expected update time, s (0.02)
#define INPUT_BYTES            200      // Serial input buffer sizes (200)
#define SERIAL_BAUD         115200      // Serial baud rate (115200).
#define TAU_FILT              0.01      // Tau filter, sec (0.01)
#define TAU_Q_FILT            0.01      // Quiet rate time constant, sec (0.01)
#define MIN_Q_FILT            -20.      // Quiet filter minimum, g's / rps(-20)
#define MAX_Q_FILT             20.      // Quiet filter maximum, g's / rps (20)
#define WN_Q_FILT              25.      // Quiet filter-2 natural frequency, r/s (25.)
#define ZETA_Q_FILT            0.9      // Quiet fiter-2 damping factor (0.9)
#define MAX_T_Q_FILT          0.02      // Quiet filter max update time, s (0.02)
#define QUIET_A                0.1      // Quiet set threshold, sec (0.1)
#define QUIET_S                0.4      // Quiet set persistence, sec (0.4)
#define O_QUIET_THR           12.0      // rps quiet detection threshold (12.)
#define G_QUIET_THR            4.0      // g's quiet detection threshold (4.)
#define NDATUM                 560      // Number of datum entries to store (560)  varies depending on program size
#define NHOLD                    5      // Number of precursor entries to store (5)
#define R_SCL                  10.      // Quiet reset persistence scalar on QUIET_S ('up 1 down 10')
#define ARBITRARY_TIME  1704067196      // 1/1/2024 at ~12:00:00 AM
#define D_EYE_VOLTAGE_D_VCC 0.7614      // Sensitivity of eye voltage to VCC, V/V (0.7614)

const float t_kp_def = 10.0;             // Proportional gain Kp (10.0)
const float t_ki_def = 2.0;              // Integral gain Ki (2.0)
const float pitch_thr_def_forte = 17.;   // Threshold sleep detect screech (17.), deg
const float roll_thr_def_forte = 17.;    // Threshold sleep detect screech (17.), deg
const float pitch_thr_def_piano = 12.;   // Threshold sleep detect buzz only (12.), deg 
const float roll_thr_def_piano = 12.;    // Threshold sleep detect buzz only (12.), deg
const float delta_pitch_def = 0.;        // Initial delta pitch (0.)
const float delta_roll_def = 0.;         // Initial delta roll (0.)
const float CLOSED_S = 1.0;              // Persistence eye closed IR sense, sec (1.0)  Head needs none; heavily filtered by Mahoney
const float CLOSED_R = 0.5;              // Persistence eye closed IR sense, sec (0.5)
const float  TAU_LT = 20.;               // Long term filter time constant, sec (20)
const float  TAU_ST = 0.4;               // Short term filter time constant, sec (0.4)
const float FLT_THR_POS = 0.04;          // LTST filter positive dltst fault threshold, v (0.04)
const float FRZ_THR_POS = 0.01;          // LTST filter positive dltst freeze threshold, v (0.01)

// Pui 24a
// const int   buzz_freq_grav = 3500;       // Buzzer frequency when gravity detected, Hz (2000)
// const int   buzz_freq_ir = 3800;        // Buzzer frequency when IR detected, Hz (3000) 
// Pui x4033
const int   buzz_freq_grav = 2800;       // Buzzer frequency when gravity detected, Hz (2800)
const int   buzz_freq_ir = 3300;        // Buzzer frequency when IR detected, Hz (3300) 

const float QUIET_R = (QUIET_S/R_SCL);  // Quiet reset persistence, sec 
const float O_SCL = (16000./W_MAX);     // Rotational int16_t scale factor
const float G_SCL = (16000./G_MAX);     // Rotational int16_t scale factor
const float T_SCL = (32000./T_MAX);     // Rotational int16_t scale factor
const uint8_t NREG = (NDATUM)/((QUIET_S)/(LOG_DELAY)*1000*float(R_SCL+1)/float(R_SCL)+NHOLD); // Dynamically determine number of data sets to allow for if small as possible

// Analog sensor definitions
const float v3v3_nom = 3.3;                 // IR detector power supply, v (3.3)
const int v3v3_units = 4095;            // A/D range, units (1023)
