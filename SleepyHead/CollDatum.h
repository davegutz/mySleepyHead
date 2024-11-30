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


#ifndef _COLLDATUM_H
#define _COLLDATUM_H

#include <SafeString.h>
#include "constants.h"
#include "Sensors.h"
#include "TimeLib.h"

#ifdef USE_ARDUINO
  #include <Arduino.h> //needed for Serial.println
  #include <Arduino_LSM6DS3.h>
#else
  #error "Only Arduino nano 33 iot has built in IMU"
  #include "application.h"  // Particle
#endif

extern time_t time_initial;


void time_long_2_str(const unsigned long long time_ms, SafeString &tempStr);


// Datum struct
struct Datum_st
{
  unsigned long long t_ms = 1ULL;
  int16_t T_rot_int = 0;
  int16_t a_int = 0;
  int16_t b_int = 0;
  int16_t c_int = 0;
  int16_t T_acc_int = 0;
  int16_t x_int = 0;
  int16_t y_int = 0;
  int16_t z_int = 0;

  void get() {};
  void nominal();
  void plot(const uint16_t i);
  void print(const uint16_t i);
  void filt_from(Sensors *Sen);
  void from(Datum_st input);
  void put_nominal();
  void raw_from(Sensors *Sen);
};


// Index to current data
struct Register_st
{
public:
  uint16_t i = 0;
  uint16_t n = 0;
  unsigned long long t_ms = 0ULL;
  boolean locked = false;
  float o_raw_max = 0;
  float g_raw_max = 0;
  float o_filt_max = 0;
  float g_filt_max = 0;

  boolean is_empty() { if (t_ms) return(false); else return(true); };

  // Print function
  void print(uint16_t nR)
  {
    Serial.print("Reg: t_ms:"); Serial.print(t_ms);
    Serial.print(" i:"); Serial.print(i);
    uint16_t end = i + n - 1;
    if ( end > nR - 1 ) end -= nR;
    Serial.print(" - "); Serial.print(end);
    Serial.print(" n: "); Serial.print(n);
    Serial.print(" o_raw_max:"); Serial.print(o_raw_max);
    Serial.print(" o_filt_max:"); Serial.print(o_filt_max);
    Serial.print(" g_raw_max:"); Serial.print(g_raw_max);
    Serial.print(" g_filt_max:"); Serial.print(g_filt_max);
    Serial.println();
  }
  
  void put_nominal() { i = 0; n = 0; t_ms = 0ULL; locked = false; o_raw_max = 0; g_raw_max = 0; o_filt_max = 0; g_filt_max = 0; };
};


// Ram manager
class Data_st
{
public:
  Data_st() : iR_(0), nR_(0), iP_(0), nP_(0), iRg_(0), iStart_(0), nRg_(0), nAR_(0) {};
  Data_st(uint16_t ram_datums, uint16_t pre_datums, uint16_t reg_registers) :
   iR_(ram_datums), nR_(ram_datums),
   iP_(pre_datums), nP_(pre_datums),
   iRg_(reg_registers), nRg_(reg_registers),
   nAR_(0)
  {
    int j;
    Precursor = new Datum_st*[nP_];
    for (j=0; j<nP_; j++)
    {
      Precursor[j] = new Datum_st();
      Precursor[j]->nominal();
    }
    Ram = new Datum_st*[nR_];
    for (j=0; j<nR_; j++)
    {
      Ram[j] = new Datum_st();
      Ram[j]->nominal();
    }
    Reg = new Register_st*[nRg_];
    for (j=0; j<nRg_; j++)
    {
      Reg[j] = new Register_st();
      Reg[j]->put_nominal();
    }
    Serial.print("Size of Data_st: "); Serial.println(size());
  };
  ~Data_st();
  void clear_register_overlap(Register_st *CurrentReg);
  void get();
  uint16_t iR(){ return iR_; };
  uint16_t iRg(){ return iRg_; };
  uint16_t nR(){ return nR_; };
  void move_precursor();
  int num_active_registers(){ return int(nAR_); };
  void plot_latest_ram();
  void print_latest_datum();
  void print_latest_ram();
  void print_all_registers();
  void print_latest_register();
  void print_ram();
  void put_precursor(Sensors *Sen);
  // void from(Datum_st input);
  void put_ram(Sensors *Sen);
  void put_ram(Datum_st *point);
  void register_lock(const boolean quiet, Sensors *Sen);
  void register_unlock(const boolean quiet, Sensors *Sen);
  void reset(const boolean reset);
  int size(){ return nR_ * sizeof(Datum_st); };
  void sort_registers();

protected:
  Datum_st **Precursor; // Precursor storage
  Datum_st **Ram;       // Ram storage
  Register_st **Reg, *CurrentRegPtr_;    // Register for Ram
  uint16_t iR_, iP_, iRg_, iStart_;
  uint16_t nR_, nP_, nRg_;
  uint8_t nAR_;
};


#endif
