//
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

#include "CollDatum.h"


// extern SavedPars sp;       // Various parameters to be static at system level and saved through power cycle
// extern VolatilePars ap; // Various adjustment parameters shared at system level

////////////////////////////////////////////////////////////////
// struct Datum_st data points

// Copy functions
void Datum_st::filt_from(Sensors *Sen)
{
  t_ms = Sen->t_ms;
  T_rot_int = int16_t(Sen->T_rot() * T_SCL);
  a_int = int16_t(Sen->a_filt * O_SCL);
  b_int = int16_t(Sen->b_filt * O_SCL);
  c_int = int16_t(Sen->c_filt * O_SCL);
  T_acc_int = int16_t(Sen->T_acc() * T_SCL);
  x_int = int16_t(Sen->x_filt * G_SCL);
  y_int = int16_t(Sen->y_filt * G_SCL);
  z_int = int16_t(Sen->z_filt * G_SCL);
}
void Datum_st::from(Datum_st input)
{
  t_ms = input.t_ms;
  T_rot_int = input.T_rot_int;
  a_int = input.a_int;
  b_int = input.b_int;
  c_int = input.c_int;
  T_acc_int = input.T_acc_int;
  x_int = input.x_int;
  y_int = input.y_int;
  z_int = input.z_int;
}
void Datum_st::raw_from(Sensors *Sen)
{
  t_ms = Sen->t_ms;
  T_rot_int = int16_t(Sen->T_rot() * T_SCL);
  a_int = int16_t(Sen->a_raw * O_SCL);
  b_int = int16_t(Sen->b_raw * O_SCL);
  c_int = int16_t(Sen->c_raw * O_SCL);
  T_acc_int = int16_t(Sen->T_acc() * T_SCL);
  x_int = int16_t(Sen->x_raw * G_SCL);
  y_int = int16_t(Sen->y_raw * G_SCL);
  z_int = int16_t(Sen->z_raw * G_SCL);
}

// Nominal values
void Datum_st::nominal()
{
  t_ms = time_t (1ULL);
  T_rot_int = int16_t(0);
  a_int = int16_t(0);
  b_int = int16_t(0);
  c_int = int16_t(0);
  T_acc_int = int16_t(0);
  x_int = int16_t(0);
  y_int = int16_t(0);
  z_int = int16_t(0);
}

// Print functions
void Datum_st::plot(const uint16_t i)
{
  #ifndef SAVE_RAW
    Serial.print("T_rot_filt*100:"); Serial.print(float(T_rot_int) * 100. / T_SCL, 3);
    Serial.print("\ta_filt:"); Serial.print(float(a_int) / O_SCL, 3);
    Serial.print("\tb_filt:"); Serial.print(float(b_int) / O_SCL, 3);
    Serial.print("\tc_filt:"); Serial.print(float(c_int) / O_SCL, 3);
    Serial.print("\tT_acc_filt*100:"); Serial.print(float(T_acc_int) * 100. / T_SCL, 3);
    Serial.print("\tx_filt:"); Serial.print(float(x_int) / G_SCL, 3);
    Serial.print("\ty_filt:"); Serial.print(float(y_int) / G_SCL, 3);
    Serial.print("\tz_filt:"); Serial.println(float(z_int) / G_SCL, 3);
  #else
    Serial.print("T_rot_raw*100:"); Serial.print(float(T_rot_int) * 100. / T_SCL, 3);
    Serial.print("\ta_raw:"); Serial.print(float(a_int) / O_SCL, 3);
    Serial.print("\tb_raw:"); Serial.print(float(b_int) / O_SCL, 3);
    Serial.print("\tc_raw:"); Serial.print(float(c_int) / O_SCL, 3);
    Serial.print("\tT_acc_raw*100:"); Serial.print(float(T_acc_int) * 100. / T_SCL, 3);
    Serial.print("\tx_raw:"); Serial.print(float(x_int) / G_SCL, 3);
    Serial.print("\ty_raw:"); Serial.print(float(y_int) / G_SCL, 3);
    Serial.print("\tz_raw:"); Serial.println(float(z_int) / G_SCL, 3);
  #endif
}

void Datum_st::print(const uint16_t i)
{
  cSF(prn_buff, INPUT_BYTES, "");
  prn_buff = "---";
  time_long_2_str(t_ms, prn_buff);
  Serial.print(i);
  Serial.print(" "); Serial.print(t_ms);
  Serial.print(" "); Serial.print(prn_buff);
  #ifndef SAVE_RAW
    Serial.print(" T_rot_filt "); Serial.print(float(T_rot_int) / T_SCL, 3);
    Serial.print(" a_filt "); Serial.print(float(a_int) / O_SCL, 3);
    Serial.print(" b_filt "); Serial.print(float(b_int) / O_SCL, 3);
    Serial.print(" c_filt "); Serial.print(float(c_int) / O_SCL, 3);
    Serial.print(" T_acc_filt "); Serial.print(float(T_acc_int) / T_SCL, 3);
    Serial.print(" x_filt "); Serial.print(float(x_int) / G_SCL, 3);
    Serial.print(" y_filt "); Serial.print(float(y_int) / G_SCL, 3);
    Serial.print(" z_filt "); Serial.println(float(z_int) / G_SCL, 3);
  #else
    Serial.print(" T_rot_raw "); Serial.print(float(T_rot_int) / T_SCL, 3);
    Serial.print(" a_raw "); Serial.print(float(a_int) / O_SCL, 3);
    Serial.print(" b_raw "); Serial.print(float(b_int) / O_SCL, 3);
    Serial.print(" c_raw "); Serial.print(float(c_int) / O_SCL, 3);
    Serial.print(" T_acc_raw "); Serial.print(float(T_acc_int) / T_SCL, 3);
    Serial.print(" x_raw "); Serial.print(float(x_int) / G_SCL, 3);
    Serial.print(" y_raw "); Serial.print(float(y_int) / G_SCL, 3);
    Serial.print(" z_raw "); Serial.println(float(z_int) / G_SCL, 3);
  #endif
}

// nominalize
void Datum_st::put_nominal()
{
  Datum_st source;
  source.nominal();
  from(source);
}


//////////////////////////////////////////////////////////
// struct Data_st data log

// Delete the over-ridden registers except the one we're currently filling
void Data_st::clear_register_overlap(Register_st *CurrentReg)
{
  for ( int j=0; j<nRg_; j++ )
  {
    if ( CurrentReg == Reg[j] ) continue;
    if ( (iR_ < (Reg[j]->i + Reg[j]->n-1)) && (iStart_ > Reg[j]->i)  && (iR_ > Reg[j]->i) ) 
    {
Serial.print("j="); Serial.print(j); Serial.print(" iStart_="); Serial.print(iStart_); Serial.print(" iR_="); Serial.print(iR_); Serial.print(" Reg[j]->i="); Serial.print(Reg[j]->i); Serial.print(" Reg[j]->n="); Serial.print(Reg[j]->n); Serial.print(" Reg[j]->t_ms="); Serial.print(Reg[j]->t_ms);
      Serial.println(" nominal");
      Reg[j]->put_nominal();
    }
  }
  // sort_registers();
}

// Transfer precursor data to storage
void Data_st::move_precursor()
{
  uint16_t count = 0;
  uint16_t j = iP_;
  while ( count++ < nP_ -1 )
  {
    if ( ++j > (nP_-1) ) j = 0;  // circular buffer
    if ( Precursor[j]->t_ms == 1ULL ) continue;
    put_ram(Precursor[j]);
  }
  nAR_ = min(nAR_+1, NREG);  // This is  not perfect
}

void Data_st::plot_latest_ram()
{
  int begin = max(min( Reg[iRg_]->i, nR_-1), 0);
  int end = max(min(begin + Reg[iRg_]->n/2, nR_-1), 0);  // plot half
  for ( int i=begin; i<end; i+=2 )
  {
    if ( i==begin ) for ( int j=0; j<5; j++ ) { Ram[i]->plot(j); delay(16UL); }
    Ram[i]->plot(i); delay(16UL);
  }
}

void Data_st::print_all_registers()
{
  for ( int i=0; i<nRg_; i++ )
    Reg[i]->print(nR_);
}

void Data_st::print_latest_datum()
{
  Ram[iR_]->print(iR_);
}

void Data_st::print_latest_register()
{
  Reg[iRg_]->print(nR_);
}

void Data_st::print_latest_ram()
{
  int begin = max(min( Reg[iRg_]->i, nR_-1), 0);
  int end = max(min(begin + Reg[iRg_]->n, nR_-1), 0);
  for ( int i=begin; i<end; i++ )
  {
    Ram[i]->print(i);
  }
}


void Data_st::print_ram()
{
  for (int j = 0; j < nR_; j++)
  {
    Ram[j]->print(j);
  }
}

// Precursor storage
void Data_st:: put_precursor(Sensors *Sen)
{
  if ( ++iP_ > (nP_-1) ) iP_ = 0;  // circular buffer
  #ifndef SAVE_RAW
    Precursor[iP_]->filt_from(Sen);
  #else
    Precursor[iP_]->raw_from(Sen);
  #endif
}

void Data_st::put_ram(Sensors *Sen)
{
  if ( ++iR_ > (nR_-1) ) iR_ = 0;  // circular buffer
  #ifndef SAVE_RAW
    Ram[iR_]->filt_from(Sen);
    Reg[iRg_]->o_raw_max = max(Reg[iRg_]->o_raw_max, Sen->o_raw);
    Reg[iRg_]->g_raw_max = max(Reg[iRg_]->g_raw_max, Sen->g_raw);
    Reg[iRg_]->o_filt_max = max(Reg[iRg_]->o_filt_max, Sen->o_filt);
    Reg[iRg_]->g_filt_max = max(Reg[iRg_]->g_filt_max, Sen->g_filt);
  #else
    Ram[iR_]->raw_from(Sen);
  #endif
  clear_register_overlap(CurrentRegPtr_);
}

void Data_st::put_ram(Datum_st *point)
{
  if ( ++iR_ > (nR_-1) ) iR_ = 0;  // circular buffer
  #ifndef SAVE_RAW
    Ram[iR_]->from(*point);
  #else
    Ram[iR_]->from(*point);
  #endif
  clear_register_overlap(CurrentRegPtr_);
}

// Enter information about last data set into register
void Data_st::register_lock(const boolean quiet, Sensors *Sen)
{
  iRg_++;
  if ( iRg_ > (nRg_-1) ) iRg_ = 0;  // circular buffer
  iStart_ = iR_;
  if ( iStart_ > (nR_-1) ) iStart_ = 0;  // circular buffer
  if ( !quiet ) { Serial.print(" lock: iRg_="); Serial.print(iRg_); Serial.print(" iStart_="); Serial.println(iStart_); }
  Reg[iRg_]->locked = true;
  Reg[iRg_]->i = iStart_;
  CurrentRegPtr_ = Reg[iRg_];
  Reg[iRg_]->o_raw_max = 0;
  Reg[iRg_]->o_filt_max = 0;
  Reg[iRg_]->g_raw_max = 0;
  Reg[iRg_]->g_filt_max = 0;
}
void Data_st::register_unlock(const boolean quiet, Sensors *Sen)
{
  Reg[iRg_]->t_ms = Ram[iStart_]->t_ms;
  boolean reg_wrapped = false;
  if ( Reg[iRg_]->i < iR_ )
  {
    Reg[iRg_]->n = iR_ - Reg[iRg_]->i;
  }
  else
  {
    Reg[iRg_]->n = nR_ - (Reg[iRg_]->i - iR_);
    reg_wrapped = true;
  }
  if ( reg_wrapped )
  {
    uint16_t j = 0;
    while ( ( Reg[j]->i < iR_ ) && ( j < nRg_ ) )
    {
      Reg[j]->put_nominal();
      j++;
    }
  }
  Reg[iRg_]->locked = false;
  if ( !quiet )
  {
    Serial.print("unlock: iRg_="); Serial.print(iRg_);
    Serial.print(" iR_="); Serial.print(iR_); 
    Serial.print(" Reg[iRg_]->i="); Serial.print(Reg[iRg_]->i);
    Serial.print(" n="); Serial.println(Reg[iRg_]->n);
  }
}

// Reset
void Data_st::reset(const boolean reset)
{
  if ( reset )
  {
    iP_ = 0;
    for ( int j=0; j<nP_; j++ ) Precursor[j]->put_nominal();
    iR_ = 0;
    for ( int j=0; j<nR_; j++ ) Ram[j]->put_nominal();
  }
}

// Sort registers
void Data_st::sort_registers()
{
  Serial.println("TODO: sort registers");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

// For summary prints
void time_long_2_str(const unsigned long long _time_ms, SafeString &return_str)
{
    int thou_ = _time_ms % 1000;
    time_t time = _time_ms / 1000;
    char tempStr[36];
    #ifndef USE_ARDUINO
      uint32_t year_ = Time.year(time);
      uint8_t month_ = Time.month(time);
      uint8_t day_ = Time.day(time);
      uint8_t hours_ = Time.hour(time);
      uint8_t minutes_   = Time.minute(time);
      uint8_t seconds_   = Time.second(time);
    #else
      uint32_t year_ = year(time);
      uint8_t month_ = month(time);
      uint8_t day_ = day(time);
      uint8_t hours_ = hour(time);
      uint8_t minutes_   = minute(time);
      uint8_t seconds_   = second(time);
    #endif
    sprintf(tempStr, "%4u-%02u-%02uT%02u:%02u:%02u.%03d", int(year_), month_, day_, hours_, minutes_, seconds_, thou_);
    return_str = tempStr;
}

