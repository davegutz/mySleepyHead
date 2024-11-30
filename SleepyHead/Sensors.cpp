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


#include "constants.h"
#include "Sensors.h"
#include "TimeLib.h"
#include "CollDatum.h"


// Filter noise
void Sensors::filter(const boolean reset)
{

    if ( reset || acc_available_ )
    {
        x_filt = X_Filt->calculate(x_raw, reset, TAU_FILT, min(T_acc_, NOM_DT));
        y_filt = Y_Filt->calculate(y_raw, reset, TAU_FILT, min(T_acc_, NOM_DT));
        z_filt = Z_Filt->calculate(z_raw, reset, TAU_FILT, min(T_acc_, NOM_DT));
        g_filt = G_Filt->calculate(g_raw, reset, TAU_FILT, min(T_acc_, NOM_DT));
        g_qrate = GQuietRate->calculate(g_raw-1., reset, min(T_acc_, MAX_T_Q_FILT));     
        g_quiet =GQuietFilt->calculate(g_qrate, reset, min(T_acc_, MAX_T_Q_FILT));
        static int count = 0;
    }

    if ( reset || rot_available_ )
    {
        a_filt = A_Filt->calculate(a_raw, reset, TAU_FILT, min(T_rot_, NOM_DT));
        b_filt = B_Filt->calculate(b_raw, reset, TAU_FILT, min(T_rot_, NOM_DT));
        c_filt = C_Filt->calculate(c_raw, reset, TAU_FILT, min(T_rot_, NOM_DT));
        o_filt = O_Filt->calculate(o_raw, reset, TAU_FILT, min(T_rot_, NOM_DT));
        o_qrate = OQuietRate->calculate(o_raw, reset, min(T_rot_, MAX_T_Q_FILT));     
        o_quiet =OQuietFilt->calculate(o_qrate, reset, min(T_rot_, MAX_T_Q_FILT));
    }

}

// Print publish
// plot pa3
void Sensors::plot_all()  // pa3
{
  float g_q = -2.;
  float g_q_s = -2.; 
  if ( g_is_quiet_ ) g_q = -1;
  if ( g_is_quiet_sure_ ) g_q_s = -1;
  Serial.print("x_filt:"); Serial.print(x_filt, 3);
  Serial.print("\ty_filt:"); Serial.print(y_filt, 3);
  Serial.print("\tz_filt:"); Serial.print(z_filt, 3);
  Serial.print("\tg_filt-1:"); Serial.print(g_filt-1., 3);
  float o_q = -4.;
  float o_q_s = -4.; 
  if ( o_is_quiet_ ) o_q = -3;
  if ( o_is_quiet_sure_ ) o_q_s = -3;
  Serial.print("\t\ta_filt:"); Serial.print(a_filt, 3);
  Serial.print("\tb_filt:"); Serial.print(b_filt, 3);
  Serial.print("\tc_filt:"); Serial.print(c_filt, 3);
  Serial.print("\to_filt:"); Serial.println(o_filt, 3);
}

// plot pa1
void Sensors::plot_all_acc()  // pa1
{
  float g_q = -2.;
  float g_q_s = -2.; 
  if ( g_is_quiet_ ) g_q = -1;
  if ( g_is_quiet_sure_ ) g_q_s = -1;
  Serial.print("T_acc*100:"); Serial.print(T_acc_*100., 3);
  Serial.print("\tx_filt:"); Serial.print(x_filt, 3);
  Serial.print("\ty_filt:"); Serial.print(y_filt, 3);
  Serial.print("\tz_filt:"); Serial.print(z_filt, 3);
  Serial.print("\tg_filt-1:"); Serial.print(g_filt-1., 3);
  Serial.print("\tg_is_quiet-2:"); Serial.print(g_q, 3);
  Serial.print("\tg_is_quiet_sure-2:"); Serial.println(g_q_s, 3);
}

// plot pa2
void Sensors::plot_all_rot()  // pa2
{
  float o_q = -4.;
  float o_q_s = -4.; 
  if ( o_is_quiet_ ) o_q = -3;
  if ( o_is_quiet_sure_ ) o_q_s = -3;
  Serial.print("T_rot_*100:"); Serial.print(T_rot_*100., 3);
  Serial.print("\ta_filt:"); Serial.print(a_filt, 3);
  Serial.print("\tb_filt:"); Serial.print(b_filt, 3);
  Serial.print("\tc_filt:"); Serial.print(c_filt, 3);
  Serial.print("\to_filt:"); Serial.print(o_filt, 3);
  Serial.print("\to_is_quiet-4:"); Serial.print(o_q, 3);
  Serial.print("\to_is_quiet_sure-4:"); Serial.println(o_q_s, 3);
}

// pa0
void Sensors::plot_all_sum()  // pa0
{
  float g_q_s = -2.; 
  if ( g_is_quiet_sure_ ) g_q_s = -1;
  Serial.print("g_raw-1:"); Serial.print(g_raw-1., 3);
  Serial.print("\tg_filt-1:"); Serial.print(g_filt-1., 3);
  Serial.print("\tg_quiet:"); Serial.print(g_quiet, 3);
  Serial.print("\tg_is_quiet_sure-2:"); Serial.print(g_q_s, 3);
  float o_q_s = -4.; 
  if ( o_is_quiet_sure_ ) o_q_s = -3;
  Serial.print("\to_raw:"); Serial.print(o_raw, 3);
  Serial.print("\to_filt:"); Serial.print(o_filt, 3);
  Serial.print("\to_quiet:"); Serial.print(o_quiet, 3);
  Serial.print("\to_is_quiet_sure-4:"); Serial.println(o_q_s, 3);
}

// Print publish
void Sensors::plot_quiet()
{
  float o_q = -4.;
  float o_q_s = -4.; 
  if ( o_is_quiet_ ) o_q = -3;
  if ( o_is_quiet_sure_ ) o_q_s = -3;
  float g_q = -2.;
  float g_q_s = -2.; 
  if ( g_is_quiet_ ) g_q = -1;
  if ( g_is_quiet_sure_ ) g_q_s = -1;
  Serial.print("T_rot_*100:"); Serial.print(T_rot_*100., 3);
  Serial.print("\to_filt:"); Serial.print(o_filt, 3);
  Serial.print("\to_quiet:"); Serial.print(o_quiet, 3);
  Serial.print("\to_is_quiet_sure-4:"); Serial.print(o_q_s, 3);
  Serial.print("\t\tT_acc*100:"); Serial.print(T_acc_*100., 3);
  Serial.print("\tg_filt:"); Serial.print(g_filt-1., 3);
  Serial.print("\tg_quiet:"); Serial.print(g_quiet, 3);
  Serial.print("\tg_is_quiet_sure-2:"); Serial.println(g_q_s, 3);
}

// Print publish
void Sensors::plot_quiet_raw()
{
  Serial.print("o_quiet:"); Serial.print(o_quiet, 3);
  Serial.print("\t\tg_quiet:"); Serial.println(g_quiet, 3);
}

// Print publish
void Sensors::plot_total()
{
  Serial.print("T_rot_*100:"); Serial.print(T_rot_*100., 3);
  Serial.print("\to_filt:"); Serial.print(o_filt, 3);
  Serial.print("\t\tT_acc_*100:"); Serial.print(T_acc_*100., 3);
  Serial.print("\tg_filt:"); Serial.println(g_filt-1., 3);
}

void Sensors::print_all()
{
  Serial.print(T_rot_, 3); Serial.print('\t');
  Serial.print(a_filt, 3); Serial.print('\t');
  Serial.print(b_filt, 3); Serial.print('\t');
  Serial.print(c_filt, 3); Serial.print('\t');
  Serial.print(o_filt, 3); Serial.print('\t');
  Serial.print(o_is_quiet_, 3); Serial.print('\t');
  Serial.print(o_is_quiet_sure_, 3); Serial.print('\t');
  Serial.print(T_acc_, 3); Serial.print("\t\t");
  Serial.print(x_filt, 3); Serial.print('\t');
  Serial.print(y_filt, 3); Serial.print('\t');
  Serial.print(z_filt, 3); Serial.print('\t');
  Serial.print(g_filt, 3); Serial.print('\t');
  Serial.print(g_is_quiet_, 3); Serial.print('\t');
  Serial.println(g_is_quiet_sure_, 3);
}

void Sensors::print_all_header()
{
  Serial.println("T_rot_*\ta_filt\tb_filt\tc_filt\to_filt\to_is_quiet\to_is_quiet_sure\t\tT_acc\tx_filt\ty_filt\tz_filt\tg_filt\tg_is_quiet\tg_is_quiet_sure");
}
// Detect no signal present based on detection of quiet signal.
// Research by sound industry found that 2-pole filtering is the sweet spot between seeing noise
// and actual motion without 'guilding the lily'
void Sensors::quiet_decisions(const boolean reset)
{
  o_is_quiet_ = o_quiet <= O_QUIET_THR;  // o_filt is rss
  o_is_quiet_sure_ = OQuietPer->calculate(o_is_quiet_, QUIET_S, QUIET_R, T_rot_, reset);
  g_is_quiet_ = g_quiet <= G_QUIET_THR;  // g_filt is rss
  g_is_quiet_sure_ = GQuietPer->calculate(g_is_quiet_, QUIET_S, QUIET_R, T_acc_, reset);
  static int count = 0;
}

// Sample the IMU
void Sensors::sample(const boolean reset, const unsigned long long time_now_ms, const unsigned long long time_start_ms, time_t now_hms)
{
    // Reset
    if ( reset )
    {
        time_rot_last_ = time_now_ms - READ_DELAY;
        time_acc_last_ = time_now_ms - READ_DELAY;
    }

    // Accelerometer
    if ( !reset && IMU.accelerationAvailable() )
    {
        IMU.readAcceleration(x_raw, y_raw, z_raw);
        acc_available_ = true;
        g_raw = sqrt(x_raw*x_raw + y_raw*y_raw + z_raw*z_raw);
    }
    else acc_available_ = false;
    T_acc_ = double(time_now_ms - time_acc_last_) / 1000.;

    // Gyroscope
    if ( !reset && IMU.gyroscopeAvailable() )
    {
        IMU.readGyroscope(a_raw, b_raw, c_raw);
        a_raw *= deg_to_rps;
        b_raw *= deg_to_rps;
        c_raw *= deg_to_rps;
        rot_available_ = true;
        o_raw = sqrt(a_raw*a_raw + b_raw*b_raw + c_raw*c_raw);
    }
    else rot_available_ = false;
    T_rot_ = double(time_now_ms - time_rot_last_) / 1000.;

    // Tracking Filter
    t_filter->updateIMU(x_raw*gyroScale, y_raw*gyroScale, z_raw*gyroScale, a_raw, b_raw, c_raw);
    roll_raw = t_filter->getRoll();
    pitch_raw = t_filter->getPitch();
    yaw_raw = t_filter->getYaw();

    // Time stamp
    t_ms = time_now_ms - time_start_ms + (unsigned long long)now_hms*1000;
    if ( debug==9 )
    {
      cSF(prn_buff, INPUT_BYTES, "");
      time_long_2_str(t_ms, prn_buff);
      Serial.print("t_ms: "); Serial.print(prn_buff); Serial.print(" "); Serial.print(t_ms, 3); Serial.print(" = ");
      Serial.print(time_now_ms); Serial.print(" - "); Serial.print(time_start_ms, 3); Serial.print(" + "); Serial.print((unsigned long long)now_hms);
      time_long_2_str((unsigned long long)now_hms*1000, prn_buff); Serial.print(" "); Serial.println(prn_buff);

    }
    if ( acc_available_ ) time_acc_last_ = time_now_ms;
    if ( rot_available_ ) time_rot_last_ = time_now_ms;

}
