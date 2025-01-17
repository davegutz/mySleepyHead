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

extern boolean run;
extern int debug;

// Filter noise eye
void Sensors::filter_eye(const boolean reset)
{
    // IR Sensor
    eye_closed_ = LTST_Filter->calculate(eye_voltage_norm_, reset, min(T_eye_, NOM_DT_HEAD));
    eye_closed_confirmed_ = EyeClosedPer->calculate(eye_closed_, CLOSED_S, CLOSED_R, T_eye_, reset);

    // Eye buzz
    eye_buzz_ = eye_closed_confirmed_;
}

// Filter noise head
void Sensors::filter_head(const boolean reset)
{
    static int count = 0;
    static boolean turn = false;

    if ( reset || acc_available_ )
    {
        x_filt = X_Filt->calculate(x_raw, reset, TAU_FILT, min(T_acc_, NOM_DT_HEAD));
        y_filt = Y_Filt->calculate(y_raw, reset, TAU_FILT, min(T_acc_, NOM_DT_HEAD));
        z_filt = Z_Filt->calculate(z_raw, reset, TAU_FILT, min(T_acc_, NOM_DT_HEAD));
        g_filt = G_Filt->calculate(g_raw, reset, TAU_FILT, min(T_acc_, NOM_DT_HEAD));
        g_qrate = GQuietRate->calculate(g_raw-1., reset, min(T_acc_, MAX_T_Q_FILT));     
        g_quiet =GQuietFilt->calculate(g_qrate, reset, min(T_acc_, MAX_T_Q_FILT));
        static int count = 0;
    }

    if ( reset || rot_available_ )
    {
        a_filt = A_Filt->calculate(a_raw, reset, TAU_FILT, min(T_rot_, NOM_DT_HEAD));
        b_filt = B_Filt->calculate(b_raw, reset, TAU_FILT, min(T_rot_, NOM_DT_HEAD));
        c_filt = C_Filt->calculate(c_raw, reset, TAU_FILT, min(T_rot_, NOM_DT_HEAD));
        o_filt = O_Filt->calculate(o_raw, reset, TAU_FILT, min(T_rot_, NOM_DT_HEAD));
        o_qrate = OQuietRate->calculate(o_raw, reset, min(T_rot_, MAX_T_Q_FILT));     
        o_quiet =OQuietFilt->calculate(o_qrate, reset, min(T_rot_, MAX_T_Q_FILT));
    }

    // Mahony Tracking Filter
    if ( run )
      TrackFilter->updateIMU(a_raw, b_raw, c_raw, x_raw, y_raw, z_raw, T_acc_, reset);
    else
    {
      if ( turn )
      {
        TrackFilter->updateIMU(0, 0, 0, -0.679,  0.679,  0.281, T_acc_, reset);
      }
      else
        TrackFilter->updateIMU(0, 0, 0, 0, 0, 1, T_acc_, reset);
      if ( ++count > 400 ) count = 0;
      if ( count == 0 ) turn = !turn;
    }

    roll_filt = TrackFilter->getRoll();
    pitch_filt = TrackFilter->getPitch();
    yaw_filt = TrackFilter->getYaw();

    // Head sensor
    max_nod_f_ = max( abs(pitch_filt + delta_pitch_)- pitch_thr_f_, abs(roll_filt + delta_roll_) - roll_thr_f_ ) ;
    max_nod_p_ = max( abs(pitch_filt + delta_pitch_)- pitch_thr_p_, abs(roll_filt + delta_roll_) - roll_thr_p_ ) ;

    // Head buzz
    head_buzz_ = max_nod_p_ > 0.;
}

// Print publish
// plot pp3
void Sensors::plot_all()  // plot pp3
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

// plot pp1
void Sensors::plot_all_acc()  // plot pp1
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

// plot pp2
void Sensors::plot_all_rot()  // plot pp2
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

// plot pp7
void Sensors::plot_all_rpy()  // plot pp7
{
  float g_q = -2.;
  float g_q_s = -2.; 
  if ( g_is_quiet_ ) g_q = -1;
  if ( g_is_quiet_sure_ ) g_q_s = -1;
  Serial.print("T_acc*100:"); Serial.print(T_acc_*100., 3);
  Serial.print("\tx_raw:"); Serial.print(TrackFilter->ax(), 3);
  Serial.print("\ty_raw:"); Serial.print(TrackFilter->ay(), 3);
  Serial.print("\tz_raw:"); Serial.print(TrackFilter->az(), 3);
  // Serial.print("\tx_raw*200:"); Serial.print(x_raw*200+200, 3);
  // Serial.print("\ty_raw*200:"); Serial.print(y_raw*200+200, 3);
  // Serial.print("\tz_raw*200:"); Serial.print(z_raw*200+200, 3);
  // Serial.print("\ta_raw*200:"); Serial.print(a_raw*200+200, 3);
  // Serial.print("\tb_raw*200:"); Serial.print(b_raw*200+200, 3);
  // Serial.print("\tc_raw*200:"); Serial.print(c_raw*200+200, 3);
  // Serial.print("\troll_filt:"); Serial.print(roll_filt, 3);
  // Serial.print("\tpitch_filt:"); Serial.print(pitch_filt, 3);
  // Serial.print("\tyaw_filt:"); Serial.println(yaw_filt, 3);
  Serial.print("\troll_filt:"); Serial.print(TrackFilter->getRoll(), 3);
  Serial.print("\tpitch_filt:"); Serial.print(TrackFilter->getPitch(), 3);
  Serial.print("\tyaw_filt:"); Serial.print(TrackFilter->getYaw(), 3);
  // Serial.print("\thalfex:"); Serial.print(TrackFilter->getHalfex(), 3);
  // Serial.print("\thalfey:"); Serial.print(TrackFilter->getHalfey(), 3);
  // Serial.print("\thalfez:"); Serial.println(TrackFilter->getHalfez(), 3);
  Serial.print("\tq0:"); Serial.print(TrackFilter->q0(), 5);
  Serial.print("\tq1:"); Serial.print(TrackFilter->q1(), 5);
  Serial.print("\tq2:"); Serial.print(TrackFilter->q2(), 5);
  Serial.print("\tq3:"); Serial.println(TrackFilter->q3(), 5);
}

// plot pp0
void Sensors::plot_all_sum()  // plot pp0
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

// Print pp8
void Sensors::plot_buzz()  // print pp8
{
  Serial.print("eye_voltage:"); Serial.print(eye_voltage_norm_, 3);
  Serial.print("\tltstate:"); Serial.print(LTST_Filter->lt_state(), 3);
  Serial.print("\tststate:"); Serial.print(LTST_Filter->st_state(), 3);
  Serial.print("\tdltst:"); Serial.print(LTST_Filter->dltst(), 3);
  Serial.print("\teye_closed:"); Serial.print(eye_closed_);
  Serial.print("\tconf:"); Serial.print(eye_closed_confirmed_);
  Serial.print("\teye_buzz:"); Serial.print(eye_buzz_);
  Serial.print("\tmax_nod_f:"); Serial.print(max_nod_f_, 3);
  Serial.print("\tmax_nod_p:"); Serial.print(max_nod_p_, 3);
  Serial.print("\thead_buzz:"); Serial.println(head_buzz_);
}

// plot pp4
void Sensors::plot_quiet()  // plot pp4
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

// Print pp9 header
void Sensors::header_rapid_9()
{
  Serial.print("key_Rapid,");
  Serial.print("cTime,");
  Serial.print("v3v3,");
  Serial.print("eye_voltage_norm,");
  Serial.print("eye_closed,");
  Serial.print("eye_closed_confirmed,");
  Serial.print("max_nod_f,");
  Serial.print("max_nod_p,");
  Serial.print("head_buzz,");
  Serial.print("eye_buzz,");
  Serial.print("lt_state,");
  Serial.print("st_state,");
  Serial.print("dltst,");
  Serial.print("freeze,");
  Serial.println("");
}

// Print pp9
void Sensors::print_rapid_9(const float time)
{
  Serial.print(unit_.c_str()); Serial.print(",");
  Serial.print(time, 6); Serial.print(",");
  Serial.print(v3v3_, 4); Serial.print(",");
  Serial.print(eye_voltage_norm_, 4); Serial.print(",");
  Serial.print(eye_closed_); Serial.print(",");
  Serial.print(eye_closed_confirmed_); Serial.print(",");
  Serial.print(max_nod_f_, 3); Serial.print(",");
  Serial.print(max_nod_p_, 3); Serial.print(",");
  Serial.print(head_buzz_); Serial.print(",");
  Serial.print(eye_buzz_); Serial.print(",");
  Serial.print(LTST_Filter->lt_state(), 4); Serial.print(",");
  Serial.print(LTST_Filter->st_state(), 4); Serial.print(",");
  Serial.print(LTST_Filter->dltst(), 4); Serial.print(",");
  Serial.print(LTST_Filter->freeze()); Serial.print(",");
  Serial.println("");
}

// Manage IR sensor data streaming
void Sensors::print_rapid(const boolean reset, const boolean print_now, const float time_s)
{
  static uint8_t last_read_debug = 0;     // Remember first time with new debug to print headers
  if ( ( debug==9 ) )
  {
    if ( reset || (last_read_debug != debug) )
    {
      header_rapid_9();
    }
    if ( print_now )
    {
      print_rapid_9(time_s);
    }
  }
  last_read_debug = debug;
}

// Sample the IR (Eye)
void Sensors::sample_eye(const boolean reset, const unsigned long long time_eye_ms)
{
    time_eye_ms_ = time_eye_ms;

    // Reset
    if ( reset )
    {
        time_eye_last_ = time_eye_ms_ - EYE_DELAY;
    }

    // Half v3v3_nom = 3.3; v3v3_units = 4095;
    // v3v3_ = analogRead(v3v3Pin_) * v3v3_nom / float(v3v3_units) * 2.;
    v3v3_ = v3v3_nom;  // Disable.  Very small effect.  Don't bother wiring it.


    // IR Sensor
    eye_voltage_norm_ = analogRead(sensorPin_) * v3v3_nom / float(v3v3_units) - (v3v3_ - v3v3_nom) / D_EYE_VOLTAGE_D_VCC;
    T_eye_ = double(time_eye_ms - time_eye_last_) / 1000.;
    time_eye_last_ = time_eye_ms_;

}

// Sample the IMU (Head)
void Sensors::sample_head(const boolean reset, const unsigned long long time_now_ms, const unsigned long long time_start_ms, time_t now_hms)
{
    time_head_ms_ = time_now_ms;

    // Reset
    if ( reset )
    {
        time_rot_last_ = time_head_ms_ - HEAD_DELAY;
        time_acc_last_ = time_head_ms_ - HEAD_DELAY;
    }

    // Accelerometer
    if ( !reset && IMU.accelerationAvailable() )
    {
        IMU.readAcceleration(x_raw, y_raw, z_raw);
        y_raw *= -1.0;
        acc_available_ = true;
        g_raw = sqrt(x_raw*x_raw + y_raw*y_raw + z_raw*z_raw);
    }
    else acc_available_ = false;
    T_acc_ = double(time_head_ms_ - time_acc_last_) / 1000.;

    // Gyroscope
    if ( !reset && IMU.gyroscopeAvailable() )
    {
        IMU.readGyroscope(a_raw, b_raw, c_raw);
        a_raw *= -deg_to_rps;
        b_raw *= deg_to_rps;
        c_raw *= -deg_to_rps;
        rot_available_ = true;
        o_raw = sqrt(a_raw*a_raw + b_raw*b_raw + c_raw*c_raw);
    }
    else rot_available_ = false;
    T_rot_ = double(time_head_ms_ - time_rot_last_) / 1000.;

    // Time stamp
    t_ms = time_head_ms_ - time_start_ms + (unsigned long long)now_hms*1000;
    if ( debug==10 )
    {
      cSF(prn_buff, INPUT_BYTES, "");
      time_long_2_str(t_ms, prn_buff);
      Serial.print("t_ms: "); Serial.print(prn_buff); Serial.print(" "); Serial.print(t_ms, 3); Serial.print(" = ");
      Serial.print(time_head_ms_); Serial.print(" - "); Serial.print(time_start_ms, 3); Serial.print(" + "); Serial.print((unsigned long long)now_hms);
      time_long_2_str((unsigned long long)now_hms*1000, prn_buff); Serial.print(" "); Serial.println(prn_buff);

    }
    if ( acc_available_ ) time_acc_last_ = time_head_ms_;
    if ( rot_available_ ) time_rot_last_ = time_head_ms_;

}
