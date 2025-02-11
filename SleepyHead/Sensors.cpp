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


#include "constants.h"
#include "Sensors.h"
#include "src/Time/TimeLib.h"
#include "src/Tones/Tones.h"

extern int debug;

// Filter and calculate eye shut
void Sensors::filter_eye(const boolean reset, const float elapsed_time)
{
    elapsed_time_ = elapsed_time;
    glasses_off_ = eye_voltage_norm_ > GLASSES_OFF_VOLTAGE;
    glasses_reset_ = glasses_off_ || eye_reset_RLR_ || eye_reset_LRL_;
    // IR Sensor
    eye_reset_ = reset || GlassesOffPer->calculate( glasses_reset_, OFF_S, OFF_R, T_eye_, reset);
    eye_closed_ = LTST_Filter->calculate(eye_voltage_norm_, eye_reset_, min(T_eye_, MAX_DT_EYE), -FRZ_THR_POS);
    eye_closed_confirmed_ = EyeClosedPer->calculate(eye_closed_, eye_set_time_, eye_reset_time_, T_eye_, eye_reset_);
    eye_rate_ = EyeRateFilt->calculate(eye_voltage_norm_, reset, min(T_eye_, MAX_DT_EYE));
    boolean eye_ready_hold = EyeReadyHold->calculate(eye_reset_, EYE_RESET_CHIRP_HOLD, EYE_READY_CHIRP_HOLD, T_eye_, reset_);
    eye_ready_chirp_ = !eye_reset_ && eye_ready_hold;
    eye_reset_chirp_ = eye_reset_ && !eye_ready_hold;
    eye_buzz_ = eye_closed_confirmed_;
}

// Filter noise head
void Sensors::filter_head(const boolean reset, const boolean run)
{
  reset_ = reset;
  static int count = 0;
  static boolean turn = false;

  if ( reset || acc_available_ )
  {
      x_filt = X_Filt->calculate(x_raw, reset, TAU_FILT, min(T_acc_, MAX_DT_HEAD));
      y_filt = Y_Filt->calculate(y_raw, reset, TAU_FILT, min(T_acc_, MAX_DT_HEAD));
      z_filt = Z_Filt->calculate(z_raw, reset, TAU_FILT, min(T_acc_, MAX_DT_HEAD));
      g_filt = G_Filt->calculate(g_raw, reset, TAU_FILT, min(T_acc_, MAX_DT_HEAD));
      g_qrate = GQuietRate->calculate(g_raw-1., reset, min(T_acc_, MAX_DT_HEAD));     
      g_quiet = GQuietFilt->calculate(g_qrate, reset, wn_q_filt_, ZETA_Q_FILT, min(T_acc_, MAX_DT_HEAD));
      static int count = 0;
  }

  if ( reset || rot_available_ )
  {
      a_filt = A_Filt->calculate(a_raw, reset, TAU_FILT, min(T_rot_, MAX_DT_HEAD));
      b_filt = B_Filt->calculate(b_raw, reset, TAU_FILT, min(T_rot_, MAX_DT_HEAD));
      c_filt = C_Filt->calculate(c_raw, reset, TAU_FILT, min(T_rot_, MAX_DT_HEAD));
      o_filt = O_Filt->calculate(o_raw, reset, TAU_FILT, min(T_rot_, MAX_DT_HEAD));
      o_qrate = OQuietRate->calculate(o_raw, reset, min(T_rot_, MAX_DT_HEAD));     
      o_quiet = OQuietFilt->calculate(o_qrate, reset, wn_q_filt_, ZETA_Q_FILT, min(T_rot_, MAX_DT_HEAD));
  }

  // Mahony Tracking Filter
  if ( run )
    TrackFilter->updateIMU(a_raw, b_raw, c_raw, x_raw, y_raw, z_raw, T_acc_, reset);
  else  // Manual testing
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

  // RPY
  roll_deg = TrackFilter->getRollDeg() + delta_roll_;
  pitch_deg = TrackFilter->getPitchDeg() + delta_pitch_;
  yaw_deg = TrackFilter->getYawDeg();

  // Rates, deg/s
  roll_rate_ = RollRateFilt->calculate(roll_deg, reset, min(T_rot_, MAX_DT_HEAD));
  pitch_rate_ = PitchRateFilt->calculate(pitch_deg, reset, min(T_rot_, MAX_DT_HEAD));
  yaw_rate_ = YawRateFilt->calculate_wrap(yaw_deg, reset, min(T_rot_, MAX_DT_HEAD), YAW_WRAP_DETECT, YAW_WRAP_MAG);
  // Serial.print("YawRate: "); YawRateFilt->repr();

  // Head sensor
  max_nod_f_ = max( abs(pitch_deg)- pitch_thr_f_, abs(roll_deg) - roll_thr_f_ ) ;
  max_nod_p_ = max( abs(pitch_deg)- pitch_thr_p_, abs(roll_deg) - roll_thr_p_ ) ;

  head_reset_ = reset || HeadShakePer->calculate( ( !o_is_quiet_sure_ && !g_is_quiet_sure_ ), SHAKE_S, SHAKE_R, T_acc_, reset);
  boolean head_ready_hold = HeadReadyHold->calculate(head_reset_, HEAD_RESET_CHIRP_HOLD, HEAD_READY_CHIRP_HOLD, T_acc_, reset_);
  head_ready_chirp_ = !head_reset_ && head_ready_hold;
  head_reset_chirp_ = head_reset_ && !head_ready_hold;

  max_nod_f_confirmed_ = HeadNodPerF->calculate(max_nod_f_ > 0 && !head_reset_, head_set_time_, head_reset_time_, T_acc_, head_reset_);
  max_nod_p_confirmed_ = HeadNodPerP->calculate(max_nod_p_ > 0 && !head_reset_, head_set_time_, head_reset_time_, T_acc_, head_reset_);

  // Head buzz
  head_buzz_f_ = max_nod_f_confirmed_;
  head_buzz_p_ = max_nod_p_confirmed_;

  // Yaw head wag to reset eye sensors
  eye_reset_RLR_ = YawWagRLR->calculate(reset, T_rot_, yaw_rate_<=YAW_RATE_LOW, yaw_rate_>=YAW_RATE_HIGH);
  eye_reset_LRL_ = YawWagLRL->calculate(reset, T_rot_, yaw_rate_>=YAW_RATE_HIGH, yaw_rate_<=YAW_RATE_LOW);

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
  Serial.print("\to_filt:"); Serial.print(o_filt, 3);
  Serial.println("");
}

// plot pp1
void Sensors::plot_all_acc()  // plot pp1
{
  float g_q = -2.;
  float g_q_s = -2.; 
  if ( g_is_quiet_ ) g_q = -1;
  if ( g_is_quiet_sure_ ) g_q_s = -1;
  Serial.print("T_acc*10:"); Serial.print(T_acc_*10., 3);
  Serial.print("\tx_filt:"); Serial.print(x_filt, 3);
  Serial.print("\ty_filt:"); Serial.print(y_filt, 3);
  Serial.print("\tz_filt:"); Serial.print(z_filt, 3);
  Serial.print("\tg_filt-1:"); Serial.print(g_filt-1., 3);
  Serial.print("\tg_is_quiet-2:"); Serial.print(g_q, 3);
  Serial.print("\tg_is_quiet_sure-2:"); Serial.print(g_q_s, 3);
  Serial.println("");
}

// plot pp2
void Sensors::plot_all_rot()  // plot pp2
{
  float o_q = -4.;
  float o_q_s = -4.; 
  if ( o_is_quiet_ ) o_q = -3;
  if ( o_is_quiet_sure_ ) o_q_s = -3;
  Serial.print("T_rot_*10:"); Serial.print(T_rot_*10., 3);
  Serial.print("\ta_filt:"); Serial.print(a_filt, 3);
  Serial.print("\tb_filt:"); Serial.print(b_filt, 3);
  Serial.print("\tc_filt:"); Serial.print(c_filt, 3);
  Serial.print("\to_filt:"); Serial.print(o_filt, 3);
  Serial.print("\to_is_quiet-4:"); Serial.print(o_q, 3);
  Serial.print("\to_is_quiet_sure-4:"); Serial.print(o_q_s, 3);
  Serial.println("");
}

// plot pp7
void Sensors::plot_all_rpy()  // plot pp7
{
  Serial.print("\troll_filt:"); Serial.print(TrackFilter->getRollDeg() + delta_roll_, 3);
  Serial.print("\tpitch_filt:"); Serial.print(TrackFilter->getPitchDeg() + delta_pitch_, 3);
  Serial.print("\tyaw_filt:"); Serial.print(TrackFilter->getPitchDeg() + delta_pitch_, 3);
  Serial.print("\troll_rate:"); Serial.print(roll_rate_, 3);
  Serial.print("\tpitch_rate:"); Serial.print(pitch_rate_, 3);
  Serial.print("\tyaw_rate:"); Serial.print(yaw_rate_, 3);
  Serial.print("\teye_rate:"); Serial.print(eye_rate_);
  Serial.println("");
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
  Serial.print("\to_is_quiet_sure-4:"); Serial.print(o_q_s, 3);
  Serial.println("");
}

// Plot pp8
void Sensors::plot_head_buzz()  // plot pp8
{
  // float g_q_s = -2.; 
  // if ( g_is_quiet_sure_ ) g_q_s = -1;
  // float o_q_s = -4.; 
  // if ( o_is_quiet_sure_ ) o_q_s = -3;
  float all_quiet = -2;
  if ( g_is_quiet_sure_ && g_is_quiet_sure_ ) all_quiet = -1;
  Serial.print("head_reset:"); Serial.print(head_reset_, 3);
  // Serial.print("\tg_is_quiet_sure-2:"); Serial.print(g_q_s, 3);
  // Serial.print("\to_is_quiet_sure-4:"); Serial.print(o_q_s, 3);
  Serial.print("\tall_quiet-2:"); Serial.print(all_quiet);
  Serial.print("\tnod_f/10:"); Serial.print(max_nod_f_/10., 3);
  Serial.print("\tnod_p/10:"); Serial.print(max_nod_p_/10., 3);
  Serial.print("\thead_buzz_f:"); Serial.print(head_buzz_f_);
  Serial.print("\thead_buzz_p:"); Serial.print(head_buzz_p_);
  Serial.print("\teye_cf+3:"); Serial.print(LTST_Filter->cf()+3, 3);
  Serial.print("\teye_buzz+5:"); Serial.print(eye_buzz_+5);
  // Serial.print("\t HeadShakePer:"); HeadShakePer->repr();
  Serial.println("");
}

// Plot pp9
void Sensors::plot_eye_buzz()  // plot pp9
{
  Serial.print("eye_voltage:"); Serial.print(eye_voltage_norm_, 3);
  Serial.print("\tltstate:"); Serial.print(LTST_Filter->lt_state(), 3);
  Serial.print("\tststate:"); Serial.print(LTST_Filter->st_state(), 3);
  Serial.print("\tdltst:"); Serial.print(LTST_Filter->dltst(), 3);
  Serial.print("\teye_closed:"); Serial.print(eye_closed_);
  Serial.print("\tconf:"); Serial.print(eye_closed_confirmed_);
  Serial.print("\teye_buzz+5:"); Serial.print(eye_buzz_+5);
  Serial.print("\tnod_f/10:"); Serial.print(max_nod_f_/10., 3);
  Serial.print("\tnod_p/10:"); Serial.print(max_nod_p_/10., 3);
  Serial.print("\teye_cf+3:"); Serial.print(LTST_Filter->cf()+3, 3);
  Serial.print("\teye_reset+2:"); Serial.print(eye_reset_+2);
  Serial.print("\teye_ready_chirp+4:"); Serial.print(eye_ready_chirp_+4);
  Serial.print("\teye_reset_chirp+4:"); Serial.print(eye_reset_chirp_+4);
  Serial.println("");
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
  Serial.print("T_rot_*10:"); Serial.print(T_rot_*10., 3);
  Serial.print("\to_filt-2:"); Serial.print(o_filt-2, 3);
  Serial.print("\to_quiet-2:"); Serial.print(o_quiet-2, 3);
  Serial.print("\to_is_quiet_sure-4:"); Serial.print(o_q_s, 3);
  Serial.print("\t\tT_acc*10:"); Serial.print(T_acc_*10., 3);
  Serial.print("\tg_filt+2:"); Serial.print(g_filt-1.+2, 3);
  Serial.print("\tg_quiet+2:"); Serial.print(g_quiet+2, 3);
  Serial.print("\tg_is_quiet_sure-2:"); Serial.print(g_q_s, 3);
  Serial.println("");
}

// Print publish
void Sensors::plot_quiet_raw()  // pp5
{
  Serial.print("o_quiet:"); Serial.print(o_quiet, 3);
  Serial.print("\t\tg_quiet:"); Serial.print(g_quiet, 3);
  Serial.println("");
}

// Print publish
void Sensors::plot_total()  // pp6
{
  Serial.print("T_rot_*10:"); Serial.print(T_rot_*10., 3);
  Serial.print("\to_filt:"); Serial.print(o_filt, 3);
  Serial.print("\t\tT_acc_*10:"); Serial.print(T_acc_*10., 3);
  Serial.print("\tg_filt:"); Serial.print(g_filt-1., 3);
  Serial.println("");
}


void Sensors::plot_yaw_reset()  // pp12
{
  Serial.print("yaw/100:"); Serial.print( (TrackFilter->getYawDeg()-180.) / 100.);
  Serial.print("\tyaw_rate/100:"); Serial.print(yaw_rate_/100.);
  Serial.print("\tyawRLR-4:"); Serial.print(eye_reset_RLR_-4.);
  Serial.print("\tyawLRL-2:"); Serial.print(eye_reset_LRL_-2.);
  Serial.print("\treset+2:"); Serial.print(reset_+2.);
  Serial.print("\tglasses_off-6:"); Serial.print(glasses_off_-6.);
  Serial.print("\tglasses_reset-8:"); Serial.print(glasses_off_-8.);
  Serial.print("\teye_reset+2:"); Serial.print(eye_reset_+2.);
  Serial.print("\tcf-8:"); Serial.print(LTST_Filter->cf()-8.);
  Serial.println("");
}

void Sensors::pretty_print_head()
{
  Serial.println("Head:");
  Serial.print("\thead_reset\t"); Serial.println(head_reset_, 3);
  Serial.print("\tpitch\t\t"); Serial.println(pitch_deg, 3);
  Serial.print("\troll\t\t"); Serial.println(roll_deg, 3);
  Serial.print("\tmax_nod_p\t"); Serial.println(max_nod_p_, 3);
  Serial.print("\tmax_nod_f\t"); Serial.println(max_nod_f_, 3);
  Serial.print("\tnod_p_conf\t"); Serial.println(max_nod_p_confirmed_, 2);
  Serial.print("\tnod_f_conf\t"); Serial.println(max_nod_f_confirmed_, 2);
  Serial.print("\thead_buzz_f\t"); Serial.println(head_buzz_f_, 2);
  Serial.print("\thead_buzz_p\t"); Serial.println(head_buzz_p_, 2);
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
  Serial.print(g_is_quiet_sure_, 3);
  Serial.println("");
}

void Sensors::print_all_header()
{
  Serial.println("T_rot_*\ta_filt\tb_filt\tc_filt\to_filt\to_is_quiet\to_is_quiet_sure\t\tT_acc\tx_filt\ty_filt\tz_filt\tg_filt\tg_is_quiet\tg_is_quiet_sure");
}

// Print default header key
void Sensors::print_default_hdr(const uint8_t plot_num)
{
 switch ( plot_num )
  {
    case 0 ... 9:
      Serial.println("header embedded in each line of output for Arduino Serial Plotter use");
      break;
    case 10:
      Serial.println("pp10");
      print_rapid(true, time_eye_s());  // pp10
      break;
    case 11:
      Serial.println("pp11");
      print_Mahony(true, time_eye_s());  // pp11
      break;
    case 12:
      Serial.println("pp12");
      plot_yaw_reset();  // pp12
      break;
    default:
      Serial.println("unknown input; check code");
      break;
  }
}

// Print pp10 header
void Sensors::print_rapid_10_hdr()
{
  Serial.print("key_Rapid,");
  Serial.print("reset,");
  Serial.print("elapsed_time,");
  Serial.print("cTime,");
  Serial.print("head_reset,");
  Serial.print("eye_reset,");
  Serial.print("eye_voltage_norm,");
  Serial.print("a_raw,");
  Serial.print("b_raw,");
  Serial.print("c_raw,");
  Serial.print("x_raw,");
  Serial.print("y_raw,");
  Serial.print("z_raw,");
  Serial.print("eye_closed,");
  Serial.print("eye_closed_confirmed,");
  Serial.print("max_nod_f,");
  Serial.print("max_nod_f_confirmed,");
  Serial.print("max_nod_p,");
  Serial.print("max_nod_p_confirmed,");
  Serial.print("delta_pitch,");
  Serial.print("pitch_deg,");
  Serial.print("delta_roll,");
  Serial.print("roll_deg,");
  Serial.print("yaw_deg,");
  Serial.print("head_buzz_f,");
  Serial.print("head_buzz_p,");
  Serial.print("eye_buzz,");
  Serial.print("lt_state,");
  Serial.print("st_state,");
  Serial.print("dltst,");
  Serial.print("cf,");
  Serial.print("freeze,");
  Serial.print("v3v3,");
  Serial.print("head_buzz,");
  Serial.print("g_quiet,");
  Serial.print("o_quiet,");
  Serial.print("g_is_quiet_sure,");
  Serial.print("o_is_quiet_sure,");
  Serial.print("g_is_quiet,");
  Serial.print("o_is_quiet,");
  Serial.print("roll_rate,");
  Serial.print("pitch_rate,");
  Serial.print("yaw_rate,");
  Serial.print("eye_rate,");
  Serial.println("");
  Serial.flush();
  delay(100);
}

// Print pp10
void Sensors::print_rapid_10(const float time)  // pp10
{
  Serial.print(unit_.c_str()); Serial.print(",");
  Serial.print(reset_); Serial.print(",");
  Serial.print(elapsed_time_, 6); Serial.print(",");
  Serial.print(time, 6); Serial.print(",");
  Serial.print(head_reset_); Serial.print(",");
  Serial.print(eye_reset_); Serial.print(",");
  Serial.print(eye_voltage_norm_, 6); Serial.print(",");
  Serial.print(a_raw, 6); Serial.print(",");
  Serial.print(b_raw, 6); Serial.print(",");
  Serial.print(c_raw, 6); Serial.print(",");
  Serial.print(x_raw, 6); Serial.print(",");
  Serial.print(y_raw, 6); Serial.print(",");
  Serial.print(z_raw, 6); Serial.print(",");
  Serial.print(eye_closed_); Serial.print(",");
  Serial.print(eye_closed_confirmed_); Serial.print(",");
  Serial.print(max_nod_f_, 3); Serial.print(",");
  Serial.print(max_nod_f_confirmed_, 3); Serial.print(",");
  Serial.print(max_nod_p_, 3); Serial.print(",");
  Serial.print(max_nod_p_confirmed_, 3); Serial.print(",");
  Serial.print(delta_pitch_, 3); Serial.print(",");
  Serial.print(pitch_deg, 5); Serial.print(",");
  Serial.print(delta_roll_, 5); Serial.print(",");
  Serial.print(roll_deg, 5); Serial.print(",");
  Serial.print(yaw_deg, 5); Serial.print(",");
  Serial.print(head_buzz_f_); Serial.print(",");
  Serial.print(head_buzz_p_); Serial.print(",");
  Serial.print(eye_buzz_); Serial.print(",");
  Serial.print(LTST_Filter->lt_state(), 5); Serial.print(",");
  Serial.print(LTST_Filter->st_state(), 5); Serial.print(",");
  Serial.print(LTST_Filter->dltst(), 5); Serial.print(",");
  Serial.print(LTST_Filter->cf(), 5); Serial.print(",");
  Serial.print(LTST_Filter->freeze()); Serial.print(",");
  Serial.print(v3v3_); Serial.print(",");
  Serial.print(head_buzz_f_); Serial.print(",");
  Serial.print(g_quiet, 5); Serial.print(",");
  Serial.print(o_quiet, 5); Serial.print(",");
  Serial.print(g_is_quiet_sure_); Serial.print(",");
  Serial.print(o_is_quiet_sure_); Serial.print(",");
  Serial.print(g_is_quiet_); Serial.print(",");
  Serial.print(o_is_quiet_); Serial.print(",");
  Serial.print(roll_rate_, 5); Serial.print(",");
  Serial.print(pitch_rate_, 5); Serial.print(",");
  Serial.print(yaw_rate_, 5); Serial.print(",");
  Serial.print(eye_rate_, 5); Serial.print(",");
  Serial.println("");
}

// Print pp11 header
void Sensors::print_rapid_11_hdr()
{
  Serial.print("key_Rapid,");
  Serial.print("reset,");
  Serial.print("cTime,");
  Serial.print("head_reset,");
  Serial.print("a_raw,");
  Serial.print("b_raw,");
  Serial.print("c_raw,");
  Serial.print("x_raw,");
  Serial.print("y_raw,");
  Serial.print("z_raw,");
  Serial.print("halfex,");
  Serial.print("halfey,");
  Serial.print("halfez,");
  Serial.print("halfvx,");
  Serial.print("halfvy,");
  Serial.print("halfvz,");
  Serial.print("ifb_x,");
  Serial.print("ifb_y,");
  Serial.print("ifb_z,");
  Serial.print("q0,");
  Serial.print("q1,");
  Serial.print("q2,");
  Serial.print("q3,");
  Serial.print("roll,");
  Serial.print("pitch,");
  Serial.print("yaw,");
  Serial.print("roll_deg,");
  Serial.print("pitch_deg,");
  Serial.print("yaw_deg,");
  Serial.print("twoKi,");
  Serial.print("twoKp,");
  Serial.println("");
  Serial.flush();
  delay(100);
}

// Print pp11
void Sensors::print_rapid_11(const float time)  // pp11
{
  Serial.print(unit_.c_str()); Serial.print(",");
  Serial.print(reset_); Serial.print(",");
  Serial.print(time, 6); Serial.print(",");
  Serial.print(head_reset_); Serial.print(",");
  Serial.print(a_raw, 5); Serial.print(",");
  Serial.print(b_raw, 5); Serial.print(",");
  Serial.print(c_raw, 5); Serial.print(",");
  Serial.print(x_raw, 5); Serial.print(",");
  Serial.print(y_raw, 5); Serial.print(",");
  Serial.print(z_raw, 5); Serial.print(",");
  Serial.print(TrackFilter->getHalfex(), 6); Serial.print(",");
  Serial.print(TrackFilter->getHalfey(), 6); Serial.print(",");
  Serial.print(TrackFilter->getHalfez(), 6); Serial.print(",");
  Serial.print(TrackFilter->getHalfvx(), 6); Serial.print(",");
  Serial.print(TrackFilter->getHalfvy(), 6); Serial.print(",");
  Serial.print(TrackFilter->getHalfvz(), 6); Serial.print(",");
  Serial.print(TrackFilter->getIntegralFBx(), 6); Serial.print(",");
  Serial.print(TrackFilter->getIntegralFBy(), 6); Serial.print(",");
  Serial.print(TrackFilter->getIntegralFBz(), 6); Serial.print(",");
  Serial.print(TrackFilter->getQ0(), 5); Serial.print(",");
  Serial.print(TrackFilter->getQ1(), 5); Serial.print(",");
  Serial.print(TrackFilter->getQ2(), 5); Serial.print(",");
  Serial.print(TrackFilter->getQ3(), 5); Serial.print(",");
  Serial.print(TrackFilter->getRollRadians(), 5); Serial.print(",");
  Serial.print(TrackFilter->getPitchRadians(), 5); Serial.print(",");
  Serial.print(TrackFilter->getYawRadians(), 5); Serial.print(",");
  Serial.print(TrackFilter->getRollDeg(), 4); Serial.print(",");
  Serial.print(TrackFilter->getPitchDeg(), 4); Serial.print(",");
  Serial.print(TrackFilter->getYawDeg(), 4); Serial.print(",");
  Serial.print(TrackFilter->getTwoKi(), 3); Serial.print(",");
  Serial.print(TrackFilter->getTwoKp(), 3); Serial.print(",");
  Serial.println("");
}

// Manage Mahony data streaming
void Sensors::print_Mahony(const boolean print_hdr, const float time_s)  // pp11
{
    if ( print_hdr ) print_rapid_11_hdr();  // pp11
    print_rapid_11(time_s);  // pp11
}

// Manage IR sensor data streaming
void Sensors::print_rapid(const boolean print_hdr, const float time_s)  // pp10
{
    if ( print_hdr ) print_rapid_10_hdr();  // pp10
    print_rapid_10(time_s);  // pp10
}

// Detect no signal present based on detection of quiet signal.
// Research by sound industry found that 2-pole filtering is the sweet spot between seeing noise
// and actual motion without 'guilding the lily'
void Sensors::quiet_decisions(const boolean reset, const float o_quiet_thr, const float g_quiet_thr)
{
  o_is_quiet_ = abs(o_quiet) <= o_quiet_thr;  // o_filt is rss
  o_is_quiet_sure_ = OQuietPer->calculate(o_is_quiet_, QUIET_S, QUIET_R, T_rot_, reset);
  g_is_quiet_ = abs(g_quiet) <= g_quiet_thr;  // g_filt is rss
  g_is_quiet_sure_ = GQuietPer->calculate(g_is_quiet_, QUIET_S, QUIET_R, T_acc_, reset);
  static int count = 0;
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
    // // dag
    // if ( reset ) eye_voltage_norm_ = 0.;
    // else eye_voltage_norm_ = 1.;
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
    if ( IMU.accelerationAvailable() )
    {
        IMU.readAcceleration(x_raw, y_raw, z_raw);
        y_raw *= -1.0;
        acc_available_ = true;
        g_raw = sqrt(x_raw*x_raw + y_raw*y_raw + z_raw*z_raw);
    }
    else acc_available_ = false;
    T_acc_ = double(time_head_ms_ - time_acc_last_) / 1000.;

    // Gyroscope
    if ( IMU.gyroscopeAvailable() )
    {
        IMU.readGyroscope(a_raw, b_raw, c_raw);
        rot_available_ = true;
        o_raw = sqrt(a_raw*a_raw + b_raw*b_raw + c_raw*c_raw);
    }
    else rot_available_ = false;
    T_rot_ = double(time_head_ms_ - time_rot_last_) / 1000.;

    if ( acc_available_ ) time_acc_last_ = time_head_ms_;
    if ( rot_available_ ) time_rot_last_ = time_head_ms_;

}
