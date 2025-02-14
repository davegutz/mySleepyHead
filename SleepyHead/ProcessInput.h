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
#include "SleepyHead.h"
#include "constants.h"

uint8_t plot_num = plot_num_def;
boolean monitoring = false;
boolean plotting_all = plotting_all_def;
boolean enable_motor = true;
CommandPars cp = CommandPars();       // Various control parameters commanding at system level.  Initialized on start up.  Not retained.
extern CommandPars cp;

// Worker bee code
void process_input_str(Sensors *Sen, float *g_quiet_thr, float *o_quiet_thr, boolean *reset, boolean *run)
{
    if ( cp.inp_str.length() )
    {
      // Now we know the letters
      Serial.println(""); Serial.print("cp.inp_str: "); Serial.println(cp.inp_str);
      char letter_0 = '\0';
      char letter_1 = '\0';
      letter_0 = cp.inp_str.charAt(0);
      letter_1 = cp.inp_str.charAt(1);
      Serial.print(" letter_0: "); Serial.print(letter_0); Serial.print(" letter_1: "); Serial.println(letter_1);
      float f_value = cp.inp_str.substring(2).toFloat();
      int i_value = int(f_value);
      Serial.print(" i_value: "); Serial.print(i_value); Serial.print(" f_value: "); Serial.println(f_value);
      Serial.println("");

      switch ( letter_0 )
      {
        case ( 'a' ):  // a - adjust
          switch ( letter_1 )
          {
            case ( 'p' ):  // ap - proportional gain
              Serial.print("Mahony prop gain from "); Serial.print(Sen->TrackFilter->getTwoKp(), 3);
              Sen->TrackFilter->setKp(f_value);
              Serial.print(" to "); Serial.println(Sen->TrackFilter->getTwoKp(), 3);
              break;
            case ( 'i' ):  // ai - integral gain
              Serial.print("Mahony int gain from "); Serial.print(Sen->TrackFilter->getTwoKi(), 3);
              Sen->TrackFilter->setTwoKi(f_value);
              Serial.print(" to "); Serial.println(Sen->TrackFilter->getTwoKi(), 3);
              break;
            case ( 'l' ):  // al - LTST fault threshold
              Serial.print("LTST fault threshold from "); Serial.print(Sen->LTST_Filter->get_fault_thr_pos(), 3);
              Sen->LTST_Filter->set_fault_thr_pos(f_value);
              Serial.print(" to "); Serial.println(Sen->LTST_Filter->get_fault_thr_pos(), 3);
              break;
            case ( 'R' ):  // aR - eye reset time
              Serial.print("Eye reset time from "); Serial.print(Sen->get_eye_reset_time(), 2);
              Sen->set_eye_reset_time(f_value);
              Serial.print(" to "); Serial.println(Sen->get_eye_reset_time(), 2);
              break;
            case ( 'r' ):  // ar - roll threshold
              Serial.print("Roll threshold from "); Serial.print(Sen->roll_thr(), 3);
              Sen->roll_thr(f_value);
              Serial.print(" to "); Serial.println(Sen->roll_thr(), 3);
              break;
            case ( 'S' ):  // aS - eye set time
              Serial.print("Eye set time from "); Serial.print(Sen->get_eye_set_time(), 2);
              Sen->set_eye_set_time(f_value);
              Serial.print(" to "); Serial.println(Sen->get_eye_set_time(), 2);
              break;
            case ( 't' ):  // at - pitch threshold
              Serial.print("Pitch threshold from "); Serial.print(Sen->pitch_thr(), 3);
              Sen->pitch_thr(f_value);
              Serial.print(" to "); Serial.println(Sen->pitch_thr(), 3);
              break;
            case ( 'z' ):  // az - LTST freeze threshold
              Serial.print("LTST freeze threshold from "); Serial.print(Sen->LTST_Filter->get_freeze_thr_pos(), 3);
              Sen->LTST_Filter->set_freeze_thr_pos(f_value);
              Serial.print(" to "); Serial.println(Sen->LTST_Filter->get_freeze_thr_pos(), 3);
              break;
           default:
              Serial.print(letter_0); Serial.println(" unknown");
              break;
          }
          break;
        case ( 'e' ):  // e - enable
          switch ( letter_1 )
          {
            case ( 'm' ):  // em enable motor buzz
              enable_motor = !enable_motor;
              break;
           default:
              Serial.print(letter_0); Serial.println(" unknown");
              break;
          }
          break;          
        case ( 'f' ):  // f - filter adjust
          switch ( letter_1 )
          {
            case ( 'l' ):  // fl - tau_lt
              Serial.print("Tau lt from "); Serial.print(Sen->LTST_Filter->get_tau_lt(), 3);
              Sen->LTST_Filter->set_tau_lt(f_value);
              Serial.print(" to "); Serial.println(Sen->LTST_Filter->get_tau_lt(), 3);
              break;
            case ( 's' ):  // fs - tau_st
              Serial.print("Tau st from "); Serial.print(Sen->LTST_Filter->get_tau_st(), 3);
              Sen->LTST_Filter->set_tau_st(f_value);
              Serial.print(" to "); Serial.println(Sen->LTST_Filter->get_tau_st(), 3);
              break;
           default:
              Serial.print(letter_0); Serial.println(" unknown");
              break;
          }
          break;
        case ( 'H' ):  // H - header print legend top row and one line of data
          Sen->print_default_hdr(plot_num_def);
          break;
        case ( 'h' ):  // h  - help
          plotting_all = false;
          monitoring = false;
          Serial.println("a?<val> - adjust");
          Serial.print("\t ap = Mahony proportional gain ("); Serial.print(Sen->TrackFilter->getTwoKp(), 3); Serial.println(")");
          Serial.print("\t ai = Mahony integral gain ("); Serial.print(Sen->TrackFilter->getTwoKi(), 3); Serial.println(")");
          Serial.print("\t al = LTST fault thr ("); Serial.print(Sen->LTST_Filter->get_fault_thr_pos(), 3); Serial.println(")");
          Serial.print("\t aR = eye RESET time, s ("); Serial.print(Sen->get_eye_reset_time(), 3); Serial.println(")");
          Serial.print("\t aS = eye SET time, s ("); Serial.print(Sen->get_eye_set_time(), 3); Serial.println(")");
          Serial.print("\t ar = roll thr ("); Serial.print(Sen->roll_thr(), 3); Serial.println(")");
          Serial.print("\t at = pitch thr ("); Serial.print(Sen->pitch_thr(), 3); Serial.println(")");
          Serial.print("\t az = LTST freeze thr ("); Serial.print(Sen->LTST_Filter->get_freeze_thr_pos(), 3); Serial.println(")");
          Serial.println("e? - enable toggles");
          Serial.print("\t em - enable motor buzz("); Serial.print(enable_motor); Serial.println(")");
          Serial.println("f?<val> - filter adjust");
          Serial.print("\t fl = tau_lt ("); Serial.print(Sen->LTST_Filter->get_tau_lt(), 3); Serial.println(")");
          Serial.print("\t fs = tau_st ("); Serial.print(Sen->LTST_Filter->get_tau_st(), 3); Serial.println(")");
          Serial.println("h - this help");
          Serial.println("H - print header and one line of data for the default data stream that starts automatically on startup");
          Serial.println("P? - Print stuff");
          Serial.println("\t PL - LTST Filter");
          Serial.println("\t PY - Yaw Rate Filter");
          Serial.print("pp? - plot all version ("); Serial.print(plot_num); Serial.println(")");
          Serial.println("\t pp-1 - stop plotting");
          Serial.println("\t pp0 - summ:      (g_raw, g_filt, g_quiet, q_is_quiet_sure, o_raw, o_filt, o_quiet, o_is_quiet_sure)");
          Serial.println("\t pp1 - acc:       (T_acc, x_filt, y_filt, z_filt, g_filt, g_is_quiet, g_is_quiet_sure)");
          Serial.println("\t pp2 - rot:       (T_rot, a_filt, b_filt, c_filt, o_filt, o_is_quiet, o_is_quiet_sure)");
          Serial.println("\t pp3 - all sen:   (x_filt, y_filt, z_filt, g_filt, a_filt, b_filt, c_filt, o_filt)");
          Serial.println("\t pp4 - quiet:     (T_rot, o_filt, o_quiet, o_is_quiet_sure, T_acc, g_filt, g_quiet, g_is_quiet_sure)");
          Serial.println("\t pp5 - quiet raw: (o_quiet, g_quiet)");
          Serial.println("\t pp6 - total:     (T_rot, o_filt, T_acc, g_filt)");
          Serial.println("\t pp7 - rpy:       (roll_deg, pitch_deg, roll_rate, pitch_rate, yaw_rate, eye_rate)");
          Serial.println("\t pp8 - head buzz: (g_is_quiet_sure, o_is_quiet_sure, max_nod_f, max_nod_p, head_buzz, cf, eye_tone)");
          Serial.println("\t pp9 - eye buzz:  (ltstate, ststate, dltst, eye_closed, eye_closed_confirmed, eye_tone, max_nod_f, max_nod_p, eye_cf, eye_reset)");
          Serial.println("\t pp10- stream:    (key_Rapid, cTime, v3v3, eye_voltage_norm, eye_closed, eye_closed_confirmed, max_nod_f, max_nod_p, head_buzz, eye_tone, lt_state, st_state, dltst, freeze)");
          Serial.println("\t pp11 - Mahony:   (...)");
          Serial.println("\t pp12 - yaw reset:(yaw, yaw_rate, yawRLR, yawLRL, eye_reset, glasses_off, glasses_reset)");
          Serial.println("t?<val> - trim attitude");
          Serial.print("\t tg = G quiet thr, small more sensitive ("); Serial.print(*g_quiet_thr, 3); Serial.println(")");
          Serial.print("\t to = O angular speed quiet thr, small more sensitive ("); Serial.print(*o_quiet_thr, 3); Serial.println(")");
          Serial.print("\t tp = pitch bias nodding ("); Serial.print(Sen->get_delta_pitch(), 3); Serial.println(")");
          Serial.print("\t tr = roll bias tilting ("); Serial.print(Sen->get_delta_roll(), 3); Serial.println(")");
          Serial.print("\t tw = noise filter cutoff ("); Serial.print(Sen->get_wn_q_filt(), 3); Serial.println(")");
          Serial.println("vv?  - verbosity debug level (not implemented)");
          Serial.println("x?  - manually test Mahony tracking filter ( true = real time, false = pulse )");
          break;
        case ( 'P' ):  // P - print
          switch ( letter_1 )
          {
            case ( 'H' ):  // PH - Print Head data
              Sen->pretty_print_head();
              break;
            case ( 'L' ):  // PL - Print LTST Filter
              Sen->LTST_Filter->pretty_print();
              break;
            case ( 'Y' ):  // PY - Print Yaw Rate Filter
              Sen->print_yaw_rate_filt();
              break;
           default:
              Serial.print(letter_0); Serial.println(" unknown");
              break;
          }
          break;          
        case ( 'p' ):  // p - plot
          switch ( letter_1 )
          {
            case ( 'p' ):  // pp - plot all filtered pp0, pp1, pp2, pp3, pp4, pp5, pp6, pp7, pp8, pp9, pp10, pp11, pp12
              switch ( i_value )
              {
                case 0 ... 12:
                  plot_num = i_value;
                  plotting_all = true;
                  monitoring = false;
                  break;
                default:
                  Serial.println("plot number unknown enter plot number e.g. pp0 (sum), pp1 (acc), pp2 (rot), pp3 (all), pp4 (quiet), pp5 (quiet raw), pp6 (total), pp7 (roll-pitch-yaw), pp8 (head_buzz), pp9 (eye_tone), pp10 (stream), pp11 (Mahony), pp12 (yaw reset)");
                  plotting_all = false;
                  break;
              }
              break;
            default:
              Serial.print(cp.inp_str.charAt(1)); Serial.print(" for "); Serial.print(cp.inp_str); Serial.println(" unknown");
              break;
          }
          break;
        case ( 't' ):  // t - trim
          switch ( letter_1 )
          {
            case ( 'g' ):  // tg - G quiet threshold
              Serial.print("G quiet threshold from "); Serial.print(*g_quiet_thr, 3);
              *g_quiet_thr = f_value;
              Serial.print(" to "); Serial.print(*g_quiet_thr, 3);
              break;
            case ( 'o' ):  // to - Angular speed quiet threshold
              Serial.print("O quiet threshold from "); Serial.print(*o_quiet_thr, 3);
              *o_quiet_thr = f_value;
              Serial.print(" to "); Serial.print(*o_quiet_thr, 3);
              break;
            case ( 'p' ):  // tp - trim pitch
              Serial.print("Pitch bias "); Serial.print(Sen->get_delta_pitch(), 3);
              Sen->set_delta_pitch(f_value);
              Serial.print(" to "); Serial.println(Sen->get_delta_pitch(), 3);
              break;
            case ( 'r' ):  // tr - trim roll
              Serial.print("Roll bias "); Serial.print(Sen->get_delta_roll(), 3);
              Sen->set_delta_roll(f_value);
              Serial.print(" to "); Serial.println(Sen->get_delta_roll(), 3);
              break;
            case ( 'w' ):  // tw - wn_q_filt
              Serial.print("Wn noise filter "); Serial.print(Sen->get_wn_q_filt(), 3);
              Sen->set_wn_q_filt(f_value);
              Serial.print(" to "); Serial.println(Sen->get_wn_q_filt(), 3);
              break;
           default:
              Serial.print(letter_0); Serial.println(" unknown");
              break;
          }
          break;          
        case ( 'v' ):
          switch ( letter_1 )
          {
            case ( 'v' ):
              debug = i_value;
            break;
            default:
              Serial.print(letter_1); Serial.println(" unknown");
              break;
          }
          break;
        case ( 'x' ):  // x - manually test toggle of Mahony filter
          *run = !*run;
          break;
        default:
          Serial.print(letter_0); Serial.println(" unknown");
          break;
      }
      cp.inp_str = "";
    }
}
