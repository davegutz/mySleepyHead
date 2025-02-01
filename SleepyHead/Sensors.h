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

#include "constants.h"
#ifdef USE_ARDUINO
  #include <Arduino.h> //needed for Serial.println
  #include <Arduino_LSM6DS3.h>
#else
  #error "Only Arduino nano 33 iot has built in IMU"
  #include "application.h"  // Particle
#endif

#include "src/MahonyAHRS/MahonyAHRS.h"
#include "src/Filters/myFilters.h"

extern int debug;


// Sensors (like a big struct with public access)
class Sensors
{
public:
    Sensors():
      a_raw(0), b_raw(0), c_raw(0), o_raw(0), a_filt(0), b_filt(0), c_filt(0), o_filt(0),
      x_raw(0), y_raw(0), z_raw(0), g_raw(0), x_filt(0), y_filt(0), z_filt(0), g_filt(0),
      time_acc_last_(0ULL), time_eye_last_(0LL), time_rot_last_(0ULL),
      o_is_quiet_(true), o_is_quiet_sure_(true), g_is_quiet_(true), g_is_quiet_sure_(true),
      roll_deg(0), pitch_deg(0), yaw_deg(0),
      eye_closed_(false), eye_closed_confirmed_(false), sensorPin_(0), eye_buzz_(false),
      head_buzz_f_(false), head_buzz_p_(false),
      pitch_thr_f_(0), roll_thr_f_(0), eye_voltage_norm_(0), v3v3_(0),
      eye_reset_(true), eye_set_time_(0), eye_reset_time_(0),
      head_reset_(true), head_set_time_(0), head_reset_time_(0), max_nod_f_confirmed_(false), max_nod_p_confirmed_(false),
      reset_(false), wn_q_filt_(0)
    {};
    Sensors(const unsigned long long time_now, const float two_kp, const float two_ki,
      const int sensorPin, const String unit):
      a_raw(0), b_raw(0), c_raw(0), o_raw(0), a_filt(0), b_filt(0), c_filt(0), o_filt(0),
      x_raw(0), y_raw(0), z_raw(0), g_raw(1), x_filt(0), y_filt(0), z_filt(0), g_filt(0),
      time_acc_last_(time_now), time_eye_last_(time_now), time_rot_last_(time_now),
      o_is_quiet_(true), o_is_quiet_sure_(true), g_is_quiet_(true), g_is_quiet_sure_(true),
      roll_deg(0), pitch_deg(0), yaw_deg(0),
      eye_closed_(false), eye_closed_confirmed_(false), sensorPin_(sensorPin), eye_buzz_(false),
      head_buzz_f_(false), head_buzz_p_(false),
      pitch_thr_f_(pitch_thr_def_forte), roll_thr_f_(roll_thr_def_forte),
      pitch_thr_p_(pitch_thr_def_piano), roll_thr_p_(roll_thr_def_piano), eye_voltage_norm_(0),
      unit_(unit), v3v3_(v3v3_nom), delta_pitch_(delta_pitch_def), delta_roll_(delta_roll_def),
      eye_reset_(true), eye_set_time_(EYE_S), eye_reset_time_(EYE_R),
      head_reset_(true), head_set_time_(HEAD_S), head_reset_time_(HEAD_R), max_nod_f_confirmed_(false), max_nod_p_confirmed_(false),
      reset_(true), wn_q_filt_(WN_Q_FILT)
    {
        // Update time and time constant changed on the fly
        float Tfilt_head_init = HEAD_DELAY/1000.;
        float Tfilt_eye_init = EYE_DELAY/1000.;
        
        A_Filt = new LagExp(Tfilt_head_init, TAU_FILT, -D_MAX, D_MAX);
        B_Filt = new LagExp(Tfilt_head_init, TAU_FILT, -D_MAX, D_MAX);
        C_Filt = new LagExp(Tfilt_head_init, TAU_FILT, -D_MAX, D_MAX);
        O_Filt = new LagExp(Tfilt_head_init, TAU_FILT, -D_MAX, D_MAX);
        OQuietFilt = new General2_Pole(Tfilt_head_init, WN_Q_FILT, ZETA_Q_FILT, -D_MAX, D_MAX);  // actual update time provided run time
        OQuietRate = new RateLagExp(Tfilt_head_init, TAU_Q_FILT, -D_MAX, D_MAX);
        OQuietPer = new TFDelay(true, QUIET_S, QUIET_R, Tfilt_head_init);
        
        X_Filt = new LagExp(Tfilt_head_init, TAU_FILT, -G_MAX, G_MAX);
        Y_Filt = new LagExp(Tfilt_head_init, TAU_FILT, -G_MAX, G_MAX);
        Z_Filt = new LagExp(Tfilt_head_init, TAU_FILT, -G_MAX, G_MAX);
        G_Filt = new LagExp(Tfilt_head_init, TAU_FILT, -G_MAX, G_MAX);
        GQuietFilt = new General2_Pole(Tfilt_head_init, WN_Q_FILT, ZETA_Q_FILT, -G_MAX, G_MAX);  // actual update time provided run time
        GQuietRate = new RateLagExp(Tfilt_head_init, TAU_Q_FILT, -G_MAX, G_MAX);
        GQuietPer = new TFDelay(true, QUIET_S, QUIET_R, Tfilt_head_init);
        TrackFilter = new Mahony(two_kp, two_ki);
        LTST_Filter = new LongTermShortTerm_ExpFilter(Tfilt_eye_init, TAU_LT, TAU_ST, -1.e6, -1.e5, FLT_THR_POS, FRZ_THR_POS, -v3v3_nom, v3v3_nom);
        HeadNodPerF = new TFDelay(true, EYE_S, EYE_R, Tfilt_head_init);
        HeadNodPerP = new TFDelay(true, EYE_S, EYE_R, Tfilt_head_init);
        EyeClosedPer = new TFDelay(false, EYE_S, EYE_R, Tfilt_eye_init); 
        GlassesOffPer = new TFDelay(true, OFF_S, OFF_R, Tfilt_eye_init); 
        HeadShakePer = new TFDelay(false, SHAKE_S, SHAKE_R, Tfilt_eye_init);
        EyeRateFilt = new RateLagExp(Tfilt_head_init, TAU_E_FILT, -v3v3_nom, v3v3_nom);
        RollRateFilt = new RateLagExp(Tfilt_head_init, TAU_HEAD_RATE_FILT, -D_MAX, D_MAX);
        PitchRateFilt = new RateLagExp(Tfilt_head_init, TAU_HEAD_RATE_FILT, -D_MAX, D_MAX);
        YawRateFilt = new RateLagExp(Tfilt_head_init, TAU_HEAD_RATE_FILT, -D_MAX, D_MAX);

    };

    unsigned long long millis;
    ~Sensors(){};

    boolean both_are_quiet() { return o_is_quiet_sure_ && g_is_quiet_sure_; };
    boolean both_not_quiet() { return ( !o_is_quiet_sure_ && !g_is_quiet_sure_ ); };
    void filter_eye(const boolean reset);
    void filter_head(const boolean reset, const boolean run);
    boolean g_is_quiet_sure() { return g_is_quiet_sure_; };
    float get_delta_pitch() { return delta_pitch_; };
    float get_delta_roll() { return delta_roll_; };
    float get_eye_reset_time() { return eye_reset_time_; };
    float get_eye_set_time() { return eye_set_time_; };
    float get_wn_q_filt() { return wn_q_filt_; };

    boolean o_is_quiet_sure() { return o_is_quiet_sure_; };
    boolean eye_closed_sure() { return eye_closed_confirmed_; };
    float eye_rate() { return eye_rate_; };
    boolean head_buzz_f() { return head_buzz_f_; };
    boolean head_buzz_p() { return head_buzz_p_; };
    float max_nod_forte_confirmed() { return max_nod_f_confirmed_; };
    float max_nod_piano_confirmed() { return max_nod_p_confirmed_; };
    float pitch_thr() { return pitch_thr_f_; };
    void pitch_thr(const float pitch_thr) { pitch_thr_f_ = pitch_thr; };
    void roll_thr(const float roll_thr) { roll_thr_f_ = roll_thr; };
    float roll_thr() { return roll_thr_f_; };
    void plot_all();  // pp3
    void plot_all_acc();  // pp1
    void plot_all_rot();  // pp2
    void plot_all_rpy();  // pp7
    void plot_all_sum();  // pp0
    void plot_eye_buzz();  // pp9
    void plot_head_buzz();  // pp8
    void plot_quiet();  // pp4
    void plot_quiet_raw();  // pp5
    void plot_total();  // pp6
    void pretty_print_head();
    void print_all_header();
    void print_all();
    void print_default_hdr(const uint8_t plot_num);
    void print_Mahony(const boolean print_hdr, const float time_s);  // pp11
    void print_rapid(const boolean print_hdr, const float time_s);  // pp10
    void print_rapid_10(const float time_s);  // pp10
    void print_rapid_10_hdr();  // pp10
    void print_rapid_11(const float time_s);  // pp11
    void print_rapid_11_hdr();  // pp11
    void quiet_decisions(const boolean reset, const float o_quiet_thr, const float g_quiet_thr);
    void sample_eye(const boolean reset, const unsigned long long time_eye_ms);
    void sample_head(const boolean reset, const unsigned long long time_now_ms, const unsigned long long time_start_ms, time_t now_hms);
    void set_delta_pitch(const float input) { delta_pitch_ = input; };
    void set_delta_roll(const float input) { delta_roll_ = input; };
    void set_eye_reset_time(const float input) { eye_reset_time_ = input; };
    void set_eye_set_time(const float input) { eye_set_time_ = input; };
    void set_wn_q_filt(const float input) { wn_q_filt_ = input; };
    float T_acc() { return T_acc_; };
    float T_rot() { return T_rot_; };
    float time_eye_s() { return float(time_eye_ms_)/1000.0; };
    float time_now_s() { return float(time_head_ms_)/1000.0; };

    // Gyroscope in deg/second
    float a_raw;
    float b_raw;
    float c_raw;
    float o_raw;  // Total gyroscope
    // Acceleration in g's
    float x_raw;
    float y_raw;
    float z_raw;
    float g_raw;  // Total acceleration
    // Filtered in deg/second
    float a_filt;
    float b_filt;
    float c_filt;
    float o_filt;
    // Filtered for quiet detection
    float o_qrate;
    float o_quiet;
    // Filtered in g's
    float x_filt;
    float y_filt;
    float z_filt;
    float g_filt;
    // Filtered for quiet detection
    float g_qrate;  // Gyro quiet rate for logic, g/s
    float g_quiet;  // Gyro quiet for logic, g/s
    // Euler 321 angles, deg
    float roll_deg;  // Roll about x-axis, deg
    float pitch_deg;  // Pitch about y-axis, deg
    float yaw_deg;  // Yaw about z-axis, deg

    Mahony *TrackFilter;   // Mahony tracking filter
    LongTermShortTerm_Filter *LTST_Filter;  // LTST filter
protected:
    LagExp *A_Filt;     // Noise filter
    LagExp *B_Filt;     // Noise filter
    LagExp *C_Filt;     // Noise filter
    LagExp *O_Filt;     // Noise filter
    General2_Pole *OQuietFilt; // Quiet detector
    RateLagExp *OQuietRate;    // Quiet detector
    TFDelay *OQuietPer; // Persistence ib quiet disconnect detection
    LagExp *X_Filt;     // Noise filter
    LagExp *Y_Filt;     // Noise filter
    LagExp *Z_Filt;     // Noise filter
    LagExp *G_Filt;     // Noise filter
    General2_Pole *GQuietFilt;  // Quiet detector
    RateLagExp *GQuietRate;     // Quiet detector
    RateLagExp *EyeRateFilt;    // Rate detector
    RateLagExp *RollRateFilt;   // Rate detector
    RateLagExp *PitchRateFilt;  // Rate detector
    RateLagExp *YawRateFilt;    // Rate detector
    TFDelay *GQuietPer; // Persistence ib quiet disconnect detection
    TFDelay *EyeClosedPer; // Persistence eye closed detection
    TFDelay *HeadNodPerF;  // Persistence forte head nod detection
    TFDelay *HeadNodPerP;  // Persistence piano head nod detection
    TFDelay *GlassesOffPer; // Persistence eye glasses off detection, for reset of LTST filter
    TFDelay *HeadShakePer;  // Persistence head shake detection
    unsigned long long time_acc_last_;
    unsigned long long time_eye_last_;
    unsigned long long time_rot_last_;
    double T_acc_;  // Detected update of accelerometer time, s
    double T_eye_;  // Detected update of eye voltage time, s
    double T_rot_;  // Detected update of gyroscope time, s
    boolean acc_available_;  // IMU available
    boolean rot_available_;  // IMU available
    boolean o_is_quiet_;  // Quiet detected on gyroscope
    boolean o_is_quiet_sure_;  // Quiet detected on gyroscope and persistent
    boolean g_is_quiet_;  // Quiet detected on acceleration
    boolean g_is_quiet_sure_; // Quiet detected on acceleration and persistent
    float max_nod_f_;  // in deg for forte signal
    float max_nod_p_;  // in deg for piano signal
    boolean eye_closed_;  // Eye closed detected
    boolean eye_closed_confirmed_;  // Eye closed detected and persistent
    int sensorPin_;  // Pin connected to the IR sensor
    boolean eye_buzz_;  // Declaring a buzz to wake driver
    boolean head_buzz_f_;  // Declaring a buzz to wake driver
    boolean head_buzz_p_;  // Declaring a buzz to wake driver
    // in deg for forte signal
    float pitch_thr_f_;
    float roll_thr_f_;
    // in deg for piano signal
    float pitch_thr_p_;
    float roll_thr_p_;
    float eye_voltage_norm_;  // Sensed eye voltage normalized to 3.3V
    unsigned long long time_eye_ms_;
    unsigned long long time_head_ms_;
    String unit_;
    float v3v3_;  // VCC voltage in 3.3V units
    float delta_pitch_;  //bias, deg
    float delta_roll_;   //bias, deg
    boolean eye_reset_;  // Eye reset signal
    float eye_set_time_;  // Eye persistent reset time, s
    float eye_reset_time_;  // Eye persistent reset time, s
    boolean head_reset_;  // Head reset signal
    float head_set_time_;  // Head persistent set time, s
    float head_reset_time_;  // Head persistent reset time, s
    boolean max_nod_f_confirmed_;  // Forte nod exceedance persistent
    boolean max_nod_p_confirmed_;  // Piano nod exceedance persistent
    boolean reset_;  // Logic initialization indicator
    float wn_q_filt_;  // Quiet filter-2 natural frequency, r/s
    float eye_rate_;  // Eye rate, v/s
    float roll_rate_;  // Roll rate based on raw data, deg/s
    float pitch_rate_;  // Pitch rate based on raw data, deg/s
    float yaw_rate_;  // Yaw rate based on raw data, deg/s
};
