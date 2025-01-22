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

#pragma once

#ifdef USE_ARDUINO
  #include <Arduino.h> //needed for Serial.println
  #include <Arduino_LSM6DS3.h>
#else
  #error "Only Arduino nano 33 iot has built in IMU"
  #include "application.h"  // Particle
#endif

#include "MahonyAHRS.h"

#include "myFilters.h"
extern int debug;



// Sensors (like a big struct with public access)
class Sensors
{
public:
    Sensors(): t_ms(0),
      a_raw(0), b_raw(0), c_raw(0), o_raw(0), a_filt(0), b_filt(0), c_filt(0), o_filt(0),
      x_raw(0), y_raw(0), z_raw(0), g_raw(0), x_filt(0), y_filt(0), z_filt(0), g_filt(0),
      time_acc_last_(0ULL), time_eye_last_(0LL), time_rot_last_(0ULL),
      o_is_quiet_(true), o_is_quiet_sure_(true), g_is_quiet_(true), g_is_quiet_sure_(true),
      roll_filt(0), pitch_filt(0), yaw_filt(0),
      eye_closed_(false), eye_closed_confirmed_(false), sensorPin_(0), eye_buzz_(false), head_buzz_(false),
      pitch_thr_f_(0), roll_thr_f_(0), eye_voltage_norm_(0), v3v3_(0),
      v3v3Pin_(0),
      eye_reset_(true), event_set_time_(0), event_reset_time_(0),
      head_reset_(true), max_nod_f_confirmed_(false), max_nod_p_confirmed_(false)
    {};
    Sensors(const unsigned long long time_now, const double NOM_DT, const float t_kp, const float t_ki,
      const int sensorPin, const String unit, const int v3v3_pin): t_ms(0),
      a_raw(0), b_raw(0), c_raw(0), o_raw(0), a_filt(0), b_filt(0), c_filt(0), o_filt(0),
      x_raw(0), y_raw(0), z_raw(0), g_raw(1), x_filt(0), y_filt(0), z_filt(0), g_filt(0),
      time_acc_last_(time_now), time_eye_last_(time_now), time_rot_last_(time_now),
      o_is_quiet_(true), o_is_quiet_sure_(true), g_is_quiet_(true), g_is_quiet_sure_(true),
      roll_filt(0), pitch_filt(0), yaw_filt(0),
      eye_closed_(false), eye_closed_confirmed_(false), sensorPin_(sensorPin), eye_buzz_(false), head_buzz_(false),
      pitch_thr_f_(pitch_thr_def_forte), roll_thr_f_(roll_thr_def_forte),
      pitch_thr_p_(pitch_thr_def_piano), roll_thr_p_(roll_thr_def_piano), eye_voltage_norm_(0),
      unit_(unit), v3v3_(v3v3_nom), v3v3Pin_(v3v3_pin), delta_pitch_(delta_pitch_def), delta_roll_(delta_roll_def),
      eye_reset_(true), event_set_time_(EVENT_S), event_reset_time_(EVENT_R),
      head_reset_(true), max_nod_f_confirmed_(false), max_nod_p_confirmed_(false)
    {
        // Update time and time constant changed on the fly
        float Tfilt_head_init = HEAD_DELAY/1000.;
        float Tfilt_eye_init = EYE_DELAY/1000.;
        
        A_Filt = new LagExp(Tfilt_head_init, TAU_FILT, -W_MAX, W_MAX);
        B_Filt = new LagExp(Tfilt_head_init, TAU_FILT, -W_MAX, W_MAX);
        C_Filt = new LagExp(Tfilt_head_init, TAU_FILT, -W_MAX, W_MAX);
        O_Filt = new LagExp(Tfilt_head_init, TAU_FILT, -W_MAX, W_MAX);
        OQuietFilt = new General2_Pole(Tfilt_head_init, WN_Q_FILT, ZETA_Q_FILT, MIN_Q_FILT, MAX_Q_FILT);  // actual update time provided run time
        OQuietRate = new RateLagExp(Tfilt_head_init, TAU_Q_FILT, MIN_Q_FILT, MAX_Q_FILT);
        OQuietPer = new TFDelay(true, QUIET_S, QUIET_R, Tfilt_head_init);
        
        X_Filt = new LagExp(Tfilt_head_init, TAU_FILT, -G_MAX, G_MAX);
        Y_Filt = new LagExp(Tfilt_head_init, TAU_FILT, -G_MAX, G_MAX);
        Z_Filt = new LagExp(Tfilt_head_init, TAU_FILT, -G_MAX, G_MAX);
        G_Filt = new LagExp(Tfilt_head_init, TAU_FILT, -G_MAX, G_MAX);
        GQuietFilt = new General2_Pole(Tfilt_head_init, WN_Q_FILT, ZETA_Q_FILT, MIN_Q_FILT, MAX_Q_FILT);  // actual update time provided run time
        GQuietRate = new RateLagExp(Tfilt_head_init, TAU_Q_FILT, MIN_Q_FILT, MAX_Q_FILT);
        GQuietPer = new TFDelay(true, QUIET_S, QUIET_R, Tfilt_head_init);
        TrackFilter = new Mahony(t_kp, t_ki);
        LTST_Filter = new LongTermShortTerm_Filter(Tfilt_eye_init, TAU_LT, TAU_ST, -1.e6, -1.e5, FLT_THR_POS, FRZ_THR_POS);
        HeadNodPerF = new TFDelay(true, EVENT_S, EVENT_R, Tfilt_head_init);
        HeadNodPerP = new TFDelay(true, EVENT_S, EVENT_R, Tfilt_head_init);
        EyeClosedPer = new TFDelay(false, EVENT_S, EVENT_R, Tfilt_eye_init); 
        GlassesOffPer = new TFDelay(true, OFF_S, OFF_R, Tfilt_eye_init); 
        HeadShakePer = new TFDelay(false, SHAKE_S, SHAKE_R, Tfilt_eye_init); 
    };

    unsigned long long millis;
    ~Sensors(){};

    boolean both_are_quiet() { return o_is_quiet_sure_ && g_is_quiet_sure_; };
    boolean both_not_quiet() { return ( !o_is_quiet_sure_ && !g_is_quiet_sure_ ); };
    void filter_eye(const boolean reset);
    void filter_head(const boolean reset);
    boolean g_is_quiet_sure() { return g_is_quiet_sure_; };
    float get_delta_pitch() { return delta_pitch_; };
    float get_delta_roll() { return delta_roll_; };
    float get_event_reset_time() { return event_reset_time_; };
    float get_event_set_time() { return event_set_time_; };
    void header_rapid_10();
    boolean o_is_quiet_sure() { return o_is_quiet_sure_; };
    boolean eye_closed_sure() { return eye_closed_confirmed_; };
    float max_nod_forte() { return max_nod_f_; };
    float max_nod_piano() { return max_nod_p_; };
    float max_nod_forte_confirmed() { return max_nod_f_confirmed_; };
    float max_nod_piano_confirmed() { return max_nod_p_confirmed_; };
    float pitch_thr() { return pitch_thr_f_; };
    void pitch_thr(const float pitch_thr) { pitch_thr_f_ = pitch_thr; };
    void roll_thr(const float roll_thr) { roll_thr_f_ = roll_thr; };
    float roll_thr() { return roll_thr_f_; };
    void plot_all();
    void plot_all_acc();
    void plot_all_rot();
    void plot_all_rpy();
    void plot_all_sum();
    void plot_eye_buzz();
    void plot_head_buzz();
    void plot_quiet();
    void plot_quiet_raw();
    void plot_total();
    void pretty_print_head();
    void print_all_header();
    void print_all();
    void print_rapid(const boolean reset, const boolean print_now, const float time_s);
    void print_rapid_10(const float time_s);
    void quiet_decisions(const boolean reset, const float o_quiet_thr, const float g_quiet_thr);
    void sample_eye(const boolean reset, const unsigned long long time_eye_ms);
    void sample_head(const boolean reset, const unsigned long long time_now_ms, const unsigned long long time_start_ms, time_t now_hms);
    void set_delta_pitch(const float input) { delta_pitch_ = input; };
    void set_delta_roll(const float input) { delta_roll_ = input; };
    void set_event_reset_time(const float input) { event_reset_time_ = input; };
    void set_event_set_time(const float input) { event_set_time_ = input; };
    float T_acc() { return T_acc_; };
    float T_rot() { return T_rot_; };
    float time_eye_s() { return float(time_eye_ms_)/1000.0; };
    float time_now_s() { return float(time_head_ms_)/1000.0; };

    time_t t_ms;
    // Gyroscope in radians/second
    float a_raw;
    float b_raw;
    float c_raw;
    float o_raw;  // Total gyroscope
    // Acceleration in g's
    float x_raw;
    float y_raw;
    float z_raw;
    float g_raw;  // Total acceleration
    float a_filt;
    float b_filt;
    float c_filt;
    float o_filt;
    float o_qrate;
    float o_quiet;
    float x_filt;
    float y_filt;
    float z_filt;
    float g_filt;
    float g_qrate;
    float g_quiet;
    float roll_filt;
    float pitch_filt;
    float yaw_filt;
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
    General2_Pole *GQuietFilt; // Quiet detector
    RateLagExp *GQuietRate;    // Quiet detector
    TFDelay *GQuietPer; // Persistence ib quiet disconnect detection
    TFDelay *EyeClosedPer; // Persistence eye closed detection
    TFDelay *HeadNodPerF;  // Persistence forte head nod detection
    TFDelay *HeadNodPerP;  // Persistence piano head nod detection
    TFDelay *GlassesOffPer; // Persistence eye glasses off detection, for reset of LTST filter
    TFDelay *HeadShakePer;  // Persistence head shake detection
    unsigned long long time_acc_last_;
    unsigned long long time_eye_last_;
    unsigned long long time_rot_last_;
    double T_acc_;
    double T_eye_;
    double T_rot_;
    boolean acc_available_;
    boolean rot_available_;
    boolean o_is_quiet_;
    boolean o_is_quiet_sure_;
    boolean g_is_quiet_;
    boolean g_is_quiet_sure_;
    float max_nod_f_;
    float max_nod_p_;
    boolean eye_closed_;
    boolean eye_closed_confirmed_;
    int sensorPin_;
    boolean eye_buzz_;
    boolean head_buzz_;
    float pitch_thr_f_;
    float roll_thr_f_;
    float pitch_thr_p_;
    float roll_thr_p_;
    float eye_voltage_norm_;
    unsigned long long time_eye_ms_;
    unsigned long long time_head_ms_;
    String unit_;
    float v3v3_;
    int v3v3Pin_;
    float delta_pitch_;
    float delta_roll_;
    boolean eye_reset_;
    float event_set_time_;
    float event_reset_time_;
    boolean head_reset_;
    boolean max_nod_f_confirmed_;
    boolean max_nod_p_confirmed_;
};
