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
      time_acc_last_(0ULL), time_rot_last_(0ULL),
      o_is_quiet_(true), o_is_quiet_sure_(true), g_is_quiet_(true), g_is_quiet_sure_(true),
      roll_raw(0), pitch_raw(0), yaw_raw(0)
    {};
    Sensors(const unsigned long long time_now, const double NOM_DT ): t_ms(0),
      a_raw(0), b_raw(0), c_raw(0), o_raw(0), a_filt(0), b_filt(0), c_filt(0), o_filt(0),
      x_raw(0), y_raw(0), z_raw(0), g_raw(1), x_filt(0), y_filt(0), z_filt(0), g_filt(0),
      time_acc_last_(time_now), time_rot_last_(time_now),
      o_is_quiet_(true), o_is_quiet_sure_(true), g_is_quiet_(true), g_is_quiet_sure_(true),
      roll_raw(0), pitch_raw(0), yaw_raw(0)
    {
        // Update time and time constant changed on the fly
        float Tfilt_init = READ_DELAY/1000.;
        
        A_Filt = new LagExp(Tfilt_init, TAU_FILT, -W_MAX, W_MAX);
        B_Filt = new LagExp(Tfilt_init, TAU_FILT, -W_MAX, W_MAX);
        C_Filt = new LagExp(Tfilt_init, TAU_FILT, -W_MAX, W_MAX);
        O_Filt = new LagExp(Tfilt_init, TAU_FILT, -W_MAX, W_MAX);
        OQuietFilt = new General2_Pole(Tfilt_init, WN_Q_FILT, ZETA_Q_FILT, MIN_Q_FILT, MAX_Q_FILT);  // actual update time provided run time
        OQuietRate = new RateLagExp(Tfilt_init, TAU_Q_FILT, MIN_Q_FILT, MAX_Q_FILT);
        OQuietPer = new TFDelay(true, QUIET_S, QUIET_R, Tfilt_init);
        
        X_Filt = new LagExp(Tfilt_init, TAU_FILT, -G_MAX, G_MAX);
        Y_Filt = new LagExp(Tfilt_init, TAU_FILT, -G_MAX, G_MAX);
        Z_Filt = new LagExp(Tfilt_init, TAU_FILT, -G_MAX, G_MAX);
        G_Filt = new LagExp(Tfilt_init, TAU_FILT, -G_MAX, G_MAX);
        GQuietFilt = new General2_Pole(Tfilt_init, WN_Q_FILT, ZETA_Q_FILT, MIN_Q_FILT, MAX_Q_FILT);  // actual update time provided run time
        GQuietRate = new RateLagExp(Tfilt_init, TAU_Q_FILT, MIN_Q_FILT, MAX_Q_FILT);
        GQuietPer = new TFDelay(true, QUIET_S, QUIET_R, Tfilt_init);
        t_filter = new Mahony();
    };
    unsigned long long millis;
    ~Sensors(){};

    boolean both_are_quiet() { return o_is_quiet_sure_ && g_is_quiet_sure_; };
    boolean both_not_quiet() { return ( !o_is_quiet_sure_ && !g_is_quiet_sure_ ); };
    void filter(const boolean reset);
    boolean g_is_quiet_sure() { return g_is_quiet_sure_; };
    boolean o_is_quiet_sure() { return o_is_quiet_sure_; };
    void plot_all();
    void plot_all_acc();
    void plot_all_rot();
    void plot_all_rpy();
    void plot_all_sum();
    void plot_quiet();
    void plot_quiet_raw();
    void plot_total();
    void print_all_header();
    void print_all();
    void quiet_decisions(const boolean reset);
    void sample(const boolean reset, const unsigned long long time_now_ms, const unsigned long long time_start_ms, time_t now_hms);
    float T_acc() { return T_acc_; };
    float T_rot() { return T_rot_; };
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
    float roll_raw;
    float pitch_raw;
    float yaw_raw;
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
    Mahony *t_filter;   // Mahony tracking filter
    unsigned long long time_acc_last_;
    unsigned long long time_rot_last_;
    double T_acc_;
    double T_rot_;
    boolean acc_available_;
    boolean rot_available_;
    boolean o_is_quiet_;
    boolean o_is_quiet_sure_;
    boolean g_is_quiet_;
    boolean g_is_quiet_sure_;
};
