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

/***************************************************
  A simple dynamic filter library

  Class code for embedded application.

  07-Jan-2015   Dave Gutz   Created
  30-Sep-2016   Dave Gutz   LeadLagTustin
  23-Nov-2016   Dave Gutz   LeadLagExp
  09-Feb-2021   Dave Gutz   RateLagExp, LagExp, General2_Pole
 ****************************************************/

#pragma once

#define USE_ARDUINO

#ifdef USE_ARDUINO
  #include <Arduino.h> //needed for Serial.println
#else
  #include "application.h"
#endif

#include <math.h>

#define DEAD(X, HDB)  ( max(X-HDB, 0) + min(X+HDB, 0) )


/* Pseudo-Random Binary Sequence, 7 bits
   Useful noise device
   Pseudo-Random Binary Sequence, 7 bits.  Seed in range [0-255] or [0x00-0xFF]
*/
class PRBS_7
{
public:
  PRBS_7();
  PRBS_7(const uint8_t seed);
  ~PRBS_7();
  // operators
  // functions
  uint8_t calculate();
protected:
  uint8_t noise_;   // Static value of sequence, [0-255] or [0x00-0xFF]
};


class Debounce
{
public:
  Debounce();
  Debounce(const bool icValue, const int updates);
  ~Debounce();
  // operators
  // functions
  bool calculate(const bool in);
  bool calculate(const bool in, const int RESET);
protected:
  int nz_;     // Number of past consequetive states to agree with input to pass debounce
  bool passed_out_; // latched value of output
  bool *past_; // Array(nz_-1) of past inputs
};


class DetectRise
{
public:
  DetectRise();
  ~DetectRise();
  // operators
  // functions
  bool calculate(const bool in);
  bool calculate(const int in);
  bool calculate(const double in);
protected:
  double past_;
};


class SRLatch
{
public:
  SRLatch();
  SRLatch(const bool icValue);
  ~SRLatch();
  // operators
  // functions
  bool calculate(const bool S, const bool R);
protected:
  bool state_;
};


class Delay
{
public:
  Delay();
  Delay(const double in, const int nz);
  ~Delay();
  // operators
  // functions
  double calculate(const double in);
  double calculate(const double in, const int RESET);
protected:
  double *past_;
  int nz_;
};


class TFDelay
{
public:
  TFDelay();
  TFDelay(const bool in, const double Tt, const double Tf, const double T);
  ~TFDelay();
  // operators
  // functions
  boolean calculate(const boolean in);
  boolean calculate(const boolean in, const int RESET);
  boolean calculate(const boolean in, const double Tt, const double Tf);
  boolean calculate(const boolean in, const double Tt, const double Tf, const double T);
  boolean calculate(const boolean in, const double Tt, const double Tf, const int RESET);
  boolean calculate(const boolean in, const double Tt, const double Tf, const double T, const int RESET);
  void repr();
  boolean state() { return ( timer_> 0 ); };
  int timer() { return timer_; };
  int nt() { return nt_; };
  int nf() { return nf_; };
  int T() { return T_; };
  int T_init() { return T_init_; };
protected:
  int timer_;
  int nt_;
  int nf_;
  double T_;
  double T_init_;
  boolean reset_;
  boolean input_;
  double Tt_;
  double Tf_;
  boolean result_;
};


class RateLimit
{
public:
  RateLimit();
  RateLimit(const double in, const double T);
  RateLimit(const double in, const double T, const double Rmax, const double Rmin);
  ~RateLimit();
  // operators
  // functions
  double calculate(const double in);
  double calculate(const double in, const int RESET);
  double calculate(const double in, const double Rmax, const double Rmin);
  double calculate(const double in, const double Rmax, const double Rmin, const int RESET);
  double calculate(const double in, const double Rmax, const double Rmin, const int RESET, const double T);
protected:
  double past_;
  double jmax_;   // Max rate limit, units of in/update
  double jmin_;   // Min rate limit, units of in/update (<0)
  double T_;      // Update rate, sec
};


class SlidingDeadband
{
public:
  SlidingDeadband();
  SlidingDeadband(const double hdb);
  ~SlidingDeadband();
  // operators
  // functions
  double update(const double in);
  double update(const double in, const int RESET);
protected:
  double z_;      // State of output, units of input
  double hdb_;    // Half of deadband width, units of input
};


// ************************** 1-Pole Filters ***********************************************
class DiscreteFilter
{
public:
  DiscreteFilter();
  DiscreteFilter(const double T, const double tau, const double min, const double max);
  virtual ~DiscreteFilter();
  // operators
  // functions
  virtual double calculate(double in, int RESET);
  virtual void assignCoeff(double tau);
  virtual void rateState(double in);
  virtual double rateStateCalc(double in);
  virtual double state(void);
protected:
  double max_;
  double min_;
  double rate_;
  double T_;
  double tau_;
};


// Tustin rate-lag rate calculator, non-pre-warped, no limits
class LeadLagTustin : public DiscreteFilter
{
public:
  LeadLagTustin();
  LeadLagTustin(const double T, const double tld, const double tau, const double min, const double max);
  //  LeadLagTustin(const LeadLagTustin & RLT);
  ~LeadLagTustin();
  //operators
  //functions
  virtual double calculate(const double in, const int RESET);
  virtual double calculate(const double in, const int RESET, const double T);
  virtual double calculate(double in, int RESET, const double T, const double tau, const double tld);
  virtual void assignCoeff(const double tld, const double tau, const double T);
  virtual double rateStateCalc(const double in);
  virtual double rateStateCalc(const double in, const double T);
  virtual double state(void);
protected:
  double a_;
  double b_;
  double state_;
  double tld_;
};


// Tustin rate-lag rate calculator, non-pre-warped, no limits, fixed update rate
class LeadLagExp : public DiscreteFilter
{
public:
  LeadLagExp();
  LeadLagExp(const double T, const double tld, const double tau, const double min, const double max);
  //  LeadLagExp(const LeadLagExp & RLT);
  ~LeadLagExp();
  //operators
  //functions
  virtual double calculate(const double in, const int RESET);
  virtual double calculate(const double in, const int RESET, const double T);
  virtual double calculate(double in, int RESET, const double T, const double tau, const double tld);
  virtual void assignCoeff(const double tld, const double tau, const double T);
  virtual double rateStateCalc(const double in);
  virtual double rateStateCalc(const double in, const double T);
  virtual double state(void);
protected:
  double a_;
  double b_;
  double state_;
  double instate_;
  double tld_;
};


// Tustin rate-lag rate calculator, non-pre-warped, no limits, fixed update rate
class RateLagTustin : public DiscreteFilter
{
public:
  RateLagTustin();
  RateLagTustin(const double T, const double tau, const double min, const double max);
  //  RateLagTustin(const RateLagTustin & RLT);
  ~RateLagTustin();
  //operators
  //functions
  virtual double calculate(double in, int RESET);
  virtual void assignCoeff(double tau);
  virtual void rateState(double in);
  virtual double state(void);
protected:
  double a_;
  double b_;
  double state_;
};


// Exponential rate-lag rate calculator
class RateLagExp : public DiscreteFilter
{
public:
  RateLagExp();
  RateLagExp(const double T, const double tau, const double min, const double max);
  ~RateLagExp();
  //operators
  //functions
  virtual double calculate(double in, int RESET);
  virtual double calculate(double in, int RESET, const double T);
  virtual void assignCoeff(double tau);
  virtual void rateState(double in);
  virtual void rateState(double in, const double T);
  void repr();
  virtual double state(void);
  double a() { return (a_); };
  double b() { return (b_); };
  double c() { return (c_); };
  double lstate() { return (lstate_); };
  double rstate() { return (rstate_); };
protected:
  double a_;
  double b_;
  double c_;
  double lstate_; // lag state
  double rstate_; // rate state
};


// Tustin lag calculator
class LagTustin : public DiscreteFilter
{
public:
  LagTustin();
  LagTustin(const double T, const double tau, const double min, const double max);
  //  LagTustin(const LagTustin & RLT);
  ~LagTustin();
  //operators
  //functions
  virtual double calculate(double in, int RESET);
  virtual double calculate(double in, int RESET, const double T);
  virtual void assignCoeff(double tau);
  virtual void calcState(double in);
  virtual void calcState(double in, const double T);
  virtual double state(void);
  virtual void state(const double in) { state_ = in; }  // For severity testing - sudden offset
  double a() { return (a_); };
  double b() { return (b_); };
  double rate() { return (rate_); };
protected:
  double a_;
  double b_;
  double rate_;
  double state_;
};


// Exponential lag calculator
class LagExp : public DiscreteFilter
{
public:
  LagExp();
  LagExp(const double T, const double tau, const double min, const double max);
  ~LagExp();
  //operators
  //functions
  void absorb(LagExp *LE) { lstate_ = LE->lstate_; rstate_ = LE->rstate_; };
  virtual double calculate(double in, int RESET);
  virtual double calculate(double in, int RESET, const double tau, const double T);
  virtual void assignCoeff(double tau, double T);
  virtual void rateState(double in);
  double a() { return (a_); };
  double b() { return (b_); };
  double c() { return (c_); };
  double rate() { return (rate_); };
  double lstate() { return (lstate_); };
  void lstate(const double in) { lstate_ = in; };
  double rstate() { return (rstate_); };
  void rstate(const double in) { rstate_ = in; };
protected:
  double a_;
  double b_;
  double c_;
  // double rate_;
  double lstate_;
  double rstate_;
};


// *********************** Integrators *******************************************
class DiscreteIntegrator
{
public:
  DiscreteIntegrator();
  DiscreteIntegrator(const double T, const double min, const double max, const double a, const double b, const double c);
  virtual ~DiscreteIntegrator();
  // operators
  // functions
  virtual double calculate(double in, int RESET, double init_value);
  virtual double calculate(double in, double T, int RESET, double init_value);
  virtual void newState(double newState);
  virtual double state() { return lstate_; };
  virtual bool lim() { return lim_; };
protected:
  double a_;
  double b_;
  double c_;
  bool lim_;
  double max_;
  double min_;
  double lstate_;
  double rstate_;
  double T_;
};


// AB2_Integrator
class AB2_Integrator : public DiscreteIntegrator
{
public:
  AB2_Integrator();
  AB2_Integrator(const double T, const double min, const double max);
  ~AB2_Integrator();
  //operators
  //functions
protected:
};


// Tustin Integrator
class TustinIntegrator : public DiscreteIntegrator
{
public:
  TustinIntegrator();
  TustinIntegrator(const double T, const double min, const double max);
  ~TustinIntegrator();
  //operators
  //functions
protected:
};


// ************************************** 2-pole filters  *************************
class DiscreteFilter2
{
public:
  DiscreteFilter2();
  DiscreteFilter2(const double T, const double omega_n, const double zeta, const double min, const double max);
  virtual ~DiscreteFilter2();
  // operators
  // functions
  virtual double calculate(const double in, const int RESET);
  virtual void assignCoeff(const double T);
  virtual void rateState(const double in, const int RESET);
  virtual void rateStateCalc(const double in, const double T, const int RESET);
protected:
  double max_;
  double min_;
  double omega_n_;
  double T_;
  double zeta_;
};


// General 2-Pole for any value of z, aliases easily though
class General2_Pole : public DiscreteFilter2
{
public:
  General2_Pole();
  General2_Pole(const double T, const double omega_n, const double zeta, const double min, const double max);
  ~General2_Pole();
  //operators
  //functions
  virtual double calculate(const double in, const int RESET);
  virtual double calculate(const double in, const int RESET, const double T);
  virtual double calculate(double in, int RESET, const double wn, const double zeta, const double T);
  virtual void assignCoeff(const double T);
  virtual void assignCoeff(const double T, const double wn, const double zeta);
  virtual void rateState(const double in, const int RESET);
  virtual void rateStateCalc(const double in, const double T, const int RESET);
  virtual void rateStateCalc(double in, const double T, const double wn, const double zeta, const int RESET);
protected:
  AB2_Integrator *AB2_;
  double a_;
  double b_;
  TustinIntegrator *Tustin_;
};


class LongTermShortTerm_Filter
{
public:
  LongTermShortTerm_Filter()
    : flt_thr_neg_(0), flt_thr_pos_(0), freeze_(false), frz_thr_neg_(0), frz_thr_pos_(0), T_(0), tau_lt_(0), tau_st_(0)
  {};
  LongTermShortTerm_Filter(const double T, const double tau_lt, const double tau_st, const double flt_thr_neg, const double frz_thr_neg,
                           const double flt_thr_pos, const double frz_thr_pos);
  ~LongTermShortTerm_Filter(){};
  void assign_coeff(const double T);
  virtual boolean calculate(const double in, const boolean RESET, const double T, const double lt_bias_init);
  virtual boolean calculate(const double in, const boolean RESET, const double tau_lt, const double tau_st, const double T, const double lt_bias_init);
  double cf() { return (cf_); };
  double dltst() { return (dltst_); };
  boolean freeze() { return (freeze_); };
  double get_fault_thr_neg() { return (flt_thr_neg_); };
  double get_fault_thr_pos() { return (flt_thr_pos_); };
  double get_freeze_thr_neg() { return (frz_thr_neg_); };
  double get_freeze_thr_pos() { return (frz_thr_pos_); };
  double get_klt() { return (klt_); };
  double get_kst() { return (kst_); };
  double get_tau_lt() { return (tau_lt_); };
  double get_tau_st() { return (tau_st_); };
  double lt_state() { return (lt_state_); };
  void pretty_print();
  void set_fault_thr_neg(const double input) { flt_thr_neg_ = input; };
  void set_fault_thr_pos(const double input) { flt_thr_pos_ = input; };
  void set_freeze_thr_neg(const double input) { frz_thr_neg_ = input; };
  void set_freeze_thr_pos(const double input) { frz_thr_pos_ = input; };
  void set_tau_lt(const double input) { tau_lt_ = input; klt_ = T_ / tau_lt_; };
  void set_tau_st(const double input) { tau_st_ = input; kst_ = T_ / tau_st_; };
  double st_state() { return (st_state_); };
protected:
  double cf_;
  double dltst_;
  boolean fault_;
  double flt_thr_neg_;
  double flt_thr_pos_;
  boolean freeze_;
  double frz_thr_neg_;
  double frz_thr_pos_;
  double input_;
  double klt_;
  double kst_;
  double lt_state_;
  boolean RESET_;
  double st_state_;
  double T_;
  double tau_lt_;
  double tau_st_;
};


class LongTermShortTerm_ExpFilter : public LongTermShortTerm_Filter
{
public:
  LongTermShortTerm_ExpFilter();
  LongTermShortTerm_ExpFilter(const double T, const double tau_lt, const double tau_st,
    const double flt_thr_neg, const double frz_thr_neg, const double flt_thr_pos, const double frz_thr_pos,
    const double min, const double max);
  ~LongTermShortTerm_ExpFilter(){};
  virtual void assign_coeff(const double T);
  virtual boolean calculate(const double in, const boolean RESET, const double T);
  virtual boolean calculate(const double in, const boolean RESET, const double tau_lt, const double tau_st, const double T);
  virtual void set_tau_lt(const double input) { tau_lt_ = input; };
  virtual void set_tau_st(const double input) { tau_st_ = input; };
protected:
  LagExp *lt_;
  LagExp *st_;
};


// PID
struct PID
{
  double G;     // Gain, r/s = %/units of err
  double tau;   // Lead, s
  double MAX;   // Integrator max limit, %
  double MIN;   // Integrator min limit, %
  double LLMAX; // Lead max, %
  double LLMIN; // Lead min, %
  double prop;  // Proportional output, %
  double integ; // Integrator output, %
  double DB;    // Half deadband width, units of err
  double err;   // Error, units
  double err_comp; // Compensated error, units of err
  double cont;  // Total control output, %
  double sd_;   // Derivative tlead lookup scalar
  double sg_;   // Integral lookup scalar
  double st_;   // Proportional lookup scalar
  double ad_;   // Derivative tlead adder
  double ag_;   // Integral lookup adder
  double at_;   // Proportional lookup adder
  double kick_th_; // Threshold to kick
  double kick_; // Kick scalar
  double Sd(void) { return(sd_);};  // Der Tld scalar
  double Sg(void) { return(sg_);};  // Gain scalar
  double St(void) { return(st_);};  // Prop Tld scalar
  double Ad(void) { return(ad_);}; 
  double Ag(void) { return(ag_);};
  double At(void) { return(at_);};
  void GN(const double g)  { G = g; };
  void Sd(const double S) { sd_ = S; };
  void Ad(const double A) { ad_ = A; };
  void Sg(const double S) { sg_ = S; };
  void Ag(const double A) { ag_ = A; };
  void St(const double S) { st_ = S; };
  void At(const double A) { at_ = A; };
  PID(double G, double tau, double MAX, double MIN, double LLMAX, double LLMIN, double prop, double integ, double DB,
    double err, double err_comp, double cont, double kick_th, double kick)
  {
    this->G = G;
    this->tau = tau;
    this->MAX = MAX;
    this->MIN = MIN;
    this->LLMAX = LLMAX;
    this->LLMIN = LLMIN;
    this->prop = prop;
    this->integ = integ;
    this->DB = DB;
    this->err = err;
    this->err_comp = err_comp;
    this->cont = cont;
    this->sd_ = 1;
    this->ad_ = 0;
    this->sg_ = 1;
    this->ag_ = 0;
    this->st_ = 1;
    this->at_ = 0;
    this->kick_th_ = kick_th;
    this->kick_ = kick;
  }
  void update(bool reset, double ref, double fb, double updateTime, double init, double dyn_max, double dyn_min, bool kill_db)
  {
    err = ref - fb;
    double DB_loc = DB;
    if ( kill_db ) DB_loc = 0.;
    err_comp = DEAD(err, (DB_loc*sd_+ad_)) * (G*sg_+ag_);
    if ( fabs(err)>kick_th_ ) err_comp *= kick_;
    prop = max(min(err_comp * (tau*st_+at_), LLMAX), LLMIN);
    integ = max(min(integ + updateTime*err_comp, dyn_max-prop), dyn_min-prop);
    if ( reset ) integ = init;
    cont = max(min(integ + prop, dyn_max), dyn_min);
  }
};
