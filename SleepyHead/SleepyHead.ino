/*
  Uses LSM6DS3 in Arduino Nano 33 IoT
  This unit is reported to have BLE but I have not confirmed.
  This unit requires special EEPROM handling using FlashStorage library.  But not used SleepyHead

  The circuit:
  - Arduino Uno WiFi Rev 2 or Arduino Nano 33 IoT
  - USB for monitor and power

  The Arduino Libraries:
  - Arduino_LSM6DS3, AceCommon, AceCRC, AceRoutine, AceUtils, Adafruit BusIO, Adafruit LSMDS?, SafeString
  
  Arduino board for CTE Nano:
  - Arduino Nano 33 IoT in Afduino SAMD library
 
  created 1 Feb 2025
  by Dave Gutz

  This example code is in the public domain.

  Requirements:
  1.
  Notes:
  1.  IMU in Nano captures 6 dof IMU at 100 Hz.   Too slow but good for 20 Hz analysis

*/

// Defines and constants
#include "constants.h"

// Include libraries
#ifdef USE_ARDUINO
  #include <Arduino.h> //needed for Serial.println
  #include <Arduino_LSM6DS3.h>
#else
  #error "Only Arduino nano 33 iot has built in IMU"
  #include "application.h"  // Particle
#endif
#include <SafeString.h>

// User defined functions
#include "Sync.h"
#include "src/Filters/myFilters.h"
#include "Sensors.h"
#include "src/Time/TimeLib.h"
#include "src/Tones/Tones.h"  // depends on some things above
#include "command.h"

// Global variables
String serial_str;
cSF(unit, INPUT_BYTES);
cSF(serial_st, INPUT_BYTES, "");
cSF(input_str, INPUT_BYTES, "");
cSF(prn_buff, INPUT_BYTES, "");
boolean string_cpt = false;
boolean plotting_all = false;
uint8_t plot_num = 0;
boolean monitoring = false;
time_t time_initial = ARBITRARY_TIME;
unsigned long long millis_flip = millis(); // Timekeeping
unsigned long long last_sync = millis();   // Timekeeping
int debug = 0;
Tone buzz = Tone(buzzerPin);
CommandPars cp = CommandPars();       // Various control parameters commanding at system level.  Initialized on start up.  Not retained.

// External variables
extern int debug;
extern CommandPars cp;            // Various parameters shared at system level

// Setup
void setup()
{
  setTime(time_initial);
  unit = version.c_str(); unit  += "_"; unit += HDWE_UNIT.c_str();

  init_serial(SERIAL_BAUD);

  init_LED();

  init_imu();

  init_buzzer();

  init_IR();

  init_motor();

  say_hello();
}


// Loop
void loop()
{
  // Timekeeping
  static unsigned long long now_ms = (unsigned long long) millis();
  boolean chitchat = false;
  static Sync *Talk = new Sync(TALK_DELAY);
  boolean read_eye = false;
  static Sync *ReadEye = new Sync(EYE_DELAY);
  boolean read_head = false;
  static Sync *ReadHead = new Sync(HEAD_DELAY);
  boolean publishing = false;
  static Sync *Plotting = new Sync(PUBLISH_DELAY);
  boolean control = false;
  static Sync *ControlSync = new Sync(CONTROL_DELAY);
  boolean blink = false;
  boolean blink_on = false;
  static Sync *BlinkSync = new Sync(BLINK_DELAY);
  boolean active = false;
  static Sync *ActiveSync = new Sync(ACTIVE_DELAY);
  unsigned long long elapsed = 0;
  static boolean reset = true;
  static unsigned long long time_start = millis();
  static boolean monitoring_past = monitoring;

  // Sensors
  boolean gyro_ready = false;
  boolean accel_ready = false;
  static Sensors *Sen = new Sensors(millis(), double(NOM_DT_EYE), t_kp_def, t_ki_def, sensorPin, unit_key + "_Rapid");
  boolean plotting = false;
  static boolean eye_closed = false;
  static boolean buzz_en_ir = true;
  static boolean buzz_en_grav = true;
  static float o_quiet_thr = O_QUIET_THR;
  static float g_quiet_thr = G_QUIET_THR;


  ///////////////////////////////////////////////////////////// Top of loop////////////////////////////////////////

  // Synchronize
  now_ms = (unsigned long long) millis();
  if ( now_ms - last_sync > ONE_DAY_MILLIS || reset )  sync_time(&last_sync, &millis_flip); 
  read_eye = ReadEye->update(millis(), reset);
  read_head = ReadHead->update(millis(), reset);
  chitchat = Talk->update(millis(), reset);
  elapsed = ReadHead->now() - time_start;
  control = ControlSync->update(millis(), reset);
  blink = BlinkSync->update(millis(), reset);
  active = ActiveSync->update(millis(), reset);
  publishing = Plotting->update(millis(), reset);
  plotting = plotting_all;
  boolean inhibit_talk = plotting_all && plot_num==10;
  static boolean run = true;  // Manual test feature for debugging Mahony filter

  // Read sensors
  if ( read_eye )
  {
    Sen->sample_eye(reset, millis());
    Sen->filter_eye(reset);
  }
  if ( read_head )
  {
    Sen->sample_head(reset, millis(), time_start, now());
    Sen->filter_head(reset, run);
    Sen->quiet_decisions(reset, o_quiet_thr, g_quiet_thr);

  }  // end read_head

  // Control
  if ( control )
  {
    if ( Sen->eye_closed_sure() )
    {
      turn_on_motor_and_led();
      if ( buzz_en_ir )
      {
        if ( !buzz.isPlaying() ) buzz.play_ir();
      }
    }
    else
    {
      if ( Sen->head_buzz_p() > 0 )
      {
        turn_on_motor_and_led();
        if ( buzz_en_grav && Sen->head_buzz_f() > 0 )
        {
          if ( !buzz.isPlaying() ) buzz.play_grav();
        }
      }
      else
      {
        turn_off_motor_and_led();
        if ( buzz.isPlaying() ) buzz.stop();
      }
    }
  }

  // Publish
  if ( publishing )
  {
    if ( monitoring && ( monitoring != monitoring_past ) ) Sen->print_all_header();
    if ( monitoring ) Sen->print_all();
    else if ( plotting_all )
    {
      static boolean done = false;
      switch ( plot_num )
      {
      case 0:
        Sen->plot_all_sum(); // pp0
        break;
      case 1:
        Sen->plot_all_acc(); // pp1
        break;
      case 2:
        Sen->plot_all_rot(); // pp2
        break;
      case 3:
        Sen->plot_all(); // pp3
        break;
      case 4:
        Sen->plot_quiet(); // pp4
        break;
      case 5:
        Sen->plot_quiet_raw();  // pp5
        break;
      case 6:
        Sen->plot_total();  // pp6
        break;
      case 7:
        Sen->plot_all_rpy();  // pp7
        break;
      case 8:
        Sen->plot_head_buzz();  // pp8
        break;
      case 9:
        Sen->plot_eye_buzz();  // pp9
        break;
      case 10:
        Sen->print_rapid(reset, true, Sen->time_eye_s());  // pp10
        debug = 10;
        break;
      default:
        Serial.println("plot number unknown enter plot number e.g. pp0 (sum), pp1 (acc), pp2 (rot), pp3 (all), pp4 (quiet), pp5 (quiet raw), pp6 (total), pp7 (roll-pitch-yaw), pp8 (head_buzz), pp9 (eye_buzz), pp10 (buzz list)");
        break;
      }
    }

    monitoring_past = monitoring;
  }

  gyro_ready = false;
  accel_ready = false;

  // Initialize complete once sensors and models started and summary written
  if ( read_head ) reset = false;

  if ( chitchat )
  {
    // Serial.println("chitchat");
    read_serial();  // returns one command at a time
    
    process_input_str(Sen, &g_quiet_thr, &o_quiet_thr, &reset, &run);
    if ( input_str.length() )
    {
      // Now we know the letters
      Serial.print("input_str: "); Serial.println(input_str);
      char letter_0 = '\0';
      char letter_1 = '\0';
      letter_0 = input_str.charAt(0);
      letter_1 = input_str.charAt(1);
      Serial.print(" letter_0: "); Serial.print(letter_0); Serial.print(" letter_1: "); Serial.println(letter_1);
      float f_value = 0.;
      input_str.substring(input_str, 2).toFloat(f_value);
      int i_value = int(f_value);
      Serial.print(" i_value: "); Serial.print(i_value); Serial.print(" f_value: "); Serial.println(f_value);
      Serial.println("");
      switch ( letter_0 )
      {
        case ( 'a' ):  // a - adjust
          switch ( letter_1 )
          {
            case ( 'p' ):  // ap - proportional gain
              Serial.print("Mahony prop gain from "); Serial.print(Sen->TrackFilter->getKp(), 3);
              Sen->TrackFilter->setKp(f_value);
              Serial.print(" to "); Serial.println(Sen->TrackFilter->getKp(), 3);
              break;
            case ( 'i' ):  // ai - integral gain
              Serial.print("Mahony int gain from "); Serial.print(Sen->TrackFilter->getKi(), 3);
              Sen->TrackFilter->setKi(f_value);
              Serial.print(" to "); Serial.println(Sen->TrackFilter->getKi(), 3);
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
        case ( 'b' ):  // b - buzz
          switch ( letter_1 )
          {
            case ( 'g' ):  // bG gravity frequency
              if ( i_value >= 0 && i_value <= 8000 )
              {
                buzz.gravityFreq(i_value);
                Serial.print("grav buzzer freq set to "); Serial.print(i_value); Serial.println(" Hz");
              }
              else Serial.println("buzzer freq must be between 0 and 4000 Hz");
              break;
            case ( 'i' ):  // ir frequency
              if ( i_value >= 0 && i_value <= 8000 )
              {
                buzz.irFreq(i_value);
                Serial.print("ir buzzer freq set to "); Serial.print(i_value); Serial.println(" Hz");
              }
              else Serial.println("buzzer freq must be between 0 and 4000 Hz");
              break;
           default:
              Serial.print(letter_0); Serial.println(" unknown");
              break;
          }
          break;          
        case ( 'h' ):  // h  - help
          plotting_all = false;
          monitoring = false;
          Serial.println("a?<val> - adjust");
          Serial.print("\t p = Mahony proportional gain ("); Serial.print(Sen->TrackFilter->getKp(), 3); Serial.println(")");
          Serial.print("\t i = Mahony integral gain ("); Serial.print(Sen->TrackFilter->getKi(), 3); Serial.println(")");
          Serial.print("\t l = LTST fault thr ("); Serial.print(Sen->LTST_Filter->get_fault_thr_pos(), 3); Serial.println(")");
          Serial.print("\t R = eye RESET time, s ("); Serial.print(Sen->get_eye_reset_time(), 3); Serial.println(")");
          Serial.print("\t S = eye SET time, s ("); Serial.print(Sen->get_eye_set_time(), 3); Serial.println(")");
          Serial.print("\t r = roll thr ("); Serial.print(Sen->roll_thr(), 3); Serial.println(")");
          Serial.print("\t t = pitch thr ("); Serial.print(Sen->pitch_thr(), 3); Serial.println(")");
          Serial.print("\t z = LTST freeze thr ("); Serial.print(Sen->LTST_Filter->get_freeze_thr_pos(), 3); Serial.println(")");
          Serial.println("b?<val> - buzz preferences");
          Serial.print("\t i = ir sensor freq, Hz ("); Serial.print(buzz.irFreq()); Serial.println(")");
          Serial.print("\t g = gravity sensor freq, Hz ("); Serial.print(buzz.gravityFreq()); Serial.println(")");
          Serial.println("h - this help");
          Serial.println("P? - Print stuff");
          Serial.println("\t L - LTST Filter");
          Serial.println("pp? - plot all version X");
          Serial.println("\t -1 - stop plotting");
          Serial.println("\t 0 - summary (g_raw, g_filt, g_quiet, q_is_quiet_sure, o_raw, o_filt, o_quiet, o_is_quiet_sure)");
          Serial.println("\t 1 - g sensors (T_acc, x_filt, y_filt, z_filt, g_filt, g_is_quiet, g_is_quiet_sure)");
          Serial.println("\t 2 - rotational sensors (T_rot, a_filt, b_filt, c_filt, o_filt, o_is_quiet, o_is_quiet_sure)");
          Serial.println("\t 3 - all sensors (x_filt, y_filt, z_filt, g_filt, a_filt, b_filt, c_filt, o_filt)");
          Serial.println("\t 4 - quiet results ( T_rot, o_filt, o_quiet, o_is_quiet_sure, T_acc, g_filt, g_quiet, g_is_quiet_sure)");
          Serial.println("\t 5 - quiet filtering metrics (o_quiet, g_quiet)");
          Serial.println("\t 6 - total (T_rot, o_filt, T_acc, g_filt)");
          Serial.println("\t 7 - roll-pitch-yaw(T_acc, tx_raw, ty_raw, tz_raw, roll_filt, pitch_filt, yaw_filt, q0, q1, q2, q3)");
          Serial.println("\t 8 - head buzz (g_is_quiet_sure, o_is_quiet_sure, max_nod_f, max_nod_p, head_buzz, cf, eye_buzz)");
          Serial.println("\t 9 - eye buzz (eye_voltage, ltstate, ststate, dltst, eye_closed, eye_closed_confirmed, eye_buzz, max_nod_f, max_nod_p, head_buzz)");
          Serial.println("\t 10 - stream buzz (key_Rapid, cTime, v3v3, eye_voltage_norm, eye_closed, eye_closed_confirmed, max_nod_f, max_nod_p, head_buzz, eye_buzz, lt_state, st_state, dltst, freeze)");
          Serial.println("r  - soft reset");
          Serial.println("t?<val> - trim attitude");
          Serial.print("\t g = G quiet thr, small more sensitive ("); Serial.print(g_quiet_thr, 3); Serial.println(")");
          Serial.print("\t o = O angular speed quiet thr, small more sensitive ("); Serial.print(o_quiet_thr, 3); Serial.println(")");
          Serial.print("\t p = pitch bias nodding ("); Serial.print(Sen->get_delta_pitch(), 3); Serial.println(")");
          Serial.print("\t r = roll bias tilting ("); Serial.print(Sen->get_delta_roll(), 3); Serial.println(")");
          Serial.println("vv?  - verbosity debug level");
          Serial.println("x?  - manually test Mahony tracking filter ( true = real time, false = pulse )");
          break;
        case ( 'm' ):  // m  - print all
          plotting_all = false;
          monitoring = !monitoring;
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
           default:
              Serial.print(letter_0); Serial.println(" unknown");
              break;
          }
          break;          
        case ( 'p' ):  // p - plot
          switch ( letter_1 )
          {
            case ( 'p' ):  // pp - plot all filtered
              switch ( i_value )
              {
                case 0 ... 10:
                  plot_num = i_value;
                  plotting_all = true;
                  monitoring = false;
                  break;
                default:
                  Serial.println("plot number unknown enter plot number e.g. pp0 (sum), pp1 (acc), pp2 (rot), pp3 (all), pp4 (quiet), pp5 (quiet raw), pp6 (total), pp7 (roll-pitch-yaw), pp8 (head_buzz), pp9 (eye_buzz), pp10 (buzz list)");
                  plotting_all = false;
                  break;
              }
              break;
            default:
              Serial.print(input_str.charAt(1)); Serial.print(" for "); Serial.print(input_str); Serial.println(" unknown");
              break;
          }
          break;
        case ( 'r' ):  // r - reset command toggle
          reset = true;
          break;
        case ( 't' ):  // t - trim
          switch ( letter_1 )
          {
            case ( 'g' ):  // tg - G quiet threshold
              Serial.print("G quiet threshold from "); Serial.print(g_quiet_thr, 3);
              g_quiet_thr = f_value;
              Serial.print(" to "); Serial.print(g_quiet_thr, 3);
              break;
            case ( 'o' ):  // to - Angular speed quiet threshold
              Serial.print("O quiet threshold from "); Serial.print(o_quiet_thr, 3);
              o_quiet_thr = f_value;
              Serial.print(" to "); Serial.print(o_quiet_thr, 3);
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
          run = !run;
          break;
        default:
          Serial.print(letter_0); Serial.println(" unknown");
          break;
      }
    }
    input_str = "";
    
  }
}  // loop
