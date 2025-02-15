/*
  Uses LSM6DS3 in Arduino Nano 33 IoT
  This unit is reported to have BLE but I have not confirmed.
  This unit requires special EEPROM handling using FlashStorage library.  But not used SleepyHeadl
  - Arduino Uno WiFi Rev 2 or Arduino Nano 33 IoT
  - USB for monitor and power

  The Arduino Libraries:
  - Arduino_LSM6DS3, AceCommon, AceCRC, AceRoutine, AceUtils, Adafruit BusIO, Adafruit LSMDS?
  
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

// Global variables and user includes
#include "SleepyHead.h"
#include "ProcessInput.h"
#include "ProcessOutput.h"
#include "Subs.h"
extern CommandPars cp;            // Various parameters shared at system level
extern int debug;

// Setup
void setup()
{
  setTime(time_initial);
  unit = version.c_str(); unit  += "_"; unit += HDWE_UNIT.c_str();

  init_serial(SERIAL_BAUD);

  init_LED();

  init_imu();

  init_tone();

  init_IR();

  init_motor();

  say_hello();
}


// Loop
void loop()
{
  // Timekeeping
  static Sequence *S = new Sequence();
  static boolean reset = true;
  static unsigned long long time_start = millis();
  static boolean monitoring_past = monitoring;
  static uint8_t last_plot_num = 126;
  static ProcessPinOutput *LED = new ProcessPinOutput(LED_BUILTIN, &cp.led_token, String("LED"));
  static ProcessPinOutput *MOT = new ProcessPinOutput(motorPin, &cp.motor_token, String("MOT"));
  static ProcessToneOutput *TON = new ProcessToneOutput(tonePin, &cp.tone_token, String("TON"));

  // Sensors
  static Sensors *Sen = new Sensors(millis(), kp_def, ki_def, sensorPin, unit_key + "_Rapid");
  boolean plotting = plotting_all_def;
  static boolean eye_closed = false;
  static boolean enable_tone_ir = true;
  static boolean enable_tone_grav = true;
  static float o_quiet_thr = O_QUIET_THR;
  static float g_quiet_thr = G_QUIET_THR;

  ///////////////////////////////////////////////////////////// Top of loop////////////////////////////////////////

  // Synchronize
  plotting = plotting_all;
  static boolean run = true;  // Manual test feature for debugging Mahony filter
  S->calculate(&last_sync, &millis_flip, reset);

  // Read sensors
  if ( S->read_and_calc() )
  {
    Sen->sample_eye(reset, millis());
    static boolean reset_eye_filter = true;
    if ( S->elapsed() > EYE_INIT_TIME) reset_eye_filter = false;
    else reset_eye_filter = true;
    Sen->filter_eye(reset_eye_filter, S->elapsed());
    Sen->sample_head(reset, millis(), time_start, now());
    Sen->filter_head(reset, run);
    Sen->quiet_decisions(reset, o_quiet_thr, g_quiet_thr);
  }

  // Control
  if ( S->control() )
  {
    if ( Sen->get_eye_ready() && enable_motor )
    {
      TON->write_duty(tone_freq_ir, DUTY_EYE_READY);
      LED->write_duty(DUTY_EYE_READY);
    }
    else if ( Sen->get_eye_reset() && enable_motor )
    {
      TON->write_duty(tone_freq_ir, DUTY_EYE_RESET);
      LED->write_duty(DUTY_EYE_RESET);
    }
    else
    {
      TON->turn_off();
      LED->turn_off();
    }

    if ( Sen->eye_closed_sure() )
    {
      if ( enable_motor )
        MOT->write_duty(100);
      if ( enable_tone_ir )
      {
        TON->write_duty(tone_freq_ir, DUTY_EYE_WARN);
        LED->write_duty(DUTY_EYE_WARN);
      }
    }
    else
    {
      if ( Sen->head_buzz_p() > 0 )
      {
        if ( enable_motor ) MOT->write_duty(DUTY_HEAD_WARN);
        if ( Sen->head_buzz_f() > 0 )
        {
          if ( enable_motor ) MOT->write_duty(100);
          if ( enable_tone_grav )
          {
            TON->write_duty(tone_freq_grav, DUTY_HEAD_WARN);
            LED->write_duty(DUTY_HEAD_WARN);
          }
        }
      }
      else
      {
        if ( Sen->get_head_ready() && enable_motor )  MOT->write_duty(DUTY_HEAD_READY);
        else if ( Sen->get_head_reset() && enable_motor )  MOT->write_duty(DUTY_HEAD_RESET);
        else  MOT->turn_off();
      }
    }
  }

  // Publish
  if ( S->publishing() )
  {
    if ( monitoring && ( monitoring != monitoring_past ) ) Sen->print_all_header();
    if ( monitoring )
    {
      Sen->print_all();
      last_plot_num = 126;
    }
    else if ( plotting_all )
    {
      request_plot(plot_num, plot_num!=last_plot_num, Sen, reset);
      last_plot_num = plot_num;
    }
    else last_plot_num = 126;

    monitoring_past = monitoring;
  }

  // Initialize complete once sensors and models started and summary written
  if ( S->read_and_calc() ) reset = false;

  // Interact with user over USB
  if ( S->chitchat() )
  {
    read_serial();  // returns one command at a time
    process_input_str(Sen, &g_quiet_thr, &o_quiet_thr, &reset, &run);
  }

}  // loop
