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
#include "Sequence.h"

// Global variables
String serial_str;
cSF(unit, INPUT_BYTES);
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
  static Sequence *S = new Sequence();
  static unsigned long long now_ms = (unsigned long long) millis();
  boolean chitchat = false;
  static Sync *Talk = new Sync(TALK_DELAY);
  boolean read_eye = false;
  static Sync *ReadEye = new Sync(EYE_DELAY);
  boolean read_head = false;
  static Sync *ReadHead = new Sync(HEAD_DELAY);
  // boolean publishing = false;
  // static Sync *Plotting = new Sync(PUBLISH_DELAY);
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
  // now_ms = (unsigned long long) millis();
  // if ( now_ms - last_sync > ONE_DAY_MILLIS || reset )  sync_time(&last_sync, &millis_flip); 
  read_eye = ReadEye->update(millis(), reset);
  read_head = ReadHead->update(millis(), reset);
  chitchat = Talk->update(millis(), reset);
  elapsed = ReadHead->now() - time_start;
  control = ControlSync->update(millis(), reset);
  blink = BlinkSync->update(millis(), reset);
  active = ActiveSync->update(millis(), reset);
  // publishing = Plotting->update(millis(), reset);

  plotting = plotting_all;
  boolean inhibit_talk = plotting_all && plot_num==10;
  static boolean run = true;  // Manual test feature for debugging Mahony filter
  S->calculate(&last_sync, &millis_flip, reset);

  // Read sensors
  if ( S->read_eye() )
  {
    Sen->sample_eye(reset, millis());
    Sen->filter_eye(reset);
  }
  if ( S->read_head() )
  {
    Sen->sample_head(reset, millis(), time_start, now());
    Sen->filter_head(reset, run);
    Sen->quiet_decisions(reset, o_quiet_thr, g_quiet_thr);

  }  // end read_head

  // Control
  if ( S->control() )
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
  if ( S->publishing() )
  {
    if ( monitoring && ( monitoring != monitoring_past ) ) Sen->print_all_header();
    if ( monitoring ) Sen->print_all();
    else if ( plotting_all )
    {
      request_plot(plot_num, Sen, reset);
    }

    monitoring_past = monitoring;
  }

  gyro_ready = false;
  accel_ready = false;

  // Initialize complete once sensors and models started and summary written
  if ( read_head ) reset = false;

  if ( S->chitchat() )
  {
    read_serial();  // returns one command at a time
    process_input_str(Sen, &g_quiet_thr, &o_quiet_thr, &reset, &run);
  }
}  // loop
