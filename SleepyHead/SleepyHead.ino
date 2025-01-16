/*
  Uses LSM6DS3 in Arduino Nano 33 IoT
  This unit is reported to have BLE but I have not confirmed.
  This unit requires special EEPROM handling using FlashStorage library.

  The circuit:
  - Arduino Uno WiFi Rev 2 or Arduino Nano 33 IoT
  - USB for monitor and power

  The Arduino Libraries:
  - Arduino_LSM6DS3, AceCommon, AceCRC, AceRoutine, AceUtils, Adafruit BusIO, Adafruit LSMDS?, SafeString
  
  Arduino board for CTE Nano:
  - Arduino Nano 33 IoT in Afduino SAMD library
 
  created 10 Jul 2024
  by Dave Gutz

  This example code is in the public domain.

  Requirements:
  1.  >500 Hz sampling of 6 dof IMU to support 25 Hz high fidelity analysis
  2.  Capture and store and limited number of collisions for post download and analysis
  3.  5 collisions for now
  4.  Button cell battery
  5.  UT managed by EEPROM.  OK to synchronize on restart before use.
  6.  Store 'worst' 2 collisions in EEPROM.

  Notes:
  1.  IMU in Nano captures 6 dof IMU at 100 Hz.   Too slow but good for 20 Hz analysis
  2.  A collision is approximately 3 seconds = 300 samples of 7 variables (6 dof + integer time).
    Use experience to scale to 16 bit integers for storage of several collisions possible.  Need experiment soon.
    Each collision would need 7 * 16 * 300 bits ~ 32k bits
  3. Don't know how big EEPROM is.  Want to save 'worst' collision in EEPROM.  Always preserve 'worst'
  in both EEPROM and RAM.
  4. Arduino compiler with this barebones program says 27400 bytes left in RAM = 220 k bits enough for 4 collisions.
  5. Arduino specs say EEPROM storage is 520 KB SRAM.
    AMD21G18A Processor
    256 kB Flash 32 kB Flash (256 is future possible EEPROM.  32 is now.)
    Arduino reports:
    EEPROM size: 201
    EEPROM PAGE_SIZE: 64
    EEPROM PAGES: 4096
    EEPROM MAX_FLASH: 262144 bits = 32 kB  = 262144 / 7 / 16 = 2340 samples ~8 collisions.  Confirms info about 2nd Flash being available for EEPROM.
    EEPROM ROW_SIZE: 256

*/

#include <SafeString.h>
#include "FlashStorage.h"
#include "constants.h"

#ifdef USE_ARDUINO
  #include <Arduino.h> //needed for Serial.println
  #include <Arduino_LSM6DS3.h>
#else
  #error "Only Arduino nano 33 iot has built in IMU"
  #include "application.h"  // Particle
#endif

// Dependent includes.   Easier to sp.debug code if remove unused include files
#include "Sync.h"
#include "myFilters.h"
#include "Sensors.h"
#include "CollDatum.h"
#include "TimeLib.h"

// Global
cSF(unit, INPUT_BYTES);
cSF(serial_str, INPUT_BYTES, "");
cSF(input_str, INPUT_BYTES, "");
cSF(prn_buff, INPUT_BYTES, "");
boolean string_cpt = false;
boolean plotting_all = false;
boolean run = true;
uint8_t plot_num = 0;
boolean monitoring = false;
time_t time_initial = ARBITRARY_TIME;
unsigned long long millis_flip = millis(); // Timekeeping
unsigned long long last_sync = millis();   // Timekeeping
const int v3v3Pin = 14;     // Pin connected to the IR sensor (or eye detection sensor)
const int sensorPin = 20;     // Pin connected to the IR sensor (or eye detection sensor)
const int buzzerPin = A3;     // Pin connected to the buzzer
const int motorPin = 21;     // Pin connected to the buzzer

extern int debug;
extern boolean run;
int debug = 0;
boolean print_mem = false;


// Generate tones
class Tone
{
  public:
    Tone(const int pin): buzzerPin_(pin), buzz_freq_grav_(buzz_freq_grav), buzz_freq_ir_(buzz_freq_ir), isPlaying_(false) {}
    void begin()
    {
      pinMode(buzzerPin_, OUTPUT);
      digitalWrite(buzzerPin_, LOW);
    }
    int gravityFreq() { return buzz_freq_grav_; }
    void gravityFreq(const int inp) { buzz_freq_grav_ = inp; }
    int irFreq() { return buzz_freq_ir_; }
    void irFreq(const int inp) { buzz_freq_ir_ = inp; }
    boolean isPlaying() { return isPlaying_; }
    void play_grav() { tone(buzzerPin_, buzz_freq_grav_); Serial.println("grav tone played"); isPlaying_ = true; }
    void play_ir() { tone(buzzerPin_, buzz_freq_ir_); Serial.println("ir tone played"); isPlaying_ = true; }
    void stop() { noTone(buzzerPin_); Serial.println("tone stopped"); isPlaying_ = false; }

  private:
    int buzzerPin_;
    int buzz_freq_grav_;
    int buzz_freq_ir_;
    boolean isPlaying_;
} buzz(buzzerPin);

// Set buzzer volume (0-255 for variable PWM dutry cycle based on 'volume')
void setBuzzerVolume(int volume)
{
  analogWrite(buzzerPin, volume);
}

// Setup
void setup()
{
  unit = version.c_str(); unit  += "_"; unit += HDWE_UNIT.c_str();
  setTime(time_initial);


  // Serial
  Serial.println("Serial starting over USB...");
  Serial.begin(SERIAL_BAUD);
  delay_no_block(2000UL);  // Usually takes less than 700 ms
  if (Serial) Serial.println("Serial ready");

  // LED
  Serial.print("LED starting at pin "); Serial.print(LED_BUILTIN); Serial.print("...");
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println(" done");

  // IMU
  if ( !IMU.begin() )
  {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("IMU ready");
  delay(50);

  // Buzzer
  Serial.print("Buzzer starting at pin "); Serial.print(buzzerPin); Serial.print("...");
  buzz.begin();
  Serial.println(" done");
  delay(5);

  // IR
  Serial.print("IR starting at pin "); Serial.print(sensorPin); Serial.print("...");
  pinMode(sensorPin, INPUT);   // Set sensorPin as an INPUT
  Serial.println(" done");
  // change the resolution to 12 bits (4095)
  analogReadResolution(12);
  delay(5);
 
  // v3v3
  Serial.print("v3v3_nom starting at pin "); Serial.print(v3v3Pin); Serial.print("...");
  pinMode(v3v3Pin, INPUT);   // Set sensorPin as an INPUT
  Serial.println(" done");
  // change the resolution to 12 bits (4095)
  analogReadResolution(12);
  delay(5);

  // Motor
  Serial.print("Motor starting at pin "); Serial.print(motorPin); Serial.print("...");
  pinMode(motorPin, OUTPUT);   // Set motorPin as an OUTPUT
  Serial.println(" done");
  delay(5);

  // Say 'Hello'
  say_hello();

}


// Loop
void loop()
{
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
  boolean gyro_ready = false;
  boolean accel_ready = false;
  static boolean monitoring_past = monitoring;
  static time_t new_event = 0;
  static Sensors *Sen = new Sensors(millis(), double(NOM_DT_EYE), t_kp_def, t_ki_def, sensorPin, unit_key + "_Rapid", v3v3Pin);
  static Data_st *L = new Data_st(NDATUM, NHOLD, NREG);  // Event log
  static boolean logging = false;
  static boolean logging_past = false;
  static uint16_t log_size = 0;
  boolean plotting = false;
  static boolean eye_closed = false;
  static boolean buzz_en_ir = true;
  static boolean buzz_en_grav = true;


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

  if ( reset )
  {
    Serial.print("size of ram NDATUM="); Serial.println(NDATUM);
    Serial.print("num precursors NHOLD="); Serial.println(NHOLD);
    Serial.print("num reg entries NREG="); Serial.println(NREG);
    Serial.print("iR="); Serial.println(L->iR());
    Serial.print("iRg="); Serial.println(L->iRg());
  }

  if ( print_mem )
  {
    Serial.print("size of ram NDATUM="); Serial.println(NDATUM);
    Serial.print("num precursors NHOLD="); Serial.println(NHOLD);
    Serial.print("num reg entries NREG="); Serial.println(NREG);
    Serial.print("iR="); Serial.println(L->iR());
    Serial.print("iRg="); Serial.println(L->iRg());
    Serial.print("Data_st size: "); Serial.println(L->size());
    print_mem = false;  // print once
  }

  // Read sensors
  if ( read_eye )
  {
    Sen->sample_eye(reset, millis());
    Sen->filter_eye(reset);
  }
  if ( read_head )
  {
    Sen->sample_head(reset, millis(), time_start, now());
    Sen->filter_head(reset);
    Sen->quiet_decisions(reset);
    L->put_precursor(Sen);

    // Logic
    if ( Sen->both_not_quiet() && !logging )
    {
      logging = true;
      new_event = Sen->t_ms;
      log_size++;
    }
    else
    {
      if ( Sen->both_are_quiet() && logging )
      {
        logging = false;  // This throws out the last event
      }
      log_size = 0;
    }

    // Log data - full resolution since part of 'read_head' frame
    if ( logging && !logging_past)
    {
      L->register_lock(inhibit_talk, Sen);  // after move_precursor so has values on first save
      if ( !inhibit_talk ) { Serial.println(""); Serial.println("Logging started"); }
  
      L->move_precursor();
      L->put_ram(Sen);
    }
    else if ( !logging && logging_past )
    {
      L->put_ram(Sen);
      if ( !inhibit_talk ) Serial.println("Logging stopped");
      L->register_unlock(inhibit_talk, Sen);
      if ( !plotting )
      {
        // Serial.println("All ram");
        // L->print_ram();
        Serial.println("Latest ram");
        L->print_latest_ram();
        Serial.println("Registers");
        L->print_all_registers();
        Serial.println("Latest register");
        L->print_latest_register();
      }
      else if ( plotting_all && plot_num==10 )
      {
        L->plot_latest_ram();  // pp10
      }
    }
    else if ( logging )
    {
      L->put_ram(Sen);
    }

    logging_past = logging;

  }  // end read_head

  // Control
  if ( control )
  {
    if ( Sen->eye_closed_sure() )
    {
      digitalWrite(motorPin, HIGH);
      digitalWrite(LED_BUILTIN, HIGH);
      if ( buzz_en_ir )
      {
        if ( !buzz.isPlaying() ) buzz.play_ir();
      }
    }
    else
    {
      if ( Sen->max_nod_piano() > 0 )
      {
        digitalWrite(motorPin, HIGH);
        digitalWrite(LED_BUILTIN, HIGH);
        if ( buzz_en_grav && Sen->max_nod_forte() > 0 )
        {
          if ( !buzz.isPlaying() ) buzz.play_grav();
        }
      }
      else
      {
        digitalWrite(motorPin, LOW);
        digitalWrite(LED_BUILTIN, LOW);
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
        Sen->plot_all_sum();
        break;
      case 1:
        Sen->plot_all_acc();
        break;
      case 2:
        Sen->plot_all_rot();
        break;
      case 3:
        Sen->plot_all();
        break;
      case 4:
        Sen->plot_quiet();
        break;
      case 5:
        Sen->plot_quiet_raw();
        break;
      case 6:
        Sen->plot_total();
        break;
      case 7:
        Sen->plot_all_rpy();
        break;
      case 8:
        Sen->plot_buzz();
        break;
      case 9:
        Sen->print_rapid(reset, true, Sen->time_eye_s());
        debug = 9;
        break;
      case 10:
        break;
      default:
        Serial.println("plot number unknown enter plot number e.g. pp0 (sum), pp1 (acc), pp2 (rot), pp3 (all), pp4 (quiet), pp5 (quiet raw), pp6 (total), pp7 (roll-pitch-yaw), pp8 (buzz), pp9 (buzz list) or pp10 (sum plot)");
        break;
      }
    }

    monitoring_past = monitoring;
  }

  gyro_ready = false;
  accel_ready = false;

  // Initialize complete once sensors and models started and summary written
  if ( read_head ) reset = false;

  // Blink when threshold breached and therefore logging
  if ( blink )
  {
    if ( !logging )
    {
      blink_on = false;
    }
    else
    {
      blink_on = !blink_on;
    }
    if ( blink_on ) digitalWrite(LED_BUILTIN, HIGH);
    else digitalWrite(LED_BUILTIN, LOW);
  }

  if ( active )
  {
    static int i_count = 0;

    // Blink number of stored registers
    if ( L->num_active_registers() > 0 )   // num_active_registers
    {
      if ( i_count < L->num_active_registers() ) blink_on = !blink_on;
      if ( blink_on ) digitalWrite(LED_BUILTIN, HIGH);
      else digitalWrite(LED_BUILTIN, LOW);
 
      // Reset counter on over-wrap
      if ( ++i_count >= int(NREG) ) i_count = 0;
    }
  }

  if ( chitchat )
  {
    // Serial.println("chitchat");
    read_serial();  // returns one command at a time
    

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
            case ( 'r' ):  // ar - roll threshold
              Serial.print("Roll threshold from "); Serial.print(Sen->roll_thr(), 3);
              Sen->roll_thr(f_value);
              Serial.print(" to "); Serial.println(Sen->roll_thr(), 3);
              break;
            case ( 't' ):  // at - pitch threshold
              Serial.print("Pitch threshold from "); Serial.print(Sen->pitch_thr(), 3);
              Sen->pitch_thr(f_value);
              Serial.print(" to "); Serial.println(Sen->pitch_thr(), 3);
              break;
            #ifndef USE_IR_ON_OFF
              case ( 'v' ):  // av - ir voltage threshold
                Serial.print("IR threshold from "); Serial.print(Sen->voltage_thr(), 3);
                Sen->voltage_thr(f_value);
                Serial.print(" to "); Serial.println(Sen->voltage_thr(), 3);
                break;
            #endif
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
          Serial.println("aXX <val> - adjust");
          Serial.print("\t p = Mahony proportional gain (Kp="); Serial.print(Sen->TrackFilter->getKp(), 3);Serial.println(")");
          Serial.print("\t i = Mahony integral gain (Ki=");Serial.print(Sen->TrackFilter->getKi(), 3);Serial.println(")");
          Serial.print("\t r = roll thr (roll_thr="); Serial.print(Sen->roll_thr(), 3);Serial.println(")");
          Serial.print("\t t = pitch thr (pitch_thr="); Serial.print(Sen->pitch_thr(), 3);Serial.println(")");
          #ifndef USE_IR_ON_OFF
            Serial.print("\t v = volt thr (volt_thr="); Serial.print(Sen->voltage_thr(), 3);Serial.println(")");
          #endif
          Serial.println("bX<x> - buzz toggles");
          Serial.print("\t i<freq> = ir sensor freq, Hz ("); Serial.print(buzz.irFreq());Serial.println(")");
          Serial.print("\t g<freq> = gravity sensor freq, Hz ("); Serial.print(buzz.gravityFreq());Serial.println(")");
          Serial.println("h - this help");
          Serial.println("ppX - plot all version X");
          Serial.println("\t X=blank - stop plotting");
          Serial.println("\t X=0 - summary (g_raw, g_filt, g_quiet, q_is_quiet_sure, o_raw, o_filt, o_quiet, o_is_quiet_sure)");
          Serial.println("\t X=1 - g sensors (T_acc, x_filt, y_filt, z_filt, g_filt, g_is_quiet, g_is_quiet_sure)");
          Serial.println("\t X=2 - rotational sensors (T_rot, a_filt, b_filt, c_filt, o_filt, o_is_quiet, o_is_quiet_sure)");
          Serial.println("\t X=3 - all sensors (x_filt, y_filt, z_filt, g_filt, a_filt, b_filt, c_filt, o_filt)");
          Serial.println("\t X=4 - quiet results ( T_rot, o_filt, o_quiet, o_is_quiet_sure, T_acc, g_filt, g_quiet, g_is_quiet_sure)");
          Serial.println("\t X=5 - quiet filtering metrics (o_quiet, g_quiet)");
          Serial.println("\t X=6 - total (T_rot, o_filt, T_acc, g_filt)");
          Serial.println("\t X=7 - roll-pitch-yaw");
          Serial.println("\t X=8 - buzz");
          Serial.println("\t X=9 - stream buzz");
          Serial.println("\t X=10 - summary for plot");
          Serial.println("ph - print history");
          Serial.println("pr - print registers");
          Serial.println("m  - print all");
          Serial.println("r  - reset cmd toggle");
          Serial.println("s  - print sizes for all (will vary depending on history of collision)");
          Serial.println("UTxxxxxxx - set time to x (x is integer from https://www.epochconverter.com/)");
          Serial.println("vvX  - verbosity debug level");
          Serial.println("x  - play toggle ( true = real time, false = pulse )");
          Serial.println("\t X=9  - time trace in Sensors");
          break;
        case ( 'm' ):  // m  - print all
          plotting_all = false;
          monitoring = !monitoring;
          break;
        case ( 'p' ):  // p - plot
          switch ( letter_1 )
          {
            case ( 'p' ):  // pp - plot all filtered
              switch ( i_value )
              {
                case 0 ... 9:
                  plot_num = i_value;
                  plotting_all = true;
                  monitoring = false;
                  break;
                default:
                  Serial.println("plot number unknown enter plot number e.g. pp0 (sum), pp1 (acc), pp2 (rot), pp3 (all), pp4 (quiet), pp5 (quiet raw), pp6 (total), pp7 (roll-pitch-yaw), pp8 (buzz), pp9 (buzz list) or pp10 (sum plot)");
                  plotting_all = false;
                  break;
              }
              break;
            case ( 'h' ):  // ph - print history
              Serial.println("History:");
              L->print_ram();
              break;
            case ( 'r' ):  // pr - print registers
              Serial.println("Registers:");
              L->print_all_registers();
              break;
            default:
              Serial.print(input_str.charAt(1)); Serial.print(" for "); Serial.print(input_str); Serial.println(" unknown");
              break;
          }
          break;
        case ( 'r' ):  // r - reset command toggles
          reset = true;
          break;
        case ( 's' ):  // s - sizes for all
          print_mem = true;
          break;
        case ( 'U' ):
          switch ( letter_1 )
          {
            case ( 'T' ):
              input_str.substring(input_str, 2).toInt(i_value);
              time_initial = time_t ( i_value );
              setTime(time_initial);
              prn_buff = "---";
              time_long_2_str(time_initial*1000, prn_buff);
              Serial.println("Time set to: "); Serial.print(time_initial); Serial.print(" = "); Serial.println(prn_buff);
              break;
            default:
              Serial.print(input_str.charAt(1)); Serial.println(" unknown");
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
        case ( 'x' ):  // x - play command toggles
          run = !run;
          break;
        default:
          Serial.print(letter_0); Serial.println(" unknown");
          break;
      }
    }
    input_str = "";
    
  }
    // Serial.println("end");

}  // loop


// Read serial for chitchat
void read_serial()
{
  boolean serial_ready = false;
  serial_str = "";

  // Each pass try to complete input from avaiable
  while ( !serial_ready && Serial.available() )
  {
    char in_char = (char)Serial.read();  // get the new byte

    // Intake
    // if the incoming character to finish, add a ';' and set flags so the main loop can do something about it:
    if ( is_finished(in_char) )
    {
        if ( serial_str.length() ) serial_str.concat(';');
        serial_ready = true;
        break;
    }

    else if ( in_char == '\r' )
    {
        Serial.println("\n");  // scroll user terminal
    }

    else if ( in_char == '\b' && serial_str.length() )
    {
        Serial.print("\b \b");  // scroll user terminal
        serial_str.remove(serial_str.length() -1 );  // backspace
    }

    else
    {
        serial_str += in_char;  // process new valid character
    }

  }

  // Pass info to serial_str
  if ( serial_ready )
  {
    input_str += serial_str.c_str();
    finish_request(input_str);
    serial_ready = false;
    serial_str = "";
  }
}

// Cleanup string for final processing by chitchat
void finish_request(SafeString &str)
{
  // Remove whitespace
  str.trim();
  str.replace("\n", "");
  str.replace("\0", "");
  str.replace("", "");
  str.replace(",", "");
  str.replace(" ", "");
  str.replace("=", "");
  str.replace(";", "");
}

// Test for string completion character
boolean is_finished(const char in_char)
{
    return  in_char == '\n' ||
            in_char == '\0' ||
            in_char == ';'  ||
            in_char == ',';    
}

// Say hello
void say_hello()
{
  Serial.println("");
  Serial.println("Hello!");
  Serial.println();
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println("Gyroscope in degrees/second");
  Serial.println();
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println("Acceleration in g's");
  Serial.println();
}

// Time synchro so printed decimal times align with hms rolls
void sync_time(unsigned long long *last_sync, unsigned long long *millis_flip)
{
  *last_sync = millis();

  // Refresh millis() at turn of Time.now
  int count = 0;
  long time_begin = now();  // Seconds since start of epoch
  while ( now()==time_begin && ++count<1100 )  // Time.now() truncates to seconds
  {
    delay(1);
    *millis_flip = millis()%1000;
  }
}

// Non-blocking delay
void delay_no_block(const unsigned long long interval)
{
  unsigned long long previousMillis = millis();
  unsigned long long currentMillis = previousMillis;
  while( currentMillis - previousMillis < interval )
  {
    currentMillis = millis();
  }
}
