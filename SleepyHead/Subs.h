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

void read_serial_add_verify(String *src, const String addend);

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

// Config pins
uint8_t init_config_pins()
{
    Serial.print("Config pin 0 starting at pin "); Serial.print(configPin0); Serial.print("...");
    pinMode(configPin0, INPUT);   // Set configPin0 as an INPUT
    Serial.print("Config pin 1 starting at pin "); Serial.print(configPin1); Serial.print("...");
    pinMode(configPin1, INPUT);   // Set configPin1 as an INPUT
    Serial.println(" done");
    delay(5);
    uint8_t config = (digitalRead(configPin1) << 1) | digitalRead(configPin0);
    Serial.print("Reading config pins and setting config = "); Serial.println(config);
    return config;
}

boolean init_tone()
{
  Serial.print("Tone starting at pin "); Serial.print(tonePin); Serial.print("...");
  pinMode(tonePin, OUTPUT);
  digitalWrite(tonePin, LOW);
  Serial.println(" done");
  delay(5);
  return true;
}

// IMU
boolean init_imu()
{
    if ( !IMU.begin() )
    {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }
    Serial.println("IMU ready");
    delay(50);
    return true;
}

// IR
boolean init_IR()
{
  Serial.print("IR starting at pin "); Serial.print(sensorPin); Serial.print("...");
  pinMode(sensorPin, INPUT);  // Set sensorPin as an INPUT
  analogReadResolution(12);  // change the resolution to 12 bits (4095)
  Serial.println(" done");
  delay(5);
  return true;
}

// LED
boolean init_LED()
{
  Serial.print("LED starting at pin "); Serial.print(LED_BUILTIN); Serial.print("...");
  pinMode(LED_BUILTIN, OUTPUT);
  if (Serial)
  {
    Serial.println(" done");
    return true;
  }
  else return false;
}

// Motor
boolean init_motor()
{
  Serial.print("Motor starting at pin "); Serial.print(motorPin); Serial.print("...");
  pinMode(motorPin, OUTPUT);   // Set motorPin as an OUTPUT
  Serial.println(" done");
  delay(5);
  return true;
}

// Serial
boolean init_serial(const unsigned int baud)
{
  Serial.println("Serial starting over USB...");
  Serial.begin(SERIAL_BAUD);
  delay_no_block(2000UL);  // Usually takes less than 700 ms
  if (Serial)
  {
    Serial.println("Serial ready");
    return true;
  }
  else return false;
}

// Test for string completion character
boolean is_finished(const char in_char)
{
    return  in_char == '\n' ||
            in_char == '\0' ||
            in_char == ';'  ||
            in_char == ',';    
}

// Read serial for chitchat
void read_serial()
{
  boolean serial_ready = false;
  String input_str = "";

  // Each pass try to complete input from avaiable
  while ( !serial_ready && Serial.available() )
  {
    char in_char = (char)Serial.read();  // get the new byte

    // Intake
    // if the incoming character to finish, add a ';' and set flags so the main loop can do something about it:
    if ( is_finished(in_char) )
    {
        if ( input_str.length() ) input_str.concat(';');
        serial_ready = true;
        break;
    }

    else if ( in_char == '\r' )
    {
        Serial.println("\n");  // scroll user terminal
    }

    else if ( in_char == '\b' && input_str.length() )
    {
        Serial.print("\b \b");  // scroll user terminal
        input_str.remove(input_str.length() -1 );  // backspace
    }

    else
    {
        input_str += in_char;  // process new valid character
    }

  }

  // Pass info to serial_str
  if ( serial_ready )
  {
    serial_str += input_str.c_str();
    cp.inp_token = true;
    read_serial_add_verify(&cp.inp_str, serial_str);
    serial_str = "";
    serial_ready = false;
    cp.inp_token = false;
  }
}

// Check for heap fragmentation during String += operation
// Use pointer to be sure not to miss the final assignment effect
void read_serial_add_verify(String *src, const String addend)
{
  int src_len = src->length();
  *src += addend;
  if ( src->length() != (src_len + addend.length()) )
  {
    Serial.print("\n\n\n\n**FRAG**\n\n\n\n");
  }
}

// Request plot
void request_plot(const uint8_t plot_num, const boolean print_hdr, Sensors *Sen, const boolean reset)
{
  static uint8_t last_plot_num = 126;
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
      Sen->plot_eye_tone();  // pp9
      break;
    case 10:
      Sen->print_rapid(print_hdr, Sen->time_eye_s());  // pp10
      break;
    case 11:
      Sen->print_Mahony(print_hdr, Sen->time_eye_s());  // pp11
      break;
    case 12:
      Sen->plot_yaw_reset();  // pp12
      break;
    default:
      break;
  }
  last_plot_num = plot_num;
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

// Set buzzer volume (0-255 for variable PWM dutry cycle based on 'volume')
void setBuzzerVolume(int volume)
{
  analogWrite(tonePin, volume);
}

