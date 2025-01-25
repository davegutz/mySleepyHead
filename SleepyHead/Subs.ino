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

// Buzzer
boolean init_buzzer()
{
  Serial.print("Buzzer starting at pin "); Serial.print(buzzerPin); Serial.print("...");
  buzz.begin();
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
void request_plot(const uint8_t plot_num, Sensors *Sen, const boolean reset)
{
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
      debug = 10;
      Sen->print_rapid(reset, true, Sen->time_eye_s());  // pp10
      break;
    default:
      Serial.println("plot number unknown enter plot number e.g. pp0 (sum), pp1 (acc), pp2 (rot), pp3 (all), pp4 (quiet), pp5 (quiet raw), pp6 (total), pp7 (roll-pitch-yaw), pp8 (head_buzz), pp9 (eye_buzz), pp10 (buzz list)");
      break;
  }
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

// Motor off
boolean turn_off_motor_and_led()
{
  digitalWrite(motorPin, LOW);
  digitalWrite(LED_BUILTIN, LOW);
  return true;
}

// Motor on
boolean turn_on_motor_and_led()
{
  digitalWrite(motorPin, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);
  return true;
}
