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
