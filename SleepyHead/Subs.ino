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

