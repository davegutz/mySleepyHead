#pragma once
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
};

// Set buzzer volume (0-255 for variable PWM dutry cycle based on 'volume')
void setBuzzerVolume(int volume)
{
  analogWrite(buzzerPin, volume);
}
