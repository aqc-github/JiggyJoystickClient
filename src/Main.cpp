#include <Arduino.h>
#include <math.h> // For sin() and M_PI

#define GEARING     50
#define ENCODERMULT 12

// Motor 1 pins (L298N)
#define PWM_M1 2   // PWM2, enable pin for Motor 1
#define DIR_F1 20  // F2, forward pin (Motor 1)
#define DIR_B1 21  // B2, backward pin (Motor 1)

// Motor 2 pins (defined, unused)
#define PWM_M2 3   // PWM3, enable pin for Motor 2
#define DIR_F2 22  // F1, forward pin (Motor 2)
#define DIR_B2 23  // B1, backward pin (Motor 2)

// Current sense pins
#define SENSE_M1 A13  // Current sense for Motor 1
#define SENSE_M2 A12  // Current sense for Motor 2

// Encoder pins
#define ENC1_POS A14  // E1+, Motor 1 encoder positive
#define ENC1_NEG A15  // E1-, Motor 1 encoder negative
#define ENC2_POS A16  // E2+, Motor 2 encoder positive
#define ENC2_NEG A17  // E2-, Motor 2 encoder negative

// Motor control variables
int pwm_value = 255;   // Initial PWM value (will be updated sinusoidally)
volatile unsigned long enc1_count = 0;  // Rising edges on E1+
unsigned long last_time = 0;           // For 100 ms messaging
volatile uint32_t lastA = 0;           // Last pulse time for RPM
volatile float RPM = 0.0;              // Calculated RPM
const float PWM_PERIOD = 6000.0;      // Period for sinusoidal PWM (ms)

// Interrupt handler for E1+ rising edges (counts pulses and calculates RPM)
void encoder1_isr() {
  enc1_count++;
  digitalWrite(LED_BUILTIN, HIGH);
  uint32_t currA = micros();
  if (lastA < currA) {
    // Did not wrap around
    float rev = currA - lastA;  // us
    rev = 1.0 / rev;            // rev per us
    rev *= 1000000;             // rev per sec
    rev *= 60;                  // rev per min
    rev /= GEARING;             // Account for gear ratio (20)
    rev /= ENCODERMULT;         // Account for multiple ticks per rotation (12)
    RPM = rev;
  }
  lastA = currA;
  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
  // Initialize USB serial
  Serial.begin(115200);
  // Ensure labels are the first serial output
  Serial.println("I_M1(mA),Speed(RPM/100):");
  while (!Serial && millis() < 2000); // Wait up to 2 seconds for Serial Monitor

  // Motor 1 pins
  pinMode(PWM_M1, OUTPUT);
  pinMode(DIR_F1, OUTPUT);
  pinMode(DIR_B1, OUTPUT);

  // Motor 2 pins (unused, set as output for safety)
  pinMode(PWM_M2, OUTPUT);
  pinMode(DIR_F2, OUTPUT);
  pinMode(DIR_B2, OUTPUT);

  // Set Motor 2 off
  analogWrite(PWM_M2, 0);
  digitalWrite(DIR_F2, LOW);
  digitalWrite(DIR_B2, LOW);

  // Encoder pins as input
  pinMode(ENC1_POS, INPUT);
  pinMode(ENC1_NEG, INPUT);
  pinMode(ENC2_POS, INPUT_PULLDOWN); // Avoid noise
  pinMode(ENC2_NEG, INPUT_PULLDOWN); // Avoid noise

  // Attach interrupt for E1+ rising edge
  attachInterrupt(digitalPinToInterrupt(ENC1_POS), encoder1_isr, RISING);

  // Initialize ADC resolution (10-bit, 0-1023)
  analogReadResolution(10);

  // Set PWM frequency to 100 Hz
  analogWriteFrequency(PWM_M1, 100);
}

void loop() {
  unsigned long current_time = millis();

  // Calculate sinusoidal PWM: centered at 75% (191), amplitude 25% (64)
  float t = current_time / 1000.0; // Time in seconds
  pwm_value = 191 + 64 * sin(2.0 * M_PI * t / (PWM_PERIOD / 1000.0));
  if (pwm_value < 0) pwm_value = 0;     // Clamp to 0
  if (pwm_value > 255) pwm_value = 255; // Clamp to 255

  // Set Motor 1 to forward direction
  digitalWrite(DIR_F1, HIGH);
  digitalWrite(DIR_B1, LOW);
  analogWrite(PWM_M1, pwm_value);  // Apply sinusoidal PWM value

  // Reset RPM if no pulses for 1 second (motor stopped or slow)
  if (micros() - lastA > 1000000) {
    RPM = 0.0;
  }

  // Every 100 ms, read and output measurements
  if (current_time - last_time >= 100) {
    // Read current sense (A13, A12)
    int sense_m1 = analogRead(SENSE_M1);
    float current_m1 = (sense_m1 * 3.3 / 1023.0) / 10.0;  // V_sense / R_sense

    // Output values in a single line, separated by commas
    Serial.print(current_m1 * 10, 1); // Current M1 in mA, 1 decimal place
    Serial.print(",");
    Serial.print(RPM / 10, 3); // Scaled RPM, 3 decimal places
    Serial.println(); // New line to complete the data point

    last_time = current_time;
  }
}