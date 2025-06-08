#include <Arduino.h>

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
bool motor1_forward = true;  // Motor 1 direction
const int pwm_value = 255;   // 100% duty cycle (0-255)
volatile unsigned long enc1_count = 0;  // Rising edges on E1+
unsigned long last_time = 0;
const unsigned long interval = 5000;  // 5 seconds

// Interrupt handler for E1+ rising edges
void encoder1_isr() {
  enc1_count++;
}

void setup() {
  // Initialize USB serial
  Serial.begin(115200);
  delay(2000);
  Serial.println("Setup started");

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
  pinMode(ENC2_POS, INPUT);
  pinMode(ENC2_NEG, INPUT);

  // Attach interrupt for E1+ rising edge
  attachInterrupt(digitalPinToInterrupt(ENC1_POS), encoder1_isr, RISING);

  // Initialize ADC resolution (10-bit default, 0-1023)
  analogReadResolution(10);

  // Set PWM frequency to 100 Hz
  Serial.println("PWM frequency set to 100 Hz on pin 4");

  // Debug pin states
  Serial.println("Motor 1 pins: PWM=" + String(PWM_M1) + ", F2=" + String(DIR_F1) + ", B2=" + String(DIR_B1));
  Serial.println("Sense pins: M1=A13, M2=A12");
  Serial.println("Encoder pins: E1+=" + String(ENC1_POS) + ", E1-=" + String(ENC1_NEG));
  Serial.println("Setup complete");
}

void loop() {
  unsigned long current_time = millis();

  // Every 5 seconds, toggle direction and read measurements
  if (current_time - last_time >= interval) {
    // Stop motor briefly before changing direction
    analogWrite(PWM_M1, 0);
    digitalWrite(DIR_F1, LOW);
    digitalWrite(DIR_B1, LOW);
    delay(100);  // Brief pause to prevent H-bridge stress
    Serial.println("Motor 1: Stopped briefly");

    // Toggle Motor 1 direction
    motor1_forward = !motor1_forward;

    if (motor1_forward) {
      digitalWrite(DIR_F1, HIGH);
      digitalWrite(DIR_B1, LOW);
      analogWrite(PWM_M1, pwm_value);  // 100% speed
      Serial.println("Motor 1: Forward, PWM=" + String(pwm_value));
    } else {
      digitalWrite(DIR_F1, LOW);
      digitalWrite(DIR_B1, HIGH);
      analogWrite(PWM_M1, pwm_value);  // 100% speed
      Serial.println("Motor 1: Backward, PWM=" + String(pwm_value));
    }

    // Read current sense (A13, A12)
    int sense_m1 = analogRead(SENSE_M1);
    int sense_m2 = analogRead(SENSE_M2);
    float current_m1 = (sense_m1 * 3.3 / 1023.0) / 10.0;  // V_sense / R_sense
    float current_m2 = (sense_m2 * 3.3 / 1023.0) / 10.0;

    // Read encoder states (A14-A17)
    int enc1_pos = analogRead(ENC1_POS);  // E1+
    int enc1_neg = analogRead(ENC1_NEG);  // E1-
    int enc2_pos = analogRead(ENC2_POS);  // E2+
    int enc2_neg = analogRead(ENC2_NEG);  // E2-

    // Calculate speed (edges per 5 seconds)
    float speed_rpm = (enc1_count / 2.0) * (60.0 / 5.0);  // Assume 2 pulses per revolution
    enc1_count = 0;  // Reset count

    // Determine direction from encoder phase
    String direction = (enc1_pos > 512 && enc1_neg < 512) ? "Forward" : 
                      (enc1_neg > 512 && enc1_pos < 512) ? "Backward" : "Unknown";

    // Output measurements
    Serial.print("Current M1: ");
    Serial.print(current_m1, 3);
    Serial.print(" A, Current M2: ");
    Serial.print(current_m2, 3);
    Serial.println(" A");
    Serial.print("Encoder M1: E1+=");
    Serial.print(enc1_pos);
    Serial.print(", E1-=");
    Serial.print(enc1_neg);
    Serial.print(", Speed: ");
    Serial.print(speed_rpm, 1);
    Serial.print(" RPM, Direction: ");
    Serial.println(direction);
    Serial.print("Encoder M2: E2+=");
    Serial.print(enc2_pos);
    Serial.print(", E2-=");
    Serial.println(enc2_neg);

    last_time = current_time;
  }
}