#include <Arduino.h>

// Encoder Pins
#define ENCODER_LEFT_A 2
#define ENCODER_LEFT_B 3
#define ENCODER_RIGHT_A 18
#define ENCODER_RIGHT_B 19

// Motor Driver Pins
#define ENA 10
#define IN1_A 9
#define IN2_A 8
#define ENB 5
#define IN1_B 7
#define IN2_B 6

// volatile long pulses_left = 0;
// volatile long pulses_right = 0;

// unsigned long previousMillis = 0;
// const long interval = 100;  // Sampling interval in ms
// long previous_pulses_left = 0;
// long previous_pulses_right = 0;

// // PID parameters
// float Kp = 0.0589;
// float Ki = 10.7;

// // Desired speeds
// float desired_speed_left = 100.0;  // Adjust as needed
// float desired_speed_right = 100.0; // Adjust as needed

// // Error and integral terms for PID
// float error_left = 0, error_right = 0;
// float integral_left = 0, integral_right = 0;

// void countPulsesLeft() {
//   pulses_left++;
// }

// void countPulsesRight() {
//   pulses_right++;
// }

// void setup() {
//   Serial.begin(9600);
//   pinMode(ENCODER_LEFT_A, INPUT);
//   pinMode(ENCODER_LEFT_B, INPUT);
//   pinMode(ENCODER_RIGHT_A, INPUT);
//   pinMode(ENCODER_RIGHT_B, INPUT);

//   attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), countPulsesLeft, RISING);
//   attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), countPulsesRight, RISING);

//   pinMode(ENA, OUTPUT);
//   pinMode(IN1_A, OUTPUT);
//   pinMode(IN2_A, OUTPUT);
//   pinMode(ENB, OUTPUT);
//   pinMode(IN1_B, OUTPUT);
//   pinMode(IN2_B, OUTPUT);

//   // Set motor direction
//   digitalWrite(IN1_A, LOW);
//   digitalWrite(IN2_A, HIGH);
//   digitalWrite(IN1_B, LOW);
//   digitalWrite(IN2_B, HIGH);
// }

// void loop() {
//   unsigned long currentMillis = millis();

//   if (currentMillis - previousMillis >= interval) {
//     previousMillis = currentMillis;

//     // Measure motor speeds
//     long current_pulses_left = pulses_left;
//     long current_pulses_right = pulses_right;
//     long pulse_diff_left = current_pulses_left - previous_pulses_left;
//     long pulse_diff_right = current_pulses_right - previous_pulses_right;
//     float speed_left = (pulse_diff_left * 1000.0) / interval;
//     float speed_right = (pulse_diff_right * 1000.0) / interval;

//     // Calculate errors
//     error_left = desired_speed_left - speed_left;
//     error_right = desired_speed_right - speed_right;

//     // Update integral terms
//     integral_left += error_left * (interval / 1000.0);
//     integral_right += error_right * (interval / 1000.0);

//     // Prevent integral windup
//     integral_left = constrain(integral_left, -100, 100);
//     integral_right = constrain(integral_right, -100, 100);

//     // Calculate control outputs (PWM signals)
//     float control_output_left = Kp * error_left + Ki * integral_left;
//     float control_output_right = Kp * error_right + Ki * integral_right;

//     // Constrain the PWM values to valid range (0-255)
//     int pwm_left = constrain((int)control_output_left, 0, 255);
//     int pwm_right = constrain((int)control_output_right, 0, 255);

//     // Apply PWM to motors
//     analogWrite(ENA, pwm_left);
//     analogWrite(ENB, pwm_right);

//     // Debugging output
//     Serial.print("Speed Left: ");
//     Serial.print(speed_left);
//     Serial.print(" | Speed Right: ");
//     Serial.print(speed_right);
//     Serial.print(" | PWM Left: ");
//     Serial.print(pwm_left);
//     Serial.print(" | PWM Right: ");
//     Serial.println(pwm_right);

//     // Update previous pulse counts for the next interval
//     previous_pulses_left = current_pulses_left;
//     previous_pulses_right = current_pulses_right;
//   }
// }
void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1_A, OUTPUT);
  pinMode(IN2_A, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1_B, OUTPUT);
  pinMode(IN2_B, OUTPUT);

  // Set motor directions
  digitalWrite(IN1_A, LOW);
  digitalWrite(IN2_A, HIGH);
  digitalWrite(IN1_B, LOW);
  digitalWrite(IN2_B, HIGH);

  // Apply full speed
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
}

void loop() {
  // Nothing here
}
