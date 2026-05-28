#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <string>
#include <Arduino.h>

// -------------------- Hardware Pins --------------------
constexpr int AIN1_PIN = 19; // Direction
constexpr int AIN2_PIN = 18; // PWM Speed
constexpr int ENC_A_PIN = 34;
constexpr int ENC_B_PIN = 35;
constexpr int LED_PIN = 2;   // Built-in LED (adjust if your board uses a different pin)

// -------------------- Motor & Encoder Specs --------------------
constexpr float MOTOR_CPR = 12.0f;
constexpr float GEAR_RATIO = 248.98f;
constexpr float OUTPUT_CPR = (MOTOR_CPR * GEAR_RATIO) / 2;
constexpr float COUNTS_TO_RAD = (2.0f * PI) / OUTPUT_CPR;

// -------------------- PWM --------------------
constexpr int PWM_CHANNEL_MOTOR = 0;
constexpr int PWM_CHANNEL_LED = 1;
constexpr int PWM_FREQ = 20000;
constexpr int PWM_BITS = 8;

// -------------------- Logging & State --------------------
static const uint32_t LOG_HZ = 100;
static const uint32_t LOG_PERIOD_US = 1000000UL / LOG_HZ;

// Track motor encoder ticks
extern volatile long encoder_count;
extern long target_ticks;
extern bool motor_running;
extern bool motor_direction;
extern long tick_boundary;

// Initilize motors
void initMotor();

// Actuate motors
void setMotorTarget(int pwmValue, bool dir, long ticks);

// Stop motors
void setMotorStop();

// Check if motor have reached target position and terminate 
void checkMotorStop();

// Move motor on command
void motorRoutine(String cmd);

void protocolRoutine();
void stopMotor();
void moveTo(long target, bool dir);


void logData();

#endif