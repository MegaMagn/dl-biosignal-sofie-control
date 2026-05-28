#include <Arduino.h>
#include "motor_control.h"
#include "tcp_server.h"
#include "Magsense.h"
#include "config.h"

volatile long encoder_count = 0;

long target_ticks = 0;
bool motor_running = false;
bool motor_direction = true;
int max_enc = 3500; //2500// // 4500

uint32_t last_log_us = 0;
uint32_t session_start_ms = 0;

bool session_active = false;
int cycle_num = 0;

enum MotionState {
    POS_RELEASED,
    POS_CONTRACTING,
    POS_CONTRACTED,
    POS_RELEASING
};

MotionState motion_state = POS_RELEASED;


void IRAM_ATTR encoderISR() {
    if (motor_direction)
        encoder_count++;
    else
        encoder_count--;
}


void initMotor() {

    pinMode(AIN1_PIN, OUTPUT);
    pinMode(ENC_A_PIN, INPUT);
    pinMode(ENC_B_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, CHANGE);

    ledcAttachChannel(AIN2_PIN, PWM_FREQ, PWM_BITS, PWM_CHANNEL_MOTOR);
    ledcWrite(AIN2_PIN, 0);
}


void stopMotor() {
    ledcWrite(AIN2_PIN, 0);
}


void moveTo(long target, bool dir) {

    motor_direction = dir;
    target_ticks = target;
    motor_running = true;

    digitalWrite(AIN1_PIN, dir ? HIGH : LOW);
    ledcWrite(AIN2_PIN, 255);
}


void protocolRoutine() {

    String cmd = getCommand();

    if (cmd == "startsession" && !session_active) {

        // Only allow a new session when fully released and idle
        if (!motor_running && motion_state == POS_RELEASED) {

            session_active = true;
            cycle_num = 0;
            session_start_ms = millis();
            last_log_us = 0;

            noInterrupts();
            encoder_count = 0;
            interrupts();

            client.println("SESSION STARTED");
        } else {
            client.println("IGNORED: STARTSESSION_NOT_RELEASED");
        }
    }

    else if (cmd == "contract" && session_active) {

        // Only allow contraction from the released position
        if (!motor_running && motion_state == POS_RELEASED) {

            cycle_num++;

            moveTo(max_enc, true);
            motion_state = POS_CONTRACTING;

            client.print("MOTION_STARTED,contract,");
            client.println(cycle_num);
        } else {
            client.println("IGNORED: CONTRACT");
        }
    }

    else if (cmd == "release" && session_active) {

        // Only allow release from the contracted position
        if (!motor_running && motion_state == POS_CONTRACTED) {

            moveTo(0, false);
            motion_state = POS_RELEASING;

            client.print("MOTION_STARTED,release,");
            client.println(cycle_num);
        } else {
            client.println("IGNORED: RELEASE");
        }
    }

    else if (cmd == "endsession" && session_active) {

        // Only allow session end when fully released and idle
        if (!motor_running && motion_state == POS_RELEASED) {

            session_active = false;
            client.println("SESSION DONE");
        } else {
            client.println("IGNORED: ENDSESSION_NOT_RELEASED");
        }
    }
}


void checkMotorStop() {

    if (!motor_running) return;

    long current_count;

    noInterrupts();
    current_count = encoder_count;
    interrupts();

    if (motion_state == POS_CONTRACTING) {

        double mag_vars[3] = {0};
        readMagsense(mag_vars);

        float x_gauss = mag_vars[0] * 8.0;
        bool mag_triggered = (x_gauss <= MAG_THRESHOLD);

        if (current_count >= target_ticks || mag_triggered) {

            stopMotor();
            motor_running = false;
            motion_state = POS_CONTRACTED;

            if (mag_triggered) {
                client.println("STOP_CAUSE: MAGNET");
            } else {
                client.println("STOP_CAUSE: ENCODER");
            }

            client.print("MOTION_DONE,contract,");
            client.println(cycle_num);
        }
    }

    else if (motion_state == POS_RELEASING) {

        if (current_count <= target_ticks) {

            stopMotor();
            motor_running = false;

            noInterrupts();
            encoder_count = 0;
            interrupts();

            motion_state = POS_RELEASED;

            client.print("MOTION_DONE,release,");
            client.println(cycle_num);
        }
    }
}


void logData() {

    if (!session_active) return;

    uint32_t now = micros();

    if (now - last_log_us < LOG_PERIOD_US)
        return;

    last_log_us = now;

    if (!client || !client.connected())
        return;

    uint32_t t_ms = millis() - session_start_ms;

    int adc_index = analogRead(32);
    int adc_thumb = analogRead(33);

    double vars[3] = {0};
    readMagsense(vars);

    long current_count;

    noInterrupts();
    current_count = encoder_count;
    interrupts();

    int test_id = cycle_num;

    char msg[128];

    snprintf(
        msg,
        sizeof(msg),
        "DATA,%lu,%ld,%d,%d,%.3f,%.3f,%.3f,%d\n",
        t_ms,
        current_count,
        adc_index,
        adc_thumb,
        vars[0],
        vars[1],
        vars[2],
        test_id
    );

    client.write((uint8_t*)msg, strlen(msg));
}