#include "motor_control.h"
#include "tcp_server.h"
#include "Magsense.h"
#include "config.h"

volatile long encoder_count = 0;

long target_ticks = 0;
bool motor_running = false;
bool motor_direction = true;
int max_enc = 3000; //4500

uint32_t last_log_us = 0;
uint32_t session_start_ms = 0;
uint32_t state_time = 0;

bool session_active = false;

int cycle_num = 0;
int state = 0;

int cycleMax = 3;


#define IDLE 0
#define PRE 1
#define UP 2
#define HOLD 3
#define DOWN 4
#define REST 5
#define POST 6

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

        session_active = true;
        cycle_num = 0;
        state = PRE;
        state_time = millis();
        session_start_ms = millis();

        client.println("SESSION STARTED");
    }

    if (!session_active) return;

    uint32_t now = millis();

    switch (state) {

        case PRE:
            if (now - state_time >= 5000) {
                cycle_num = 1;
                moveTo(max_enc, true);
                state = UP;
            }
            break;

        case HOLD:
            if (now - state_time >= 5000) {
                moveTo(0, false);
                state = DOWN;
            }
            break;

        case REST:
            if (now - state_time >= 5000) {

                cycle_num++;

                if (cycle_num <= cycleMax) {
                    moveTo(max_enc, true);
                    state = UP;
                } else {
                    state = POST;
                    state_time = now;
                }
            }
            break;

        case POST:
            if (now - state_time >= 5000) {
                session_active = false;
                state = IDLE;
                client.println("SESSION DONE");
            }
            break;
    }
}

void checkMotorStop() {

    if (!motor_running) return;

    double mag_vars[3] = {0};
    readMagsense(mag_vars);
    float x_gauss = mag_vars[0];
    bool mag_triggered = (x_gauss <= MAG_THRESHOLD);

    if(motor_direction){
        if (encoder_count >= target_ticks || mag_triggered) {

            stopMotor();
            motor_running = false;

            state = HOLD;
            state_time = millis();
            if (mag_triggered) {
                client.println("STOP_CAUSE: MAGNET");
            } else {
                client.println("STOP_CAUSE: ENCODER");
            }
        }
    }

    else if (!motor_direction && encoder_count <= target_ticks) {

        stopMotor();
        motor_running = false;

        encoder_count = 0;

        state = REST;
        state_time = millis();
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


    int test_id = cycle_num;
    if (state == PRE || state == POST)
        test_id = 0;

    char msg[128];

    snprintf(
        msg,
        sizeof(msg),
        "DATA,%lu,%ld,%d,%d,%.3f,%.3f,%.3f,%d\n",
        t_ms,
        encoder_count,
        adc_index,
        adc_thumb,
        vars[0],
        vars[1],
        vars[2],
        test_id
    );

    client.write((uint8_t*)msg, strlen(msg));
}