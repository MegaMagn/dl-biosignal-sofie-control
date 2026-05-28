#include "wifi_manager.h"
#include "tcp_server.h"
#include "motor_control.h"
#include "Magsense.h"

void setup() {
    Serial.begin(115200);
    connectWiFi();
    initTCP();
    initMagsense();
    initMotor();
}

void loop() {
    readCommands();
    protocolRoutine();
    checkMotorStop();
    logData();
}