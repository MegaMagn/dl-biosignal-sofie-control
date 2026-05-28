#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include <WiFi.h>
#include <string>

// Global TCP objects (shared across files)
extern WiFiServer server;
extern WiFiClient client;

// Initialize server
bool initTCP();

// Handle communication (call inside loop)
String listenTCP();

void readCommands();
String getCommand();

#endif