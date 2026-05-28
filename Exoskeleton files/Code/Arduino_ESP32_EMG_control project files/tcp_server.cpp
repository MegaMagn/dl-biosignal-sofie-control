#include "tcp_server.h"

WiFiServer server(1234);
WiFiClient client;

String latest_cmd = "";

bool initTCP() {

    server.begin();

    while (true) {

        client = server.available();

        if (client && client.connected()) {

            while (client.connected()) {

                if (client.available()) {

                    String cmd = client.readStringUntil('\n');
                    cmd.trim();

                    if (cmd == "connect") {
                        client.println("connection complete");
                        return true;
                    }
                }
            }
        }
    }
}

void readCommands() {

    if (!client || !client.connected()) return;

    while (client.available()) {

        String cmd = client.readStringUntil('\n');
        cmd.trim();

        latest_cmd = cmd;
    }
}

String getCommand() {

    String temp = latest_cmd;
    latest_cmd = "";
    return temp;
}