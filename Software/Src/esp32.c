#include <stdint.h>
#include <stdio.h>
#include <ps5Controller.h>
#include <SPI.h>
#include <RF24.h>

#define CE_PIN  4
#define CSN_PIN 5

RF24 radio(CE_PIN, CSN_PIN);

// Must match STM32 RX address
const uint8_t address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

typedef struct {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} rc_packet;

rc_packet packet;
unsigned long lastTimeStamp = 0;

void sendPacket()
{
    bool sent = radio.write(&packet, sizeof(rc_packet));

    if (sent) {
        Serial.printf("sent OK T=%.2f R=%.2f P=%.2f Y=%.2f\r\n",
                      packet.throttle,
                      packet.roll,
                      packet.pitch,
                      packet.yaw);
    } else {
        Serial.println("sent FAIL");
    }
}

void notify()
{
    packet.throttle = -(ps5.lx / 128.0f);
    packet.yaw      =  (ps5.ry / 128.0f);
    packet.pitch    = -(ps5.rx / 128.0f);
    packet.roll     =  (ps5.ly / 128.0f);
}
void onConnect()
{
    Serial.println("Connected!");
}

void onDisConnect()
{
    Serial.println("Disconnected!");
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    if (!radio.begin()) {
        Serial.println("NRF not responding");
        while (1) {
            delay(1000);
        }
    }

    delay(100);

    radio.setChannel(85);
    radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_LOW);
    radio.setAutoAck(false);
    radio.disableDynamicPayloads();
    radio.setPayloadSize(16);
    radio.setCRCLength(RF24_CRC_16);
    radio.openWritingPipe(address);
    radio.stopListening();

    radio.flush_tx();
    radio.printDetails();
    Serial.print("Payload size ");
    Serial.println(sizeof(rc_packet));

    ps5.attach(notify);
    ps5.attachOnConnect(onConnect);
    ps5.attachOnDisconnect(onDisConnect);
    ps5.begin("24:A6:FA:1C:06:E9");

    Serial.println("Ready.");

    while (ps5.isConnected() == false) {
        Serial.println("PS5 controller not found");
        delay(300);
    }
}

void loop()
{
    if (ps5.isConnected()) {
        packet.throttle = -(ps5.lx / 128.0f);
        packet.yaw      =  (ps5.ry / 128.0f);
        packet.pitch    = -(ps5.rx / 128.0f);
        packet.roll     =  (ps5.ly / 128.0f);

        bool sent = radio.write(&packet, sizeof(rc_packet));

        Serial.printf("sent %s T=%.2f R=%.2f P=%.2f Y=%.2f\r\n",
                      sent ? "OK" : "FAIL",
                      packet.throttle,
                      packet.roll,
                      packet.pitch,
                      packet.yaw);

        delay(20);
    }
}
