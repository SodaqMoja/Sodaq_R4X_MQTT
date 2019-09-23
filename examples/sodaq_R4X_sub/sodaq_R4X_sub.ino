/*!
 * \file sodaq_R4X_pub.ino
 *
 * Copyright (c) 2019 Gabriel Notman.  All rights reserved.
 *
 * This file is part of Sodaq_MQTT.
 *
 * Sodaq_MQTT is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or(at your option) any later version.
 *
 * Sodaq_MQTT is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with Sodaq_MQTT.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/*
 * This example shows how to do a MQTT PUBLISH via a GPRSbee connection.
 * It does the PUBLISH ten times and then it quits. The PUBLISH goes to
 * the test MQTT server at test.moquitto.org.
 *
 * To see the message you can subscribe, for example from the command
 * line with:
 *    mosquitto_sub -h test.mosquitto.org -t "SODAQ/demo/#"
 *
 * To build this example you need two Arduino libraries: Sodaq_MQTT and Sodaq_R4X.
 */

#include <Arduino.h>
#include <Sodaq_R4X.h>
#include <Sodaq_MQTT.h>
#include <Sodaq_R4X_MQTT.h>
#include <Sodaq_wdt.h>

#define MQTT_BROKER     "test.mosquitto.org"
#define MQTT_PORT       1883
#define MQTT_KEEP_ALIVE (5 * 60)

#define APN             "aerea.m2m.com"
#define URAT            SODAQ_R4X_LTEM_URAT
#define MNOPROF         MNOProfiles::SIM_ICCID
#define OPERATOR        AUTOMATIC_OPERATOR
#define M1_BAND_MASK    BAND_MASK_UNCHANGED
#define NB1_BAND_MASK   BAND_MASK_UNCHANGED

#if defined(ARDUINO_SODAQ_SARA) || defined(ARDUINO_SODAQ_SFF)
    #define MODEM_STREAM Serial1
    #define DEBUG_STREAM SerialUSB
#else
    #error "Please seletct SARA AFF or SFF"
#endif

static void handlePublish(const char *topic, const uint8_t *msg, size_t msg_length);
static void handlePacket(uint8_t *pckt, size_t len);

static bool is_subscribed;
static uint32_t prevPing = millis();

Sodaq_R4X r4x;
Sodaq_SARA_R4XX_OnOff saraR4xxOnOff;
Sodaq_R4X_MQTT r4x_mqtt;

bool modemConnect()
{
    if (r4x.isConnected()) {
        return true;
    }
    else {
        return r4x.connect(APN, URAT, MNOPROF, OPERATOR, M1_BAND_MASK, NB1_BAND_MASK);
    }
}

void setup()
{
    while ((!SerialUSB) || (millis() < 5000)) {};
    
    // We'll use this to print some messages
    DEBUG_STREAM.begin(57600);
    DEBUG_STREAM.println("test_mqtt");

    // Set the MQTT server hostname, and the port number
    mqtt.setServer(MQTT_BROKER, MQTT_PORT);

    // OPTIONAL. Set the user name and password
    //mqtt.setAuth("Hugh", "myPass");

    // Set the MQTT client ID
    mqtt.setClientId("sodaq_pub_12345");

    // Set the MQTT keep alive
    mqtt.setKeepAlive(MQTT_KEEP_ALIVE);

    // Set handlers
    mqtt.setPublishHandler(handlePublish);
    mqtt.setPacketHandler(handlePacket);

    /*
     * The transport layer is a Sodaq_R4X
     */
    MODEM_STREAM.begin(r4x.getDefaultBaudrate());
    r4x.init(&saraR4xxOnOff, MODEM_STREAM);
    r4x.setDiag(DEBUG_STREAM);

    // Inform our mqtt instance that we use r4x as the transport
    r4x_mqtt.setR4Xinstance(&r4x, modemConnect);
    mqtt.setTransport(&r4x_mqtt);

    r4x.on();
    r4x.execCommand("AT+CPSMS=0");
    r4x.execCommand("AT+CEDRXS=0");    
}

void loop()
{
    // Handle incoming SUBSCRIBE?

    if (mqtt.loop()) {
        // An incoming packet was handled
        // Don't do anything else in this anymore.
        return;
    }

    if (!mqtt.isConnected()) {
        is_subscribed = false;
    }

    if (!is_subscribed) {
        if (!mqtt.subscribe("SODAQ/demo/test")) {
            DEBUG_STREAM.println("subscribe failed");
        } else {
            is_subscribed = true;
        }
    }

    uint32_t now = millis();
    int32_t since = now - prevPing;
    if (since > (1000L * MQTT_KEEP_ALIVE)) {
        // Send PINGREQ
        if (!mqtt.ping()) {
            DEBUG_STREAM.println("ping failed");
        } 
        prevPing = now;
    }

    delay(1000);
}

/*
 * Handler for incoming PUBLISH packets
 */
static void handlePublish(const char *topic, const uint8_t *msg, size_t msg_length)
{
    DEBUG_STREAM.println(String("Incoming PUBLISH, topic=") + topic);
    // Hopefully the string is terminated with a NUL byte
    DEBUG_STREAM.println(String("   msg=") + (const char *)msg);
}

/*
 * Generic MQTT packet handler
 *
 * This handler is called when mqtt.loop sees a packet
 * and it is not handled otherwise.
 */
static void handlePacket(uint8_t *pckt, size_t len)
{
}
