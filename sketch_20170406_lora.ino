// Cayenne LPP Byte definition
// https://github.com/myDevicesIoT/cayenne-docs/blob/master/docs/LORA.md

// Cayenne LPP Payload buildup
// https://hansboksem.wordpress.com/2017/03/06/sending-sensor-data-through-the-things-network-to-cayenne/

// Lora TTN via LMIC and bmp280 code
// https://github.com/galagaking/ttn_nodeworkshop/blob/master/ttn_bmp280_abp.ino
//// https://github.com/rocketscream/Low-Power
//// https://github.com/matthijskooijman/arduino-lmic 

/*******************************************************************************
// https://github.com/galagaking/ttn_nodeworkshop/blob/master/ttn_bmp280_abp.ino
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example will send Temperature and Air Pressure
 * using frequency and encryption settings matching those of
 * the The Things Network. Application will 'sleep' 7x8 seconds (56 seconds)
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "LowPower.h"
#include <SoftwareSerial.h>

#include "i2c.h"
#include "i2c_BMP280.h"

BMP280 bmp280;

#include <Arduino.h>

int sleepcycles = 7;  // every sleepcycle will last 8 secs, total sleeptime will be sleepcycles * 8 sec
bool joined = false;
bool sleeping = false;
#define LedPin 2     // pin 13 LED is not used, because it is connected to the SPI port

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t DEVEUI[8] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x12 };
static const u1_t APPEUI[8] = { 0xCE, 0x42, 0x00, 0xF0, 0x7E, 0xD5, 0xB3, 0x70 };

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t APPKEY[16] = { 0xA9, 0x7D, 0x21, 0xF7, 0xC9, 0x49, 0x71, 0x61, 0x50, 0x9E, 0xDC, 0xCE, 0xD9, 0xCF, 0x63, 0x32 };

void os_getArtEui (u1_t* buf) {
  memcpy(buf, APPEUI, 8);
}

// provide DEVEUI (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
  memcpy(buf, DEVEUI, 8);
}

// provide APPKEY key (16 bytes)
void os_getDevKey (u1_t* buf) {
  memcpy(buf, APPKEY, 16);
}

static osjob_t sendjob;
static osjob_t initjob;

// Pin mapping is hardware specific.
// Pin mapping Doug Larue PCB
const lmic_pinmap lmic_pins = {
.nss = 10,
.rxtx = 0, //LMIC_UNUSED_PIN,
.rst = 0,
.dio = {4, 5, 7},
};

void onEvent (ev_t ev) {
  int i,j;
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      digitalWrite(LedPin,LOW);
      // after Joining a job with the values will be sent.
      joined = true;
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      // Re-init
      os_setCallback(&initjob, initfunc);
      break;
    case EV_TXCOMPLETE:
      sleeping = true;
        if (LMIC.dataLen) {
        // data received in rx slot after tx
        // if any data received, a LED will blink
        // this number of times, with a maximum of 10
        Serial.print(F("Data Received: "));
        Serial.println(LMIC.frame[LMIC.dataBeg],HEX);
        i=(LMIC.frame[LMIC.dataBeg]);
        // i (0..255) can be used as data for any other application
        // like controlling a relay, showing a display message etc.
        if (i>10){
          i=10;     // maximum number of BLINKs
        }
        for(j=0;j<i;j++){
          digitalWrite(LedPin,HIGH);
          delay(200);
          digitalWrite(LedPin,LOW);
          delay(400);
        }
      }
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      delay(50);  // delay to complete Serial Output before Sleeping

      // Schedule next transmission
      // next transmission will take place after next wake-up cycle in main loop
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void do_send(osjob_t* j) {
  float temperature,pascal;
  
  //Read sensor vallues from BMP Board
  bmp280.awaitMeasurement();
  bmp280.getTemperature(temperature);
  bmp280.getPressure(pascal);
  bmp280.triggerMeasurement();
  
  //Print vallue from the sensor BMP Board
  pascal=pascal/100;
  pascal = (int) pascal * 10;
  Serial.print(" Pressure: ");
  Serial.print(pascal);
  Serial.print(" Pa; T: ");
  
  temperature = (int) temperature * 10;
  Serial.print(temperature);
  Serial.println(" C");

  // Example Cayenne LPP Payload data is 03 67 01 10 05 67 00 FF  
  byte buffer[12];
  uint8_t cursor = 0;
  #define LPP_TEMPERATURE         0x67     // 103 - 2 bytes, 0.1°C signed
  #define LPP_BAROMETRIC_PRESSURE 0x73     // 115 - 2 bytes 0.1 hPa Unsigned  
  
  buffer[cursor++] = 0x03;  // data channel
  buffer[cursor++] = LPP_TEMPERATURE; 
  buffer[cursor++] = 0x01; // TEMPERATURE 27.2°C = 01 10
  buffer[cursor++] = 0x10; // TEMPERATURE 27.2°C = 01 10

  buffer[cursor++] = 0x04; // data channel
  buffer[cursor++] = LPP_BAROMETRIC_PRESSURE; 
  buffer[cursor++] = 0x01; // TEMPERATURE 27.2°C = 01 10
  buffer[cursor++] = 0xF4; // TEMPERATURE 27.2°C = 01 10
  
  buffer[cursor++] = 0x05; // data channel
  buffer[cursor++] = LPP_TEMPERATURE; 
  buffer[cursor++] = 0x01; // TEMPERATURE 27.2°C = 01 10
  buffer[cursor++] = 0xF4; // TEMPERATURE 27.2°C = 01 10

/* // example code of Cayenne LPP
  #define LPP_TEMPERATURE         103     //  0x67 - 103 - 2 bytes, 0.1°C signed
  #define LPP_BAROMETRIC_PRESSURE 115     // 0x73 - 115 - 2 bytes 0.1 hPa Unsigned 

  int16_t val;
  uint8_t channel
  byte buffer[8];
  uint8_t cursor;

  buffer = (uint8_t*) malloc(size);
  cursor = 0;
  
  val = pascal;
  channel = 0x04;
  buffer[cursor++] = channel; 
  buffer[cursor++] = LPP_BAROMETRIC_PRESSURE; 
  buffer[cursor++] = val >> 8; 
  buffer[cursor++] = val; 

  val = temperature;
  channel = 0x05;
  buffer[cursor++] = channel; 
  buffer[cursor++] = LPP_TEMPERATURE; 
  buffer[cursor++] = val >> 8; 
  buffer[cursor++] = val; 
*/
   
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, buffer, sizeof(buffer)/sizeof(buffer[0]), 0);
    
    //Serial output of message
    Serial.println(F("Sending: "));
    for(byte b=0; b<(sizeof(buffer)/sizeof(buffer[0])); b++){
      Serial.print(buffer[b]);
   }
   Serial.println(" Done, on to the next measurement");
  }
  
}//do_send()

// initial job
static void initfunc (osjob_t* j) {
    // reset MAC state
    LMIC_reset();
    // start joining
    LMIC_startJoining();
    // init done - onEvent() callback will be invoked...
}

void setup() {
    Serial.begin(115200);
    delay(250);
    Serial.println(F("Starting"));
    Serial.print("Probe BMP280: ");
    if (bmp280.initialize()) Serial.println("Sensor found");
    else{
      Serial.println("Sensor missing");
      while (1) {}
    }

    // onetime-measure:
    bmp280.setEnabled(0);
    bmp280.triggerMeasurement();
    
    // if LED is connected to pin 10, it has to be defined before any SPI initialization else
    // it will be used as SS (Slave Select) and controlled by the SPI module
    pinMode(LedPin, OUTPUT);
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    os_setCallback(&initjob, initfunc);
    LMIC_reset();
}// setup()

unsigned long time;
void loop(){
    // start OTAA JOIN
    if (joined==false) {
      os_runloop_once();
    }
    else {
      // Sent sensor values
      do_send(&sendjob);    
      while(sleeping == false) {
        os_runloop_once();
      }
      sleeping = false;
      for (int i=0;i<sleepcycles;i++){
          LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);    //sleep 8 seconds
      }
    }
    digitalWrite(LedPin,((millis()/100) % 2) && (joined==false)); // only blinking when joining and not sleeping
} // loop()
