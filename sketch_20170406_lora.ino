// Cayenne LPP Byte definition
//https://github.com/myDevicesIoT/cayenne-docs/blob/master/docs/LORA.md

// Cayenne LPP Payload buildup
// https://hansboksem.wordpress.com/2017/03/06/sending-sensor-data-through-the-things-network-to-cayenne/

// Lora TTN via LMIC and bmp280 code
// https://github.com/galagaking/ttn_nodeworkshop/blob/master/ttn_bmp280_abp.ino


#define LPP_DIGITAL_INPUT       0       // 1 byte
#define LPP_DIGITAL_OUTPUT      1       // 1 byte
#define LPP_ANALOG_INPUT        2       // 2 bytes, 0.01 signed
#define LPP_ANALOG_OUTPUT       3       // 2 bytes, 0.01 signed
#define LPP_LUMINOSITY          101     // 2 bytes, 1 lux unsigned
#define LPP_PRESENCE            102     // 1 byte, 1
#define LPP_TEMPERATURE         103     // 2 bytes, 0.1°C signed
#define LPP_RELATIVE_HUMIDITY   104     // 1 byte, 0.5% unsigned
#define LPP_ACCELEROMETER       113     // 2 bytes per axis, 0.001G
#define LPP_BAROMETRIC_PRESSURE 115     // 2 bytes 0.1 hPa Unsigned
#define LPP_GYROMETER           134     // 2 bytes per axis, 0.01 °/s
#define LPP_GPS                 136     // 3 byte lon/lat 0.0001 °, 3 bytes alt 0.01m

// Data ID + Data Type + Data Size
#define LPP_DIGITAL_INPUT_SIZE       3
#define LPP_DIGITAL_OUTPUT_SIZE      3
#define LPP_ANALOG_INPUT_SIZE        4
#define LPP_ANALOG_OUTPUT_SIZE       4
#define LPP_LUMINOSITY_SIZE          4
#define LPP_PRESENCE_SIZE            3
#define LPP_TEMPERATURE_SIZE         4
#define LPP_RELATIVE_HUMIDITY_SIZE   3
#define LPP_ACCELEROMETER_SIZE       8
#define LPP_BAROMETRIC_PRESSURE_SIZE 4
#define LPP_GYROMETER_SIZE           8
#define LPP_GPS_SIZE                 11

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
static const u1_t DEVEUI[8] = { };
static const u1_t APPEUI[8] = { };

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t APPKEY[16] = { };

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
  float hum = 0.0;
  uint8_t cursor = 0;
  int16_t help;
  uint16_t t_value, p_value, s_value;
  byte buffer[20];

  //Read sensor vallues from BMP Board
  bmp280.awaitMeasurement();
  bmp280.getTemperature(temperature);
  bmp280.getPressure(pascal);
  bmp280.triggerMeasurement();
  
  //Print vallue from the sensor BMP Board
  pascal=pascal/100;
  Serial.print(" Pressure: ");
  Serial.print(pascal);
  Serial.print(" Pa; T: ");
  Serial.print(temperature);
  Serial.println(" C");

  //create Cayenne LPP payload message
  buffer[cursor++] = 0x03;
  buffer[cursor++] = LPP_TEMPERATURE;
  //help = temp * 10;
  help = temperature;
  buffer[cursor++] = help >> 8;
  buffer[cursor++] = help;
  
  buffer[cursor++] = 0x04;
  buffer[cursor++] = LPP_RELATIVE_HUMIDITY;
  buffer[cursor++] = hum; 
  
  buffer[cursor++] = 0x05;
  buffer[cursor++] = LPP_BAROMETRIC_PRESSURE;
  //help = pres * 10;
  help = pascal;
  buffer[cursor++] = help >> 8;
  buffer[cursor++] = help;
  
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
   // LMIC_setTxData2(1, (uint8_t*) buffer, 2 , 0);
   LMIC_setTxData2(1, buffer, 2 , 0);
   Serial.println(F("Sending: "));
   for(byte b=0; b<20; b++){
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
