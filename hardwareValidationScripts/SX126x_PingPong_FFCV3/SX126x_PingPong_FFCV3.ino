/*
3/10/2025
Yahya Farag
Ping-Pong Example for SX1262 optimized for SOAR applications (GNSS)
*/
#include <RadioLib.h>

// uncomment the following only on one of the nodes to initiate the pings
#define INITIATING_NODE

//Pin Definitions for Activator

//SX1262 LORA CHIPSET INTERFACE
#define ANT_SW PD10
#define SX_RST PD3
#define SX_BUSY PD4
#define SX_DIO1 PD5
#define SX_SCK PB3
#define SX_MISO PB4
#define SX_MOSI PB5
#define SX_NSS PE0

//Setup Radio:
SPIClass SPI_6(SX_MOSI,SX_MISO,SX_SCK);

SX1262 radio = new Module(SX_NSS, SX_DIO1, SX_RST, SX_BUSY,SPI_6);

// save transmission states between loops
int transmissionState = RADIOLIB_ERR_NONE;

// flag to indicate transmission or reception state
bool transmitFlag = false;

// flag to indicate that a packet was sent or received
volatile bool operationDone = false;

void setFlag(void) {
  // we sent or received a packet, set the flag
  operationDone = true;
}

void setup() {
  pinMode(ANT_SW,OUTPUT); digitalWrite(ANT_SW,LOW);
  delay(3000);

  Serial.begin(9600);

  // initialize SX1262 with default settings
  Serial.print(F("[SX1262] Initializing ... "));

  //set freq to 915, bandwidth 125khz, spreading factor SF8, Coding Rate 4/5, private header 0x12, +22 dBm, no TXO, NO LDO (false)
  int state = radio.begin(915.0,125,8,5,RADIOLIB_SX126X_SYNC_WORD_PRIVATE,22,8,0,false);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  // when new packet is received
  radio.setDio1Action(setFlag);


  #if defined(INITIATING_NODE)
    // send the first packet on this node
    Serial.print(F("[SX1262] Sending first packet ... "));
    transmissionState = radio.startTransmit("Hello World!");
    transmitFlag = true;
  #else
    // start listening for LoRa packets on this node
    Serial.print(F("[SX1262] Starting to listen ... "));
    state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("success!"));
    } else {
      Serial.print(F("failed, code "));
      Serial.println(state);
      while (true) { delay(10); }
    }
  #endif
}

void loop() {
  // check if the previous operation finished
  if(operationDone) {
    // reset flag
    operationDone = false;

    if(transmitFlag) {
      // the previous operation was transmission, listen for response
      // print the result
      if (transmissionState == RADIOLIB_ERR_NONE) {
        // packet was successfully sent
        Serial.println(F("transmission finished!"));

      } else {
        Serial.print(F("failed, code "));
        Serial.println(transmissionState);

      }

      // listen for response
      radio.startReceive();
      transmitFlag = false;

    } else {
      // the previous operation was reception
      // print data and send another packet
      String str;
      int state = radio.readData(str);

      if (state == RADIOLIB_ERR_NONE) {
        // packet was successfully received
        Serial.println(F("[SX1262] Received packet!"));

        // print data of the packet
        Serial.print(F("[SX1262] Data:\t\t"));
        Serial.println(str);

        // print RSSI (Received Signal Strength Indicator)
        Serial.print(F("[SX1262] RSSI:\t\t"));
        Serial.print(radio.getRSSI());
        Serial.println(F(" dBm"));

        // print SNR (Signal-to-Noise Ratio)
        Serial.print(F("[SX1262] SNR:\t\t"));
        Serial.print(radio.getSNR());
        Serial.println(F(" dB"));

      }

      // wait a second before transmitting again
      delay(1000);

      // send another one
      Serial.print(F("[SX1262] Sending another packet ... "));
      transmissionState = radio.startTransmit("Hello World!");
      transmitFlag = true;
    }
  
  }
}
