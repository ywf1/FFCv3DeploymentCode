// include the library
#include <RadioLib.h>

//SX1262 LORA CHIPSET INTERFACE
#define ANT_SW PD10
#define SX_RST PD3
#define SX_BUSY PD4
#define SX_DIO1 PD5
#define SX_SCK PB3
#define SX_MISO PB4
#define SX_MOSI PB5
#define SX_NSS PE0

//Status LED BLUE
#define LED_B PA0

//Setup Radio:
SPIClass SPI_6(SX_MOSI,SX_MISO,SX_SCK);

SX1262 radio = new Module(SX_NSS, SX_DIO1, SX_RST, SX_BUSY,SPI_6);

// save transmission state between loops
int transmissionState = RADIOLIB_ERR_NONE;

// flag to indicate that a packet was sent
volatile bool transmittedFlag = false;

void setFlag(void) {
  // we sent a packet, set the flag
  transmittedFlag = true;
}

void setup() {
  pinMode(ANT_SW,OUTPUT); digitalWrite(ANT_SW,LOW);
  pinMode(LED_B, OUTPUT); digitalWrite(LED_B,LOW);

  delay(3000);  

  Serial.begin(115200);

  //set freq to 915, bandwidth 125khz, spreading factor SF8, Coding Rate 4/5, private header 0x12, +22 dBm, no TXO, NO LDO (false)
  int state = radio.begin(915.0,125,8,5,RADIOLIB_SX126X_SYNC_WORD_PRIVATE,10,8,0,false);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }
  radio.setCurrentLimit(140);
  radio.setOutputPower(22);
  
  //Serial.println(radio.checkOutputPower(22));

  // set the function that will be called
  // when packet transmission is finished
  radio.setPacketSentAction(setFlag);
  

  // start transmitting the first packet
  Serial.print(F("[SX1262] Sending first packet ... "));

  // you can transmit C-string or Arduino string up to
  // 256 characters long
  transmissionState = radio.startTransmit("Hello World!");
  //Serial.println(radio.checkOutputPower(22, 22));
}
// counter to keep track of transmitted packets
int count = 0;

void loop() {
  // check if the previous transmission finished
  if(transmittedFlag) {
    digitalWrite(LED_B,0);
    // reset flag
    transmittedFlag = false;

    if (transmissionState == RADIOLIB_ERR_NONE) {
      // packet was successfully sent
      Serial.println(F("transmission finished!"));

      // NOTE: when using interrupt-driven transmit method,
      //       it is not possible to automatically measure
      //       transmission data rate using getDataRate()

    } else {
      Serial.print(F("failed, code "));
      Serial.println(transmissionState);

    }

    // clean up after transmission is finished
    // this will ensure transmitter is disabled,
    // RF switch is powered down etc.
    radio.finishTransmit();

    digitalWrite(LED_B,1);
    // wait a second before transmitting again
    delay(1000);

    // send another one
    Serial.print(F("[SX1262] Sending another packet ... "));

    // you can transmit C-string or Arduino string up to
    // 256 characters long
    String str = "Hello World! #" + String(count++);
    transmissionState = radio.startTransmit(str);
    digitalWrite(LED_B,0);
    //Serial.println(radio.checkOutputPower(22));
  }
}
