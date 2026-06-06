#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
#include <RadioLib.h>

SFE_UBLOX_GNSS myGNSS;

//GNSS UART CONNECTION
#define GNSS_TX_RX_0 PE7
#define GNSS_RX_TX_0 PE8

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

HardwareSerial Serial7(GNSS_TX_RX_0,GNSS_RX_TX_0);

//Setup Radio:
SPIClass SPI_6(SX_MOSI,SX_MISO,SX_SCK);

SX1262 radio = new Module(SX_NSS, SX_DIO1, SX_RST, SX_BUSY,SPI_6);

// save transmission state between loops
int transmissionState = RADIOLIB_ERR_NONE;

// flag to indicate that a packet was sent
volatile bool transmittedFlag = false;

long lastTime = 0; //Simple local timer. Limits amount of I2C traffic to u-blox module.

//GNSS Values
long latitude = 0;
long longitutde = 0;
byte SIV = 0;

void setFlag(void) {
  // we sent a packet, set the flag
  transmittedFlag = true;
}

void setup()
{
  pinMode(ANT_SW,OUTPUT); digitalWrite(ANT_SW,LOW);
  pinMode(LED_B, OUTPUT); digitalWrite(LED_B,LOW);

  delay(3000);
  Serial.begin(115200);
  //while (!Serial); //Wait for user to open terminal
  Serial.println("SparkFun u-blox Example");

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

  //allow current to be high enough to transmit at +22dBm
  radio.setCurrentLimit(140);
  //Function called when when packet transmission is finished
  radio.setPacketSentAction(setFlag);

  //Assume that the U-Blox GNSS is running at 9600 baud (the default) or at 38400 baud.
  //Loop until we're in sync and then ensure it's at 38400 baud.
  do {
    Serial.println("GNSS: trying 38400 baud");
    Serial7.begin(38400);
    if (myGNSS.begin(Serial7) == true) break;

    delay(100);
    Serial.println("GNSS: trying 9600 baud");
    Serial7.begin(9600);
    if (myGNSS.begin(Serial7) == true) {
        Serial.println("GNSS: connected at 9600 baud, switching to 38400");
        myGNSS.setSerialRate(38400);
        delay(100);
    } else {
        //myGNSS.factoryReset();
        delay(2000); //Wait a bit before trying again to limit the Serial output
    }
  } while(1);
  Serial.println("GNSS serial connected");

  myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfiguration(); //Save the current settings to flash and BBR

  // send the first packet on this node
  Serial.print(F("FFCv3_Ping"));
  transmissionState = radio.startTransmit("Hello World!");
  transmittedFlag = true;
}




void loop()
{  
  if(transmittedFlag && (millis() - lastTime > 1000)) {
  // reset flag
  transmittedFlag = false;
  lastTime = millis(); //Update the timer

  if (transmissionState == RADIOLIB_ERR_NONE) {
    // packet was successfully sent
    Serial.println(F("transmission finished!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(transmissionState);
  }
  // clean up after transmission is finished
  // this will ensure transmitter is disabled,
  // RF switch is powered down etc.
  radio.finishTransmit();

  // send another one
  Serial.print(F("[SX1262] Sending another packet ... "));

  // you can transmit C-string or Arduino string up to
  // 256 characters long
  String str = String(myGNSS.getLatitude()) + "," + String(myGNSS.getLongitude()) + "," + String(myGNSS.getSIV());
  transmissionState = radio.startTransmit(str);
  Serial.println(str);
  }
}