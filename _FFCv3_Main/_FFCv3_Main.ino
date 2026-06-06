/*
Yahya Farag
Pitt SOAR
2/2/2026
*/

//Define Statment for Serial Terminal Debug, If commented DO NOT FLY
#define FLIGHT

//Radio Transmitter Settings
#define LORA_FREQ 926.375
#define LORA_BAND 125
#define LORA_PdBm 22

#define mainChuteAltitude 334.0         // Altitude (in meters) to deploy main parachute

// LOW PRIORITY:
//Initialize and setup HIGH G accel - TODO
//EEPROM based inflight recovery - TODO
//SPI FLASH data logging - TODO
//Receive activation signal for camera from ground station - TODO
//camera control & feedback - TODO
//link ANT_SW to radio controller - TODO
//gyro data - todo
//3 axis accel data - TODO
//high G data - TODO

//pinouts for FFCV3 Hardware
#include "FFCV3_PIN_DEFINITION.h"

#include <Wire.h>
#include <SPI.h>

#include <Adafruit_LSM6DSOX.h>
#include <STM32SD.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <RadioLib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>

//Piezo Buzzer PWM Timer Setup
TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(BUZ), PinMap_PWM);
uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(BUZ), PinMap_PWM));
HardwareTimer *MyTim = new HardwareTimer(Instance);

//ADXL375 I2C
TwoWire Wire2(ACCEL_SDA,ACCEL_SCL);

//LSM6DSOX SPI
SPIClass SPI_4(LSM_MOSI,LSM_MISO,LSM_CLK);

//GNSS UART
HardwareSerial Serial7(GNSS_TX_RX_0,GNSS_RX_TX_0);

//RunCam UART
HardwareSerial rcSerial(RX_1_RC_TX,TX_1_RC_RX);

//Setup Radio:
SPIClass SPI_6(SX_MOSI,SX_MISO,SX_SCK);

//Peripheral Object Decleration
Adafruit_LSM6DSOX lsm6dsox;                    // LSM6DSOX IMU sensor object
BMP581 pressureSensor;
File dataFile;                                 // SD Card file object
SFE_UBLOX_GNSS myGNSS;
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345,&Wire2);

SX1262 radio = new Module(SX_NSS, SX_DIO1, SX_RST, SX_BUSY,SPI_6); //initialize radio object

//Variables:
#define numSamples 30 //# of values in moving average
#define BUFF_SIZE 20

//camera vars (RunCamSplitv4)
bool isCamera = true;
uint8_t txBuf[BUFF_SIZE];
int recState = 0;

byte flightState = 0; //startup = 0, idle = 1, liftoff = 2, burnout = 3, apoggee & descent under drogue = 4, descent under main = 5, landing detected = 6;

// Moving Average Variables for Barometric Pressure
double pressureSamples[numSamples];
uint sampleIndex = 0;
volatile double pressureSum = 0.0;
volatile double movingAverage = 0.0;
double baselinePressure = 0.0;

// Moving Average Variables for Accelerometer Data
double accelSamples[numSamples];
uint accelIndex = 0;
double accelSum = 0.0;
double movingAvgAccel = 0.0;
double lastAccel = 0.0;                         
unsigned long accel_dt = 0;

double batteryVoltage = 0; //yeah

// Timing Variables for Sampling delta time
unsigned long lastBaroTime = 0;
unsigned long lastAccelTime = 0;

volatile unsigned long transmitPeriod = 1000000; //tranmit period for radio

// Complementary Filter Variables
double baroWeight = 0.9;
double accelWeight = 0.1;
double baroVelocity = 0.0;
double accelVelocity = 0.0;
double filteredVelocity = 0.0;

//baro velocity helpers
double lastAltitude = 0.0;
double currentAltitude = 0.0;
double deltaAltitude = 0.0;
unsigned long baro_dt = 0;             // d/dt trend-based velocity calculation
unsigned long previousBaroTime = 0;

unsigned long lastBeepTime = 0.0;
bool beeperState = false;

//imu normalization
int mainAxis = 2;                              // Axis to use for main acceleration (0=x, 1=y, 2=z)
double imuFlip = 1.0;
bool imuFlipped = false;

bool liftoffDetected = false;                  // Boolean to detect liftoff state
bool apogeeDetected = false;                   // Boolean to detect apogee state

bool barBaselineSet = false;                      // Tracks if baseline pressure has been set
bool accelBaselineSet = false;                    // Tracks if baseline accel has been set

const double liftoffAccelThreshold = 3.0;       // Acceleration threshold for liftoff (in m/s^2)
const double liftoffAltitudeThreshold = 50.0;   // Altitude threshold for liftoff (in meters)

const unsigned long apogeeDelay = 1000000;     //Nose over delay time
unsigned long apogeeTime = 0;     

unsigned long drogueActivationTime = 0;        // Tracks drogue pyro activation time
unsigned long mainChuteActivationTime = 0;     // Tracks main chute pyro activation time

//GNSS Values
int32_t latitude = 0;
int32_t longitude = 0;
int32_t gnssAltitude = 0;
byte SIV = 0;

bool droguePyroActive = false;
bool mainChutePyroActive = false;

bool droguePyroOver = false;
bool mainPyroOver = false;

bool logData = false;
bool landed = false;

unsigned long liftoffTime = 0;

volatile bool accelReady = false;

volatile bool baroReady = false;

unsigned int packetNum = 0;


//IRQ Functions
void accelIRQ(void){
  accelReady = true;
}

void baroIRQ(void){
  baroReady = true;
}

// save transmission state between loops
volatile int transmissionState = RADIOLIB_ERR_NONE;

// flag to indicate that a packet was sent
volatile bool transmittedFlag = false;

//time since last transmit
unsigned long lastTransmition = 0;

void setFlag(void) {
  // we sent a packet, set the flag
  transmittedFlag = true;
}

void setup() {
  //continuity and VBAT Analog pins
  pinMode(CONT1,INPUT_ANALOG); pinMode(CONT2,INPUT_ANALOG); pinMode(VBAT,INPUT_ANALOG);
  //RF Switch Pin
  pinMode(ANT_SW,OUTPUT); digitalWrite(ANT_SW,LOW);

  //Pin Setup
  pinMode(S0,INPUT);

  pinMode(PYRO1_DROG,OUTPUT); pinMode(PYRO2_MAIN,OUTPUT); pinMode(LED_B,OUTPUT);
  digitalWrite(PYRO1_DROG,LOW); digitalWrite(PYRO2_MAIN,LOW); digitalWrite(LED_B,LOW);
  
  delay(3000); //delay for connecting to Serial Terminal / sensors to start up

  digitalWrite(LED_B,HIGH);

  #ifndef FLIGHT
  Serial.begin(115200); //Serial Port (USB)
  #endif

  #ifndef FLIGHT
  Serial.println("FFCv3 Initializing...");
  #endif

  /*
  SD SETUP
  */

  #ifndef FLIGHT
  Serial.println("Initializing SD Card ...");
  #endif

  //Set SD card to pins
  SD.setDx(SDMMC_D0, SDMMC_D1, SDMMC_D2, SDMMC_D3); SD.setCMD(SDMMC_CMD); SD.setCK(SDMMC_CLK);

  if (!SD.begin()) {
    //Serial.println("initialization failed...");
    errorCode1();
  }

  uint16_t filenum = 1;
  char filename[15];
  while (filenum <= 9999) {   // find next available filename
    snprintf(filename, sizeof(filename), "flt_%04d.csv", filenum);
    if (!SD.exists(filename)) break;
    filenum++;
  }
  if (filenum > 9999) errorCode1();
  dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile) {
    #ifndef FLIGHT
    Serial.println("SD Card File Found and Created");
    #endif
    dataFile.println("Time(us),Altitude(m),Acceleration(mps2),Baro Velocity(mps),Fused Velocity(mps),Accel Velocity(mps), Flight State, Latitude, Longitude, SIV");
    dataFile.flush();
  }

  #ifndef FLIGHT
  Serial.println("SD Initalized.");
  #endif

  /*
  IMU SETUP
  */

  #ifndef FLIGHT
  Serial.println("Initializing IMU ...");
  #endif

  // Initialize LSM6DSOX IMU over SPI
  if (!lsm6dsox.begin_SPI(LSM_CS,&SPI_4)) {
    #ifndef FLIGHT
    Serial.println("LSM6DSOX not detected. Check wiring.");
    #endif
    errorCode1();
  }

  //accel setup
  lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G); // Set Acceleration Range to max (16G)
  lsm6dsox.setAccelDataRate(LSM6DS_RATE_6_66K_HZ); //set Accel Data Rate
  lsm6dsox.configInt2(false,false,true); //Configure Innterupt when Accel Data Ready
  attachInterrupt(digitalPinToInterrupt(LSM_INT2), accelIRQ, RISING); // set interrupt pin for lsm6dsox acceleration data

  #ifndef FLIGHT
  Serial.println("IMU - GOOD");
  #endif

  /*
  BAROMETER SETUP
  */

  Wire.begin();
  // Check if sensor is connected and initialize
  if(pressureSensor.beginI2C(BMP581_I2C_ADDRESS_SECONDARY) != BMP5_OK)
  {
    // Not connected, inform user
    #ifndef FLIGHT
    Serial.println("Error: BMP581 not connected, check wiring and I2C address!");
    #endif

    errorCode1();
  }

  // Variable to track errors returned by API calls
  int8_t err = BMP5_OK;

  err = pressureSensor.setMode(BMP5_POWERMODE_CONTINOUS);
  if(err != BMP5_OK)
  {
    errorCode1();
  }

  //multiplyers for Output data rate - 500Hz in for 1X,1X - Refer to table 9 in bmp581 datasheet
  bmp5_osr_odr_press_config osrMultipliers = {
      .osr_t = BMP5_OVERSAMPLING_1X,
      .osr_p = BMP5_OVERSAMPLING_1X,
      0,0 // Unused values, included to avoid compiler warnings-as-error
  };

  err = pressureSensor.setOSRMultipliers(&osrMultipliers);
  if(err)
  {
      errorCode1();
  }

  // Configure the BMP581 to trigger interrupts whenever a measurement is performed
  BMP581_InterruptConfig interruptConfig = {
      .enable   = BMP5_INTR_ENABLE,    // Enable interrupts
      .drive    = BMP5_INTR_PUSH_PULL, // Push-pull or open-drain
      .polarity = BMP5_ACTIVE_HIGH,    // Active low or high
      .mode     = BMP5_PULSED,         // Latch or pulse signal
      .sources  =
      {
          .drdy_en = BMP5_ENABLE,        // Trigger interrupts when data is ready
          .fifo_full_en = BMP5_DISABLE,  // Trigger interrupts when FIFO is full
          .fifo_thres_en = BMP5_DISABLE, // Trigger interrupts when FIFO threshold is reached
          .oor_press_en = BMP5_DISABLE   // Trigger interrupts when pressure goes out of range
      }
  };

  err = pressureSensor.setInterruptConfig(&interruptConfig);
  if(err != BMP5_OK)
  {
    errorCode1();
  }

  // Setup interrupt handler for BMP581
  attachInterrupt(digitalPinToInterrupt(BMP_INT), baroIRQ, RISING);

  #ifndef FLIGHT
  Serial.println("BARO - GOOD");
  #endif

  /*
  Accel SETUP
  */

  #ifndef FLIGHT
  Serial.println("High G Accel Initalizing....");
  #endif

  /* Initialise the sensor */
  if(!accel.begin(0x1D))
  {
    errorCode1();
  }

  accel.setDataRate(ADXL343_DATARATE_3200_HZ);

  #ifndef FLIGHT
  Serial.println("High G Accel Initialized");
  #endif

  /*
  GNSS SETUP
  */
  
  #ifndef FLIGHT
  Serial.println("GNSS Initalizing....");
  #endif

  unsigned long gnssStart = millis();

  //Assume that the U-Blox GNSS is running at 9600 baud (the default) or at 38400 baud.
  //Loop until we're in sync and then ensure it's at 38400 baud.
  do {
    #ifndef FLIGHT
    //Serial.println("GNSS: trying 38400 baud");
    #endif

    Serial7.begin(38400);
    if (myGNSS.begin(Serial7) == true) break;

    delay(100);

    #ifndef FLIGHT
    Serial.println("GNSS: trying 9600 baud");
    #endif

    Serial7.begin(9600);
    if (myGNSS.begin(Serial7) == true) {
      #ifndef FLIGHT
      Serial.println("GNSS: connected at 9600 baud, switching to 38400");
      #endif
      myGNSS.setSerialRate(38400);
      delay(100);
    } else {
      //myGNSS.factoryReset();
      delay(2000); //Wait a bit before trying again to limit the Serial output
    }
    if(millis() - gnssStart >= 10000){
      errorCode1();
    }
  } while(1);
  
  #ifndef FLIGHT
  Serial.println("GNSS serial connected");
  #endif

  myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

  //Set Navigation Frequency
  myGNSS.setNavigationFrequency(10); // 10 Hz

  // Set up auto PVT message (non-blocking)
 
  myGNSS.setAutoPVT(true); // Enable automatic NAV PVT messages
  // Set up auto PVT message (non-blocking)
  myGNSS.setNavigationFrequency(10); // 1 Hz
  myGNSS.setAutoPVT(true); // Enable automatic NAV PVT messages

  //Set Up Dynamic Mode
  if (myGNSS.setDynamicModel(DYN_MODEL_AIRBORNE4g) == false) // Set the dynamic model to PORTABLE
  {
    Serial.println(F("*** Warning: setDynamicModel failed ***"));
    errorCode1();
  }
  else
  {
    #ifndef FLIGHT
    Serial.println(F("Dynamic platform model changed successfully!"));
    #endif
  }

  #ifndef FLIGHT
  // Let's read the new dynamic model to see if it worked
  uint8_t newDynamicModel = myGNSS.getDynamicModel();
  if (newDynamicModel != DYN_MODEL_AIRBORNE4g)
  {
    Serial.println(F("*** Warning: getDynamicModel failed ***"));
  }
  else
  {
    Serial.print(F("The new dynamic model is: "));
    Serial.println(newDynamicModel);
  }
  #endif

  myGNSS.saveConfiguration(); //Save the current settings to flash and BBR

  #ifndef FLIGHT
  Serial.println("GNSS Conected");
  #endif

  ////////////
  ////CONT////
  ////////////
  #ifndef FLIGHT
  Serial.println(F("Checking CONT... "));
  #endif
    if(!(analogRead(CONT1) >= 380)){
    #ifndef FLIGHT
    Serial.println(analogRead(CONT1));
    #endif
    errorCode1();
  }
  if(!(analogRead(CONT2) >= 380)){
    #ifndef FLIGHT
    Serial.println(analogRead(CONT2));
    #endif
    errorCode1();
  }

  ////////////
  ////RADIO///
  ////////////

  #ifndef FLIGHT
  // initialize SX1262 with default settings
  Serial.print(F("[SX1262] Initializing ... "));
  #endif


  //set freq to 915, bandwidth 125khz, spreading factor SF7, Coding Rate 4/5, private header 0x12, +22 dBm, no TXO, NO LDO (false)
  int state = radio.begin(LORA_FREQ,LORA_BAND,8,5,RADIOLIB_SX126X_SYNC_WORD_PRIVATE,LORA_PdBm,8,0,false);
  if (state == RADIOLIB_ERR_NONE) {
    #ifndef FLIGHT
    Serial.println(F("success!"));
    #endif
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    errorCode1();
  }

  //allow current to be high enough to transmit at +22dBm
  radio.setCurrentLimit(140);
  //Function called when when packet transmission is finished
  radio.setPacketSentAction(setFlag);

  #ifndef FLIGHT
  Serial.println("Radio Initalized");
  #endif

  // send the first packet on this node
  #ifndef FLIGHT
  Serial.print(F("FFCv3_Pinging..."));
  #endif

  transmissionState = radio.startTransmit("PING");
  transmittedFlag = true;
  lastTransmition = micros();

  #ifndef FLIGHT
  Serial.println("Initializing buffers...");
  #endif

  // Initialize moving average samples and sum
  for (int i = 0; i < numSamples; i++) {
    pressureSamples[i] = 0.0;
    accelSamples[i] = 0.0;
  }

  // Determine primary axis for flight direction based on initial attitude
  sensors_event_t accel;
  lsm6dsox.getEvent(&accel, NULL, NULL);
  double x = abs(accel.acceleration.x);
  double y = abs(accel.acceleration.y);
  double z = abs(accel.acceleration.z);

  if (x > y && x > z) mainAxis = 0;
  else if (y > x && y > z) mainAxis = 1;
  else mainAxis = 2;

  #ifndef FLIGHT
  Serial.print("Primary axis for acceleration: ");
  Serial.println(mainAxis == 0 ? "X" : mainAxis == 1 ? "Y" : "Z");
  #endif

  lsm6dsox.getEvent(&accel, NULL, NULL);

  double acceleration = mainAxis == 0 ? accel.acceleration.x : mainAxis == 1 ? accel.acceleration.y : accel.acceleration.z;

  #ifndef FLIGHT
  Serial.println("FFCV3 Initialized! Awaiting Liftoff....");
  #endif

  flightState = 1; //Set Flight State to Waiting at PAD

  if(isCamera){
    setupCamera();
  }

  delay(1000);

  //analogWriteResolution(12);
  //batteryVoltage = 2.0 * 3.3 * analogRead(VBAT) /  4095.0; <- this is for the newest FC

  #ifndef FLIGHT
  Serial.println(batteryVoltage);
  #endif

  //beepBatV(batteryVoltage); //beep out battery voltage

  //TODO <- beep out main altitude

  delay(1000);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////LOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOP////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  unsigned long currentTime = micros();

  if (baroReady) {
    baroReady = false;

    bmp5_sensor_data data = {0,0};
    int8_t err = pressureSensor.getSensorData(&data);

    // Read true temperature & pressure with compensation
    double realTemperature = data.temperature;
    double realPressure = data.pressure;

    // Update moving average for barometric pressure
    pressureSum -= pressureSamples[sampleIndex];
    pressureSamples[sampleIndex] = realPressure;
    pressureSum += realPressure;
    sampleIndex = (sampleIndex + 1) % numSamples;

    movingAverage = pressureSum / numSamples;

     if (!barBaselineSet && sampleIndex == 0) {  // Set baseline after buffer fills
      baselinePressure = movingAverage;
      barBaselineSet = true;

      #ifndef FLIGHT
      Serial.println("Baseline pressure set for relative altitude calculation.");
      #endif
    }

    if (barBaselineSet) {
      currentAltitude = 44330.0 * (1.0 - pow(movingAverage / baselinePressure, 0.1903));

      // Update trend-based velocity
      if (currentTime - previousBaroTime >= 100000) {  // 500 ms interval for trend calculation
        float deltaAltitude = currentAltitude - lastAltitude;
        float deltaTime = (currentTime - previousBaroTime) / 1.0e6; // Convert to seconds
        baroVelocity = deltaAltitude / deltaTime;  // Calculate trend-based velocity

        // Reset trend variables
        lastAltitude = currentAltitude;
        previousBaroTime = currentTime;
      }
    } 
  }

  if(accelReady){
    accelReady = false;
    // Get IMU acceleration on the main axis
    sensors_event_t accel;
    lsm6dsox.getEvent(&accel, NULL, NULL);

    double acceleration = mainAxis == 0 ? accel.acceleration.x : mainAxis == 1 ? accel.acceleration.y : accel.acceleration.z;

    acceleration *= imuFlip;

    // Update moving average buffer
    accelSum -= accelSamples[accelIndex];
    accelSamples[accelIndex] = acceleration;
    accelSum += acceleration;
    accelIndex = (accelIndex + 1) % numSamples;

    // Integrate raw acceleration for velocity
    accel_dt = micros() - lastAccelTime;
    accelVelocity += (acceleration - 9.81) * accel_dt * 0.000001;
    lastAccelTime = micros();

    // Compute moving average only after buffer is filled
    if (!accelBaselineSet && accelIndex == 0) {
      accelBaselineSet = true;  // Set baseline once the buffer is full
      #ifndef FLIGHT
      Serial.println("IMU baseline set.");
      #endif
    }

    //Calculate moving average and flip if necessary
    if (accelBaselineSet) {
      movingAvgAccel = accelSum / numSamples;

      if(movingAvgAccel > 0 && !imuFlipped){
        imuFlipped = true;
        imuFlip = -1.0;
        #ifndef FLIGHT
        Serial.println("IMU FLIPPED....");
        #endif
      } 
    }
  }

  //GNSS CHECK if Serial buffer is full
  if (myGNSS.getPVT())
  {
    latitude = myGNSS.getLatitude(); longitude = myGNSS.getLongitude(); gnssAltitude = myGNSS.getAltitude(); SIV = myGNSS.getSIV();
  }

  batteryVoltage = 2.0 * 3.3 * analogRead(VBAT) /  1023.0; //experimental update battery shit

  //Gain Definitions
  if(!liftoffDetected){
    baroWeight = 0.99;
    accelWeight = 0.01;
  }else if(liftoffDetected && filteredVelocity <= 600 && movingAvgAccel >= -6.0){
    baroWeight = 0.3;
    accelWeight = 0.7;
  }else if(liftoffDetected && filteredVelocity > 600 && movingAvgAccel >= -6.0){
    baroWeight = 0.1;
    accelWeight = 0.9;
  } else{ //this shouldnt happen in but is here just in case, this is the catch for non-acceleration flight (vacuum chamber) or accel failure
    baroWeight= .9; 
    accelWeight = .1;
  }

  // Apply complementary filter
  filteredVelocity = baroWeight * baroVelocity + accelWeight * accelVelocity;


  //Raw Data Stream (Debugging) -> high bandwidth not recommended
  if(digitalRead(S0)){
    #ifndef FLIGHT
    String str = String(currentAltitude) + "," + String(movingAvgAccel)  + "," + String(baroVelocity) + "," + String(flightState) + "," + String(latitude) + "," + String(longitude) + "," + String(SIV);
    Serial.println(str);
    delay(100);
    #endif
  }

  ////////////////
  //State Machine:
  ////////////////

  // Check if liftoff detected by altitude and acceleration thresholds
  if (!liftoffDetected && accelBaselineSet && barBaselineSet) {
    if ((currentAltitude >= liftoffAltitudeThreshold) || (movingAvgAccel >= (liftoffAccelThreshold - 9.8) && currentAltitude >= 10)){
      liftoffDetected = true;
      digitalWrite(LED_B, LOW);  // Turn on LED for liftoff indication

      #ifndef FLIGHT
      Serial.println("Liftoff detected.");
      #endif
      
      logData = true;
      liftoffTime = micros();
      flightState = 2;

      if(isCamera){
        startRecording();
      }
    }
  }

  // Apogee Detection (fused velocity)
  if (liftoffDetected && !apogeeDetected && filteredVelocity < 0.1) { // When trend-based baro velocity ~ 0 at peak
    apogeeDetected = true;
    apogeeTime = micros();

    #ifndef FLIGHT
    Serial.println("Apogee detected.");
    #endif

    digitalWrite(LED_B,HIGH); //turnoff LED for apoggee detection
    flightState = 4;
  }

  // Drogue pyro activation 1s after apogee detection
  if (apogeeDetected && (micros() - apogeeTime >= apogeeDelay) && !droguePyroActive) {
    digitalWrite(PYRO1_DROG, HIGH);
    droguePyroActive = true;
    drogueActivationTime = micros();

    #ifndef FLIGHT
    Serial.println("Drogue pyro activated for drogue chute.");
    #endif
  }

  // Main chute deployment at target altitude
  if (apogeeDetected && (currentAltitude <= mainChuteAltitude) && !mainChutePyroActive) {
    digitalWrite(PYRO2_MAIN, HIGH);
    mainChutePyroActive = true;
    mainChuteActivationTime = micros();
    #ifndef FLIGHT
    Serial.println("Main chute pyro activated.");
    #endif
    flightState = 5;
  }

  //landing detection
  if (mainChutePyroActive && baroVelocity >= -1) {
    flightState = 6;
    landed = true;

    #ifndef FLIGHT
    Serial.println("landed detected");
    #endif

    digitalWrite(LED_B,LOW);

    if(isCamera){
      stopRecording();
    }

    MyTim->setPWM(channel, BUZ, 4000, 50); // 4kHz, 10% dutycycle
  }

  // Turn off pyros if they have been on for 1 second
  if (!droguePyroOver && droguePyroActive && (micros() - drogueActivationTime >= 1000000)) {
    digitalWrite(PYRO1_DROG, LOW);
    droguePyroOver = true;
    #ifndef FLIGHT
    Serial.println("Drogue pyro deactivated.");
    #endif
  }

  if (!mainPyroOver && mainChutePyroActive && (micros() - mainChuteActivationTime >= 1000000)) {
    digitalWrite(PYRO2_MAIN, LOW);
    mainPyroOver = true;
    #ifndef FLIGHT
    Serial.println("Main chute pyro deactivated.");
    #endif
  }

  ///////////////////////
  ///END STATE MACHINE///
  ///////////////////////

  //Log Data
  if (logData) {
    //turn off intterupts so avoid memory corruption
    noInterrupts();
    // Log data to file
    if (dataFile && !landed) {
      String dataCol = String(micros() - liftoffTime) + "," + String(currentAltitude) + "," + String(movingAvgAccel) + "," +
                       String(baroVelocity) + "," + String(filteredVelocity) + "," + String(accelVelocity) + "," + String(flightState) + "," + String(latitude) 
                       + "," + String(longitude) + "," + String(SIV);
      dataFile.println(dataCol);
      //dataFile.flush();
    }
    else if(dataFile && landed){
      String dataCol = String(micros() - liftoffTime) + "," + String(currentAltitude) + "," + String(movingAvgAccel) + "," +
                       String(baroVelocity) + "," + String(filteredVelocity) + "," + String(accelVelocity) + "," + String(flightState) + "," + String(latitude) 
                       + "," + String(longitude) + "," + String(SIV);
      dataFile.println(dataCol);
      dataFile.flush();
      dataFile.close();
      logData = false;
    }else {
      #ifndef FLIGHT
      Serial.println("Error writing to flight_log.csv");
      #endif
    }
    //re-enable interrupts
    interrupts();
  }

  //when on pad transmit every 5s to conserve power
  //after liftoff transmit every one second
  //after landing transmit once every 10s for better battery life
  if(!liftoffDetected){
    transmitPeriod = 5000000;
  }else if(liftoffDetected && !landed){
    transmitPeriod = 1000000;
  }else{
    transmitPeriod = 10000000;
  }

  //packet transmission every 1s
  if(micros() - lastTransmition >= transmitPeriod && !transmittedFlag){
    // send another one
    lastTransmition = micros();

    #ifndef FLIGHT
    //Serial.print(F("[SX1262] Packet ... "));
    #endif

    if(liftoffDetected){
      packetNum++;
    }

    //String str = String(flightState) + "," + String(filteredVelocity) + "," + String(currentAltitude) + "," + String(latitude)  + "," + String(longitude) + "," + String(SIV);
    String str = String(packetNum) + "," + String(latitude)  + "," + String(longitude) + "," + String(SIV) + "," + String(currentAltitude) + "," + String(flightState) + "," + String(batteryVoltage);

    transmissionState = radio.startTransmit(str);
  }

  if(transmittedFlag) {
    // reset flag
    transmittedFlag = false;
    if (transmissionState == RADIOLIB_ERR_NONE) {
      // packet was successfully sent
      #ifndef FLIGHT
      //Serial.println(F("transmission finished!"));
      #endif
    } else {
      #ifndef FLIGHT
      Serial.print(F("failed, code "));
      Serial.println(transmissionState);
      #endif
    }
    radio.finishTransmit();
  }

  //beep when standby
  if (currentTime - lastBeepTime >= 1000000 && !liftoffDetected) {
    lastBeepTime = currentTime;
    beeperState = !beeperState;
    if(beeperState){
      MyTim->setPWM(channel, BUZ, 4000, 50); // 4kHz , 0% dutycycle
    } else{
      MyTim->setPWM(channel, BUZ, 4000, 0); // 4kHz , 0% dutycycle
    }
  }
}

//beep function
void errorCode1(){
  while(1){
    MyTim->setPWM(channel, BUZ, 4000, 0); // 4kHz , 0% dutycycle
    delay(50);
    MyTim->setPWM(channel, BUZ, 4000, 50); // 4kHz, 50% dutycycle
    delay(50);
  }
}

// --------- CAMERA INTERFACE FUNCTIONS ---------
void setupCamera() {
  //delay(100);  // Optional delay before init
  rcSerial.begin(115200);
  delay(1000);  // Allow time for camera to initialize

  // Build command to toggle recording
  txBuf[0] = 0xCC;
  txBuf[1] = 0x01;  // Command ID
  txBuf[2] = 0x01;  // Parameter (toggle)
  txBuf[3] = calcCrc(txBuf, 3);
  delay(1000);
  rcSerial.write(txBuf, 4);
}

void startRecording() {
  rcSerial.write(txBuf, 4);
}

void stopRecording() {
  rcSerial.write(txBuf, 4);
}

uint8_t calcCrc(uint8_t *buf, uint8_t numBytes) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < numBytes; i++)
        crc = crc8_calc(crc, *(buf + i), 0xD5);
    return crc;
}

uint8_t crc8_calc(uint8_t crc, unsigned char a, uint8_t poly) {
  crc ^= a;
  for (int ii = 0; ii < 8; ++ii) {
    if (crc & 0x80)
      crc = (crc << 1) ^ poly;
    else
      crc = crc << 1;
  }
  return crc;
}

void beepBatV(double bv){
  int firstNumber = (int) bv; // Extract first number (integer part)
  double decimalPart = bv - firstNumber; // Get decimal part
  
  // Extract first two decimal places
  decimalPart = decimalPart * 100; // Shift decimal points to the right by 2
  int secondDecimal = (int) decimalPart % 10; // Get the second decimal
  int firstDecimal = ((int) decimalPart / 10) % 10; // Get the first decimal

  MyTim->setPWM(channel, BUZ, 4000, 50); // 5 Hertz, 10% dutycycle
  delay(5000);
  MyTim->setPWM(channel, BUZ, 4000, 0); // 5 Hertz, 10% dutycycle
  delay(1000);  
  for(int i = 0; i < firstNumber; i++){
    MyTim->setPWM(channel, BUZ, 4000, 50); // 5 Hertz, 10% dutycycle
    delay(1000);
    MyTim->setPWM(channel, BUZ, 4000, 0); // 5 Hertz, 10% dutycycle
    delay(1000);
  }
  MyTim->setPWM(channel, BUZ, 4000, 50); // 5 Hertz, 10% dutycycle
  delay(3000);
  MyTim->setPWM(channel, BUZ, 4000, 0); // 5 Hertz, 10% dutycycle
  delay(1000);
  for(int i = 0; i < firstDecimal; i++){
    MyTim->setPWM(channel, BUZ, 4000, 50); // 5 Hertz, 10% dutycycle
    delay(1000);
    MyTim->setPWM(channel, BUZ, 4000, 0); // 5 Hertz, 10% dutycycle
    delay(1000);
  }
  MyTim->setPWM(channel, BUZ, 4000, 50); // 5 Hertz, 10% dutycycle
  delay(3000);
  MyTim->setPWM(channel, BUZ, 4000, 0); // 5 Hertz, 10% dutycycle
  delay(1000);
  for(int i = 0; i < secondDecimal; i++){
    MyTim->setPWM(channel, BUZ, 4000, 50); // 5 Hertz, 10% dutycycle
    delay(1000);
    MyTim->setPWM(channel, BUZ, 4000, 0); // 5 Hertz, 10% dutycycle
    delay(1000);
  }

  MyTim->setPWM(channel, BUZ, 4000, 0); // 5 Hertz, 10% dutycycle
}