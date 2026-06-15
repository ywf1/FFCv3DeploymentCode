/*
Yahya Farag
Pitt SOAR
6/14/2026
*/

/*
Development TODO list (in order of prority):
- 3 axis accel data - DONE (accelX/Y/Z logged)
- Initialize and setup HIGH G accel for logging - DONE (ADXL375 polled @200Hz, logged)
- update logging strucutre to accomodate high G, gyro, and 3 axis accel - DONE

- EEPROM based inflight recovery
- SPI FLASH data logging - TODO
- beep out main altitude
*/

//Radio Transmitter Settings
#define LORA_FREQ 926.375
#define LORA_BAND 125
#define LORA_PdBm 22

#define mainChuteAltitude 334.0         // Altitude (in meters) to deploy main parachute

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
Adafruit_ADXL375 highGaccel = Adafruit_ADXL375(12345,&Wire2);

SX1262 radio = new Module(SX_NSS, SX_DIO1, SX_RST, SX_BUSY,SPI_6); //initialize radio object

//Variables:
#define numSamples 30 //# of values in moving average
#define BUFF_SIZE 20

bool serialDebug = false;

bool radioEnabled = true; //set to false if operating radioless (IREC 2026)

//camera vars (RunCamSplitv4)
bool isCamera = true;

//continuity status (informational, not a launch gate)
bool cont1OK = false;
bool cont2OK = false;

//SD log flush cadence (periodic flush so an abnormal end doesn't lose buffered data)
unsigned long lastFlushTime = 0;
const unsigned long flushInterval = 1000000;   // 1 Hz


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

//gyro Vars
float totalXrot = 0;
float totalYrot = 0;
float totalZrot = 0;

//3-axis raw sensor data for logging (TODO: high-G + gyro + 3-axis accel)
float accelX = 0, accelY = 0, accelZ = 0;     // LSM6DSOX low-G accel, m/s^2 (all 3 axes)
float gyroX = 0, gyroY = 0, gyroZ = 0;        // LSM6DSOX gyro rates, rad/s
float hiGx = 0, hiGy = 0, hiGz = 0;           // ADXL375 high-G accel, m/s^2
unsigned long lastHiGTime = 0;
const unsigned long hiGInterval = 5000;       // 200 Hz high-G poll (I2C, bounded)

double batteryVoltage = 0; //yeah

// Timing Variables for Sampling delta time
unsigned long lastBaroTime = 0;
unsigned long lastAccelTime = 0;
unsigned long lastGyroTime = 0;


volatile unsigned long transmitPeriod = 1000000; //tranmit period for radio

// Complementary Filter Variables
double baroWeight = 0.0;
double accelWeight = 0.0;
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

// Status-indicator state (LED_B heartbeat + buzzer tones)
unsigned long lastHeartbeat = 0;
bool heartbeatOn = false;
int lastBuzzDuty = -1;            // -1 forces the first setPWM

//imu normalization
int mainAxis = 2;                              // Axis to use for main acceleration (0=x, 1=y, 2=z)
double axisSign = 1.0;

bool liftoffDetected = false;                  // Boolean to detect liftoff state
bool apogeeDetected = false;                   // Boolean to detect apogee state
bool burnoutDetected = false;                  //Boolean to detect motor burnout

bool barBaselineSet = false;                      // Tracks if baseline pressure has been set
bool accelBaselineSet = false;                    // Tracks if baseline accel has been set

const double liftoffAccelThreshold = 25;       // Acceleration threshold for liftoff (in m/s^2)
const double liftoffAltitudeThreshold = 50.0;   // Altitude threshold for liftoff (in meters)

const unsigned long apogeeDelay = 1000000;     //Nose over delay time
unsigned long apogeeTime = 0;     
double peakAltitude = 0.0;                      // running max altitude (telemetry / apogee backup)

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

//landing detection (ported pattern: stable low-and-slow for a sustained window)
unsigned long landingStableStart = 0;
const unsigned long LANDING_STABLE_US = 3000000UL;   // 3 s of stability before "landed"
const double LANDING_VEL_BAND    = 2.0;              // m/s; |baro velocity| must be under this
const double LANDING_ALT_CEILING = 30.0;             // m above pad; rejects the ~0 m/s instant at apogee

unsigned long liftoffTime = 0;

volatile bool accelReady = false;

volatile bool baroReady = false;

volatile bool gyroReady = false;

unsigned int packetNum = 0;


//IRQ Functions
void accelIRQ(void){
  accelReady = true;
}

void baroIRQ(void){
  baroReady = true;
}

void gyroIRQ(void){
  gyroReady = true;
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

  if(digitalRead(S0)){
    serialDebug = true;
  }
  
  delay(3000); //delay for connecting to Serial Terminal / sensors to start up

  digitalWrite(LED_B,HIGH);

  if(serialDebug){
    Serial.begin(115200); //Serial Port (USB)
    Serial.println("FFCv3 Initializing...");
  }

  /*
  SD SETUP
  */

  if(serialDebug){
    Serial.println("Initializing SD Card ...");
  }

  //Set SD card to pins
  SD.setDx(SDMMC_D0, SDMMC_D1, SDMMC_D2, SDMMC_D3); SD.setCMD(SDMMC_CMD); SD.setCK(SDMMC_CLK);

  if (!SD.begin()) {
    if(serialDebug){
      Serial.println("initialization failed...");
    }
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
    if(serialDebug)
      Serial.println("SD Card File Found and Created");

    dataFile.println("Time(us),Altitude(m),Acceleration(mps2),Baro Velocity(mps),Fused Velocity(mps),Accel Velocity(mps),Flight State,Latitude,Longitude,SIV,AccelX(mps2),AccelY(mps2),AccelZ(mps2),HiGx(mps2),HiGy(mps2),HiGz(mps2),GyroX(radps),GyroY(radps),GyroZ(radps)");
    dataFile.flush();
  }

  if(serialDebug)
    Serial.println("SD Initalized.");

  /*
  IMU SETUP
  */

  if(serialDebug)
    Serial.println("Initializing IMU ...");

  // Initialize LSM6DSOX IMU over SPI
  if (!lsm6dsox.begin_SPI(LSM_CS,&SPI_4)) {
    if(serialDebug)
      Serial.println("LSM6DSOX not detected. Check wiring.");
    errorCode1();
  }

  //accel setup
  lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G); // Set Acceleration Range to max (16G)
  lsm6dsox.setAccelDataRate(LSM6DS_RATE_6_66K_HZ); //set Accel Data Rate
  lsm6dsox.configInt2(false,false,true); //Configure Innterupt when Accel Data Ready
  attachInterrupt(digitalPinToInterrupt(LSM_INT2), accelIRQ, RISING); // set interrupt pin for lsm6dsox acceleration data

  //gyro setup
  lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS); // set gyro range to max
  lsm6dsox.setGyroDataRate(LSM6DS_RATE_6_66K_HZ); //set gyro data rate
  lsm6dsox.configInt1(false,true,false); //configure gyro int for
  attachInterrupt(digitalPinToInterrupt(LSM_INT1), gyroIRQ, RISING); // set interrupt pin for lsm6dsox gyro data
  delay(1000); //let imu normalize

  // Determine primary axis for flight direction based on initial attitude
  sensors_event_t accel;
  lsm6dsox.getEvent(&accel, NULL, NULL);
  double x = abs(accel.acceleration.x);
  double y = abs(accel.acceleration.y);
  double z = abs(accel.acceleration.z);

  float rawX = accel.acceleration.x;
  float rawY = accel.acceleration.y;
  float rawZ = accel.acceleration.z;

  // Find which axis has the largest absolute value (closest to +/- 9.81 m/s²)
  if (abs(rawX) > abs(rawY) && abs(rawX) > abs(rawZ)) {
    mainAxis = 0; // X is vertical
    if (rawX > 0) {
      axisSign = 1.0;
    } else {
      axisSign = -1.0;
    }
  } 
  else if (abs(rawY) > abs(rawX) && abs(rawY) > abs(rawZ)) {
    mainAxis = 1; // Y is vertical
    if (rawY > 0) {
      axisSign = 1.0;
    } else {
      axisSign = -1.0;
    }
  } 
  else {
    mainAxis = 2; // Z is vertical
    if (rawZ > 0) {
      axisSign = 1.0;
    } else {
      axisSign = -1.0;
    }
  }

  if(serialDebug)
    Serial.println("IMU - GOOD");
  /*
  BAROMETER SETUP
  */

  Wire.begin();
  // Check if sensor is connected and initialize
  if(pressureSensor.beginI2C(BMP581_I2C_ADDRESS_SECONDARY) != BMP5_OK)
  {
    // Not connected, inform user
    if(serialDebug)
      Serial.println("Error: BMP581 not connected, check wiring and I2C address!");

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

  if(serialDebug)
    Serial.println("BARO - GOOD");

  /*
  Accel SETUP
  */

  if(serialDebug)
    Serial.println("High G Accel Initalizing....");
  /* Initialise the sensor */

  if(!highGaccel.begin(0x1D))
  {
    errorCode1();
  }

  highGaccel.setDataRate(ADXL343_DATARATE_3200_HZ);


  if(serialDebug)
    Serial.println("High G Accel Initialized");

  /*
  GNSS SETUP
  */
  
  if(serialDebug)
    Serial.println("GNSS Initalizing....");

  unsigned long gnssStart = millis();

  //Assume that the U-Blox GNSS is running at 9600 baud (the default) or at 38400 baud.
  //Loop until we're in sync and then ensure it's at 38400 baud.
  do {

    Serial7.begin(38400);
    if (myGNSS.begin(Serial7) == true) break;

    delay(100);

    if(serialDebug)
      Serial.println("GNSS: trying 9600 baud");

    Serial7.begin(9600);
    if (myGNSS.begin(Serial7) == true) {
      if(serialDebug)
        Serial.println("GNSS: connected at 9600 baud, switching to 38400");
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
  
  if(serialDebug)
    Serial.println("GNSS serial connected");

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
    if(serialDebug)
      Serial.println(F("*** Warning: setDynamicModel failed ***"));
    errorCode1();
  }
  else
  {
    if(serialDebug)
      Serial.println(F("Dynamic platform model changed successfully!"));
  }

  if(serialDebug){
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
  }

  myGNSS.saveConfiguration(); //Save the current settings to flash and BBR

  if(serialDebug)
    Serial.println("GNSS Conected");

  ////////////
  ////CONT////
  ////////////
  // Continuity is INFORMATIONAL, not a launch gate. A missing/blown channel must not
  // brick the FC -- we still want to record and log (e.g. no-pyro test flights). Status
  // is flagged and signalled with a distinct warning chirp, but setup continues.
  if(serialDebug)
    Serial.println(F("Checking CONT... "));

  cont1OK = (analogRead(CONT1) >= 380);
  cont2OK = (analogRead(CONT2) >= 380);

  if(serialDebug){
    Serial.print(F("CONT1: "));  Serial.print(cont1OK ? F("OK") : F("OPEN"));
    Serial.print(F("   CONT2: ")); Serial.println(cont2OK ? F("OK") : F("OPEN"));
  }

  if(!cont1OK || !cont2OK){
    // 3 short warning chirps (non-halting), then carry on
    for(int i = 0; i < 3; i++){
      MyTim->setPWM(channel, BUZ, 4000, 50); delay(120);
      MyTim->setPWM(channel, BUZ, 4000, 0);  delay(120);
    }
  }

  ////////////
  ////RADIO///
  ////////////

  // Only touch the radio if it's enabled -- a radioless config (radioEnabled=false)
  // must not require the SX1262 to be present or healthy. And if it IS enabled but
  // init fails, recovery is baro-based and doesn't need telemetry: warn and fly
  // without the radio rather than halting the flight computer.
  if(radioEnabled){
    if(serialDebug)
      Serial.print(F("[SX1262] Initializing ... "));

    //freq, bw 125k, SF8, CR 4/5, private sync, +22dBm, preamble 8, no TCXO, no LDO
    int state = radio.begin(LORA_FREQ,LORA_BAND,8,5,RADIOLIB_SX126X_SYNC_WORD_PRIVATE,LORA_PdBm,8,0,false);
    if (state == RADIOLIB_ERR_NONE) {
      if(serialDebug)
        Serial.println(F("success!"));

      radio.setCurrentLimit(140);            //allow current high enough for +22dBm
      radio.setPacketSentAction(setFlag);    //ISR on TX-done

      if(serialDebug){
        Serial.println("Radio Initalized");
        Serial.print(F("FFCv3_Pinging..."));
      }

      transmissionState = radio.startTransmit("PING");
      transmittedFlag = true;
      lastTransmition = micros();
    } else {
      if(serialDebug){
        Serial.print(F("radio init failed, code "));
        Serial.println(state);
        Serial.println(F("-> continuing WITHOUT radio"));
      }
      radioEnabled = false;   // disable TX in the loop; keep flying/logging/recording
    }
  }

  if(serialDebug)
    Serial.println("Initializing buffers...");

  // Initialize moving average samples and sum
  for (int i = 0; i < numSamples; i++) {
    pressureSamples[i] = 0.0;
    accelSamples[i] = 0.0;
  }

  if(serialDebug)
    Serial.println("FFCV3 Initialized! Awaiting Liftoff....");

  flightState = 1; //Set Flight State to Waiting at PAD

  if(isCamera){
    setupCamera();
  }

  delay(3000);

  if(isCamera){
    startRecording();
  }

  //analogWriteResolution(12);
  //batteryVoltage = 2.0 * 3.3 * analogRead(VBAT) /  4095.0; <- this is for the newest FC
  //beepBatV(batteryVoltage); //beep out battery voltage

  delay(1000);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////LOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOP////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  if (baroReady) {
    baroReady = false;

    bmp5_sensor_data data = {0,0};
    int8_t err = pressureSensor.getSensorData(&data);

    // Read true temperature & pressure with compensation
    double realTemperature = data.temperature;
    double realPressure = data.pressure;

    if (barBaselineSet) {
      currentAltitude = 44330.0 * (1.0 - pow(movingAverage / baselinePressure, 0.1903));
      if (currentAltitude > peakAltitude) peakAltitude = currentAltitude;   // running peak

      // Update trend-based velocity
      unsigned long currentTime = micros();
      if (currentTime - previousBaroTime >= 100000) {  // 500 ms interval for trend calculation
        float deltaAltitude = currentAltitude - lastAltitude;
        float deltaTime = (currentTime - previousBaroTime) / 1.0e6; // Convert to seconds
        baroVelocity = deltaAltitude / deltaTime;  // Calculate trend-based velocity
        // Reset trend variables
        lastAltitude = currentAltitude;
        previousBaroTime = currentTime;
      }
    } 
  
    // Update moving average for barometric pressure
    pressureSum -= pressureSamples[sampleIndex];
    pressureSamples[sampleIndex] = realPressure;
    pressureSum += realPressure;
    sampleIndex = (sampleIndex + 1) % numSamples;

    movingAverage = pressureSum / numSamples;

    if (!barBaselineSet && sampleIndex == 0) {  // Set baseline after buffer fills
      baselinePressure = movingAverage;
      barBaselineSet = true;

      if(serialDebug)
        Serial.println("Baseline pressure set for relative altitude calculation.");
    }

  }

  if(accelReady){
    accelReady = false;
    // Get IMU acceleration on the main axis
    sensors_event_t accel;
    lsm6dsox.getEvent(&accel, NULL, NULL);

    accelX = accel.acceleration.x; accelY = accel.acceleration.y; accelZ = accel.acceleration.z; // 3-axis for logging
    
    double acceleration = mainAxis == 0 ? accel.acceleration.x : mainAxis == 1 ? accel.acceleration.y : accel.acceleration.z;
    
    acceleration *= axisSign; //set axis sign - Positive 9.81 on pad startup
    acceleration -= 9.81; //subtract gravitational force vector

    unsigned long currentTime = micros();
    // Integrate raw acceleration for velocity
    accel_dt = currentTime - lastAccelTime;
    accelVelocity += acceleration * accel_dt * 0.000001;
    lastAccelTime = currentTime;

    // Update moving average buffer
    accelSum -= accelSamples[accelIndex];
    accelSamples[accelIndex] = acceleration;
    accelSum += acceleration;
    accelIndex = (accelIndex + 1) % numSamples;

    // Compute moving average only after buffer is filled
    if (!accelBaselineSet && accelIndex == 0) {
      accelBaselineSet = true;  // Set baseline once the buffer is full
      if(serialDebug)
        Serial.println("IMU baseline set.");
    }

    //Calculate moving average and flip if necessary
    if (accelBaselineSet) {
      movingAvgAccel = accelSum / numSamples;
    }
  }

  //High-G accelerometer (ADXL375 on Wire2): polled ~200 Hz for logging during boost.
  if (micros() - lastHiGTime >= hiGInterval) {
    lastHiGTime = micros();
    sensors_event_t he;
    highGaccel.getEvent(&he);
    hiGx = he.acceleration.x; hiGy = he.acceleration.y; hiGz = he.acceleration.z;
  }

  //GNSS CHECK if Serial buffer is full
  if (myGNSS.getPVT())
  {
    latitude = myGNSS.getLatitude(); longitude = myGNSS.getLongitude(); gnssAltitude = myGNSS.getAltitude(); SIV = myGNSS.getSIV();
  }

  //batteryVoltage = 2.0 * 3.3 * analogRead(VBAT) /  1023.0; //experimental update battery shit

  if(gyroReady){
    gyroReady = false;

    unsigned long currentTime = micros();
    float gyroDt = (currentTime-lastGyroTime) / 1.0e6;
    lastGyroTime = currentTime;

    sensors_event_t gyro;

    lsm6dsox.getEvent(NULL, &gyro, NULL);
    gyroX = gyro.gyro.x; gyroY = gyro.gyro.y; gyroZ = gyro.gyro.z;   // rates (rad/s) for logging
    //integrate dps of each axis to get angle
    totalXrot += gyro.gyro.x * gyroDt;
    totalYrot += gyro.gyro.y * gyroDt;
    totalZrot += gyro.gyro.z * gyroDt;
  }

  //Gain Definitions
  if(!liftoffDetected){
    baroWeight = 0.0;
    accelWeight = 0.0;
  } else if(liftoffDetected && !burnoutDetected){
    baroWeight = 0.0;
    accelWeight = 1.0;
  } else if(liftoffDetected && burnoutDetected && accelVelocity <= 300){
    baroWeight = 0.05;
    accelWeight = 0.95;
  } else{ //this shouldnt happen in but is here just in case, this is the catch for non-acceleration flight (vacuum chamber) or accel failure
    baroWeight= .9; 
    accelWeight = .1;
  }

  // Apply complementary filter
  filteredVelocity = baroWeight * baroVelocity + accelWeight * accelVelocity;

  ////////////////
  //State Machine:
  ////////////////

  // Check if liftoff detected by altitude and acceleration thresholds.
  // The acceleration spike is the primary trigger (fires within ms of ignition, before
  // the rocket has risen); the altitude threshold is the backup if accel is missed.
  if (!liftoffDetected && accelBaselineSet && barBaselineSet) {
    bool accelLiftoff = (movingAvgAccel >= liftoffAccelThreshold);
    bool altLiftoff   = (currentAltitude >= liftoffAltitudeThreshold);
    if (accelLiftoff || altLiftoff){
      liftoffDetected = true;
      liftoffTime = micros();

      // Set baseline pressure @ liftoff (TODO): when the accel spike catches liftoff with
      // the rocket still ~on the pad, re-freeze the altitude baseline to launch-instant
      // pressure so pad-sit weather drift can't skew AGL (and the main-deploy altitude).
      // NOT on an altitude-only trigger -- by then the rocket is already at the threshold
      // and re-capturing would zero out the real altitude.
      if (accelLiftoff && currentAltitude < 10.0) {
        baselinePressure = movingAverage;
      }

      peakAltitude = currentAltitude;   // start peak tracking at liftoff

      logData = true;
      flightState = 2;

      if (serialDebug) 
        Serial.println("Liftoff detected.");
    }
  }

  if(liftoffDetected && !burnoutDetected && !apogeeDetected && movingAvgAccel < 0.5){
    burnoutDetected = true;

    if(serialDebug)
      Serial.println("Motor Burnout Detected");
  
    flightState = 3;
  }

  // Apogee Detection (fused velocity)
  if (liftoffDetected && burnoutDetected && !apogeeDetected && filteredVelocity < 0.1) { // When trend-based baro velocity ~ 0 at peak
    apogeeDetected = true;
    apogeeTime = micros();

    if(serialDebug)
      Serial.println("Apogee detected.");

    flightState = 4;
  }

  // Drogue pyro activation 1s after apogee detection
  if (apogeeDetected && (micros() - apogeeTime >= apogeeDelay) && !droguePyroActive) {
    digitalWrite(PYRO1_DROG, HIGH);
    droguePyroActive = true;
    drogueActivationTime = micros();

    if(serialDebug)
      Serial.println("Drogue pyro activated for drogue chute.");
  }

  // Main chute deployment at target altitude
  if (apogeeDetected && (currentAltitude <= mainChuteAltitude) && !mainChutePyroActive) {
    digitalWrite(PYRO2_MAIN, HIGH);
    mainChutePyroActive = true;
    mainChuteActivationTime = micros();

    if(serialDebug)
      Serial.println("Main chute pyro activated.");

    flightState = 5;
  }

  if (mainChutePyroActive && !landed && fabs(baroVelocity) < LANDING_VEL_BAND && currentAltitude < LANDING_ALT_CEILING) {
    if (landingStableStart == 0) {
      landingStableStart = micros();                 // start the stability timer
    } else if (micros() - landingStableStart >= LANDING_STABLE_US) {
      landed = true;
      flightState = 6;

      if (isCamera) 
        stopRecording();

      if (serialDebug) 
        Serial.println("landing detected");
    }
  } else {
    landingStableStart = 0;                          // condition broke -> reset timer
  }

  // Turn off pyros if they have been on for 1 second
  if (!droguePyroOver && droguePyroActive && (micros() - drogueActivationTime >= 1000000)) {
    digitalWrite(PYRO1_DROG, LOW);
    droguePyroOver = true;
    if(serialDebug)
      Serial.println("Drogue pyro deactivated.");
  }

  if (!mainPyroOver && mainChutePyroActive && (micros() - mainChuteActivationTime >= 1000000)) {
    digitalWrite(PYRO2_MAIN, LOW);
    mainPyroOver = true;
    if(serialDebug)
      Serial.println("Main chute pyro deactivated.");
  }

  ///////////////////////
  ///END STATE MACHINE///
  ///////////////////////

  //Log Data -- fixed char buffer (no heap churn) + periodic flush.
  // dtostrf is used instead of snprintf("%f") because STM32 nano-newlib disables
  // float printf by default; dtostrf is heap-free and always available.
  if (logData) {
    if (dataFile) {
      char line[256];
      char fb[16];
      int n = 0;
      n += snprintf(line+n, sizeof(line)-n, "%lu,", (unsigned long)(micros() - liftoffTime));
      n += snprintf(line+n, sizeof(line)-n, "%s,",  dtostrf(currentAltitude, 0, 2, fb));
      n += snprintf(line+n, sizeof(line)-n, "%s,",  dtostrf(movingAvgAccel, 0, 2, fb));
      n += snprintf(line+n, sizeof(line)-n, "%s,",  dtostrf(baroVelocity, 0, 2, fb));
      n += snprintf(line+n, sizeof(line)-n, "%s,",  dtostrf(filteredVelocity, 0, 2, fb));
      n += snprintf(line+n, sizeof(line)-n, "%s,",  dtostrf(accelVelocity, 0, 2, fb));
      n += snprintf(line+n, sizeof(line)-n, "%d,",  (int)flightState);
      n += snprintf(line+n, sizeof(line)-n, "%ld,", (long)latitude);
      n += snprintf(line+n, sizeof(line)-n, "%ld,", (long)longitude);
      n += snprintf(line+n, sizeof(line)-n, "%d,",  (int)SIV);
      n += snprintf(line+n, sizeof(line)-n, "%s,",  dtostrf(accelX, 0, 2, fb));
      n += snprintf(line+n, sizeof(line)-n, "%s,",  dtostrf(accelY, 0, 2, fb));
      n += snprintf(line+n, sizeof(line)-n, "%s,",  dtostrf(accelZ, 0, 2, fb));
      n += snprintf(line+n, sizeof(line)-n, "%s,",  dtostrf(hiGx, 0, 2, fb));
      n += snprintf(line+n, sizeof(line)-n, "%s,",  dtostrf(hiGy, 0, 2, fb));
      n += snprintf(line+n, sizeof(line)-n, "%s,",  dtostrf(hiGz, 0, 2, fb));
      n += snprintf(line+n, sizeof(line)-n, "%s,",  dtostrf(gyroX, 0, 4, fb));
      n += snprintf(line+n, sizeof(line)-n, "%s,",  dtostrf(gyroY, 0, 4, fb));
      n += snprintf(line+n, sizeof(line)-n, "%s",   dtostrf(gyroZ, 0, 4, fb));

      dataFile.println(line);

      if (!landed) {
        // periodic flush so a crash / lost power / undetected-landing doesn't drop data
        if (micros() - lastFlushTime >= flushInterval) {
          dataFile.flush();
          lastFlushTime = micros();
        }
      } else {
        dataFile.flush();
        dataFile.close();
        logData = false;
      }
    } else if (serialDebug) {
      Serial.println("Error writing to flight_log.csv");
    }
  }

  //when on pad transmit every 5s to conserve power
  //after liftoff transmit every one second
  //after landing transmit once every 10s for better battery life
  if(radioEnabled){
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

      if(serialDebug)
        Serial.print(F("[SX1262] Packet ... "));

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
        if(serialDebug)
          Serial.println(F("transmission finished!"));
      } else {
        if(serialDebug){
          Serial.print(F("failed, code "));
          Serial.println(transmissionState);
        }
      }

      radio.finishTransmit();

    }
  }

  updateIndicators();   // LED_B heartbeat + buzzer (armed chirp on pad / locator after landing)
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

// ---- Status indicators: LED_B heartbeat + piezo buzzer state tones ----
// Reconfigure the buzzer timer ONLY on change; setPWM every loop would glitch it.
void buzz(uint32_t duty){
  if ((int)duty != lastBuzzDuty){ MyTim->setPWM(channel, BUZ, 4000, duty); lastBuzzDuty = (int)duty; }
}

// Called once per loop.
//   LED_B  : ~2 Hz loop-alive heartbeat -- if it stops blinking, the loop has hung.
//   Buzzer : slow "armed" chirp on the pad, silent in flight (no one can hear it),
//            fast attention-getting locator beep once landed (for recovery).
void updateIndicators(){
  unsigned long now = micros();

  if (now - lastHeartbeat >= 250000){ lastHeartbeat = now; heartbeatOn = !heartbeatOn; digitalWrite(LED_B, heartbeatOn); }

  if (landed){                                   // recovery locator: fast beep
    if (now - lastBeepTime >= 200000){ lastBeepTime = now; beeperState = !beeperState; buzz(beeperState ? 50 : 0); }
  } else if (!liftoffDetected){                  // on the pad: slow armed chirp
    if (now - lastBeepTime >= 1000000){ lastBeepTime = now; beeperState = !beeperState; buzz(beeperState ? 50 : 0); }
  } else {                                       // in flight: silent
    buzz(0);
  }
}

// --------- CAMERA INTERFACE FUNCTIONS ---------
// This RunCam Split only honors Simulate-Power-Button (command 0x01, action 0x01),
// which TOGGLES recording -- the explicit START/STOP actions (0x03/0x04) do nothing on
// this unit (verified on the bench). So we track recording state in firmware and toggle
// EXACTLY ONCE per start/stop, gated on that state, to avoid the double-toggle that
// otherwise leaves the camera in the wrong mode. Assumes the camera powers on idle
// (not recording); if a toggle frame is ever dropped, state can desync -- the camera's
// record LED on the pad is the ground-truth check before launch.
#define RC_CMD_CAMERA_CONTROL  0x01
#define RC_ACTION_POWER_TOGGLE 0x01   // simulate power button = toggle record

bool isRecording = false;             // firmware's view of the camera (off at power-on)

void sendCamAction(uint8_t action){
  uint8_t buf[4];
  buf[0] = 0xCC;                  // header
  buf[1] = RC_CMD_CAMERA_CONTROL; // command
  buf[2] = action;                // action (0x01 = power-button toggle)
  buf[3] = calcCrc(buf, 3);       // crc8 (dvb-s2, poly 0xD5) over the 3 bytes
  rcSerial.write(buf, 4);
}

void setupCamera() {
  rcSerial.begin(115200);
  delay(1000);          // bring up the UART only; send no command here
  isRecording = false;  // camera powers on not recording
}

void startRecording() {
  if (!isRecording) {                    // toggle only if we believe it's stopped
    sendCamAction(RC_ACTION_POWER_TOGGLE);
    isRecording = true;
  }
}

void stopRecording() {
  if (isRecording) {                     // toggle only if we believe it's recording
    sendCamAction(RC_ACTION_POWER_TOGGLE);
    isRecording = false;
  }
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