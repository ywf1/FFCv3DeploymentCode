#include <STM32SD.h>

File myFile;

unsigned long previousMicros = 0;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for Leonardo only
  }

  SD.setDx(PC8, PC9, PC10, PC11);
  SD.setCMD(PD2); // using PinName
  SD.setCK(PC12);

  Serial.print("Initializing SD card...");
  while (!SD.begin()) {
    delay(10);
  }
  Serial.println("initialization done.");

  // Open the file
  myFile = SD.open("test.txt", FILE_WRITE);

  unsigned long startTime = micros();
  unsigned long currentTime;

  Serial.println("Writing to test.txt...");

  // Write micros() as fast as possible for 1 second
  if (myFile) {
    while ((currentTime = micros()) - startTime < 1000000) { // 1 second duration
      myFile.println(currentTime);
    }
    myFile.close();
    Serial.println("Done writing.");
  } else {
    Serial.println("Error opening test.txt");
  }
}

void loop() {
  // nothing happens after setup
}