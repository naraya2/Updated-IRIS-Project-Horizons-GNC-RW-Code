#include <SPI.h>
#include <SD.h>

const int SD_CS_PIN = PA4;   // Your SD CS wire goes to A4 / PA4

void setup() {
  Serial.begin(115200);
  delay(3000);

  Serial.println("Starting SD card test...");

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD initialization failed.");
    Serial.println("Check wiring, CS pin, SD card format, or power.");
    return;
  }

  Serial.println("SD initialization successful.");

  File testFile = SD.open("sd_test.txt", FILE_WRITE);

  if (!testFile) {
    Serial.println("Could not open sd_test.txt for writing.");
    return;
  }

  testFile.println("SD CARD WRITE TEST");
  testFile.print("Time since startup: ");
  testFile.println(millis());
  testFile.println("If this file exists, SD writing works.");
  testFile.close();

  Serial.println("Write complete.");
  Serial.println("Remove SD card and check for sd_test.txt.");
}

void loop() {
}