#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_GPS.h>

#define GPSSerial Serial1

#define BUTTON_A  9
#define BUTTON_B  6
#define BUTTON_C  5

Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);
Adafruit_GPS GPS(&GPSSerial);

uint32_t timer = millis();

void setup() {

  // Serial init
  Serial.begin(115200);
  Serial.println("Display GPS data to OLED display");

  // GPS init
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate.
  delay(1000);

  // OLED init
  display.begin(0x3C, true); 
  display.setRotation(1);
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.clearDisplay();
  display.display();
}

void loop() {
  GPS.read();

  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
  }

  if (millis() - timer > 2000) {
    timer = millis();
  
    display.clearDisplay();
    display.setCursor(0,0);

    // Line 0, fix status, quality, and satellite count.
    display.print("Fix: "); display.print((int)GPS.fix);
    display.print(" Q: "); display.print((int)GPS.fixquality);
    display.print(" Sat: "); display.println((int)GPS.satellites);

    // Line 1, latitude.
    display.print("Lat: "); display.print(GPS.latitude, 4); display.println(GPS.lat);

    // Line 2, longitude.
    display.print("Lon: "); display.print(GPS.longitude, 4); display.println(GPS.lon);

    // Line 3, altitude.
    display.print("Alt: "); display.println(GPS.altitude);

    display.display();
  }
}
