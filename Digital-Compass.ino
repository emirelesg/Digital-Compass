#include <Adafruit_BNO055.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GPS.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Wire.h>
#include <utility/imumaths.h>

#define GPSSerial Serial1

#define BUTTON_A 9
#define BUTTON_B 6
#define BUTTON_C 5

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);
Adafruit_GPS GPS(&GPSSerial);

uint32_t timer = millis();

float distanceBetween(float lat1, float lon1, float lat2, float lon2) {
  // Uses the haversine formula to calculate the distance between two
  // coordinates in meters. The coordinates must be in degrees.

  float dlon = radians(lon2 - lon1);
  float dlat = radians(lat2 - lat1);
  float a = sq(sin(dlat / 2)) + cos(radians(lat1)) * cos(radians(lat2)) * sq(sin(dlon / 2));
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return c * 6371000.0;  // Radius of the Earth.
}

void setup() {
  // Serial init
  Serial.begin(115200);
  Serial.println("Display GPS data to OLED display");

  // GPS init
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  // 1 Hz update rate.
  delay(1000);

  // OLED init
  display.begin(0x3C, true);
  display.setRotation(1);
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.clearDisplay();
  display.display();

  // Bno
  bno.begin();
  bno.setExtCrystalUse(true);  // Use external crystal for better accuracy.

  // Buttons
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
}

void loop() {
  GPS.read();

  if (GPS.newNMEAreceived()) {
    Serial.println(GPS.lastNMEA());
    GPS.parse(GPS.lastNMEA());
  }

  if (!digitalRead(BUTTON_A)) Serial.print("A");
  if (!digitalRead(BUTTON_B)) Serial.print("B");
  if (!digitalRead(BUTTON_C)) Serial.print("C");

  if (millis() - timer > 2000) {
    timer = millis();

    sensors_event_t event;
    bno.getEvent(&event);

    display.clearDisplay();
    display.setCursor(0, 0);

    // Line 0, fix status, quality, and satellite count.
    display.print("Fix: ");
    display.print((int)GPS.fix);
    display.print(" Q: ");
    display.print((int)GPS.fixquality);
    display.print(" Sat: ");
    display.println((int)GPS.satellites);

    // Line 1, latitude.
    display.print("Lat: ");
    display.println(GPS.latitudeDegrees, 6);
    // display.print(GPS.latitude, 4);
    // display.println(GPS.lat);

    // Line 2, longitude.
    display.print("Lon: ");
    display.println(GPS.longitudeDegrees, 6);
    // display.print(GPS.longitude, 4);
    // display.println(GPS.lon);

    // Line 3, altitude.
    display.print("Alt: ");
    display.println(GPS.altitude, 2);

    // Line 4. distance.
    display.print("Dist: ");
    display.println(distanceBetween(GPS.latitudeDegrees, GPS.longitudeDegrees, 52.4771458, 13.4220666));

    // Line 5. bno calibration
    uint8_t sys, gyro, accel, mag = 0;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    display.print(F("Bno: "));
    display.print(sys, DEC);
    display.print(F(" "));
    display.print(gyro, DEC);
    display.print(F(" "));
    display.print(accel, DEC);
    display.print(F(" "));
    display.println(mag, DEC);

    display.display();
  }
}
