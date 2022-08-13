#include <Adafruit_BNO055.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GPS.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_Sensor.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <utility/imumaths.h>

#define GPSSerial Serial1
#define SD_CS 4
#define BUTTON_A 9
#define BUTTON_B 6
#define BUTTON_C 5

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);
Adafruit_GPS GPS(&GPSSerial);

File calibration;

adafruit_bno055_offsets_t calibrationData;
boolean calibrationSaved = false;
boolean isCalibrated = false;

uint32_t timer = millis();

float bearingTo(float lat1, float lon1, float lat2, float lon2) {
  // Calculates the initial bearing to reach second coordinate.
  // North is 0° or 360. West is 270°.

  float dlon = radians(lon2 - lon1);

  lat1 = radians(lat1);
  lat2 = radians(lat2);

  float y = sin(dlon) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);
  float theta = atan2(y, x);

  if (theta < 0) {
    theta += TWO_PI;
  }

  return degrees(theta);
}

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

  // Bno
  bno.begin(bno.OPERATION_MODE_COMPASS);

  // Configuration that matches build.
  // See pg. 25 on BNO055 datasheet for details on remapping.
  bno.setAxisRemap(bno.REMAP_CONFIG_P6);
  bno.setAxisSign(bno.REMAP_SIGN_P6);

  // SD card
  if (SD.begin(SD_CS)) {
    calibration = SD.open("IMU.BIN", FILE_READ);
    if (calibration) {
      calibration.read((uint8_t *)&calibrationData, sizeof(calibrationData));
      calibration.close();
      bno.setSensorOffsets(calibrationData);
      isCalibrated = true;
    }
  } else {
    Serial.println("[Err] SD init");
  }

  // Crystal must be configured after calibration is loaded.
  bno.setExtCrystalUse(true);  // Use external crystal for better accuracy.

  // Buttons
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

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

  if (!digitalRead(BUTTON_A)) Serial.print("A");
  if (!digitalRead(BUTTON_B)) Serial.print("B");
  if (!digitalRead(BUTTON_C)) Serial.print("C");

  if (millis() - timer > 300) {
    timer = millis();

    display.clearDisplay();
    display.setCursor(0, 0);

    // Line 0, fix status, quality, and satellite count.
    display.print("Fix: ");
    display.print((int)GPS.fix);
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

    // Line 3. distance.
    display.print("Dist: ");
    display.println(distanceBetween(GPS.latitudeDegrees, GPS.longitudeDegrees, 52.4771458, 13.4220666));

    // Line 4. bno calibration
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

    // Bearing and heading calculations.
    sensors_event_t orientationData, magnetometerData;
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    float heading = orientationData.orientation.x;
    int headingStd = (int)(360 + 90 + heading) % 360;  // Convert heading to standard angle.

    float bearing = bearingTo(GPS.latitudeDegrees, GPS.longitudeDegrees, 52.4771458, 13.4220666);
    int bearingStd = (int)(360 + headingStd - bearing) % 360;  // Convert bearing to standard angle.

    // Line 5. Bearing
    float x0 = 105;
    float y0 = 35;
    float r = 18;
    display.drawCircle(x0, y0, r, SH110X_WHITE);

    int16_t x1 = x0 + 0.5 * r * cos(radians(headingStd));
    int16_t y1 = y0 - 0.5 * r * sin(radians(headingStd));

    display.print("Head: ");
    display.println(headingStd, 1);
    display.drawLine(x0, y0, x1, y1, SH110X_WHITE);

    // Line 6. Bearing
    int16_t x1_heading = x0 + r * cos(radians(bearingStd));
    int16_t y1_heading = y0 - r * sin(radians(bearingStd));

    display.print("Bear: ");
    display.println(bearingStd, 1);
    display.drawLine(x0, y0, x1_heading, y1_heading, SH110X_WHITE);

    display.display();

    if (!isCalibrated && bno.isFullyCalibrated()) {
      bno.getSensorOffsets(calibrationData);

      if (SD.exists("IMU.BIN")) {
        SD.remove("IMU.BIN");
      }

      calibration = SD.open("IMU.BIN", FILE_WRITE);

      if (calibration) {
        calibration.write((uint8_t *)&calibrationData, sizeof(calibrationData));
        calibration.close();
      }

      isCalibrated = true;
    }
  }
}
