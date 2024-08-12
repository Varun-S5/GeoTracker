#include "SoftwareSerial.h"
#include <TinyGPS++.h>

SoftwareSerial mySerial(16, 17);  // GSM module
SoftwareSerial GPS_SoftSerial(13, 15);  // GPS module (Rx, Tx)

String cmd = "";

TinyGPSPlus gps;

// Define the boundary coordinates (rectangle)
float boundaryMinLat = 12.92395; // Minimum latitude of the boundary
float boundaryMaxLat = 12.92434; // Maximum latitude of the bounary
float boundaryMinLon = 77.50042; // Minimum longitude of the boundary
float boundaryMaxLon = 77.50098; // Maximum longitude of the boundary

void setup() {
  Serial.begin(9600);  // Serial monitor
  mySerial.begin(9600);  // GSM module
  GPS_SoftSerial.begin(9600);  // GPS module

  Serial.println("Initializing...");
  delay(1000);

  mySerial.println("AT");  // Sends an ATTENTION command to the GSM module
  updateSerial();
  mySerial.println("AT+CMGF=1");  // Configuration for sending SMS
  updateSerial();
  mySerial.println("AT+CNMI=1,2,0,0,0");  // Configuration for receiving SMS
  updateSerial();
}

void loop() {
  updateSerial();
  smartDelay(1000);  // Update GPS data every second
  // Check if the GPS location is outside the boundary and send SMS if it is
  if (!isInsideBoundary(gps.location.lat(), gps.location.lng())) {
    sendSMS();
    // Wait for a while to avoid sending multiple SMS in quick succession
    delay(30000); // 30 seconds delay
  }
}

void updateSerial() {
  delay(500);
  while (Serial.available()) {
    cmd += (char)Serial.read();
    if (cmd != "") {
      cmd.trim();  // Remove added LF in transmit
      if (cmd.equals("S")) {
        sendSMS();
      } else {
        mySerial.print(cmd);
        mySerial.println("");
      }
    }
  }

  while (mySerial.available()) {
    Serial.write(mySerial.read());  // Forward what GSM module received to Serial Port
  }
}

void sendSMS() {
  mySerial.println("AT+CMGF=1");  // Set SMS text mode
  delay(500);
  mySerial.println("AT+CMGS=\"+918095343487\"");  // Specify recipient's phone number
  delay(500);
  mySerial.print(" Person Went Outside: ");
  float lat=gps.location.lat();
  float lng=gps.location.lng();
  mySerial.print("https://maps.google.com/?q="); // Start of URL
  mySerial.print(lat, 6); // Print latitude with 6 decimal places
  mySerial.print(","); // Separator
  mySerial.println(lng, 6);
  // mySerial.print(gps.location.lat(), 6);  // Send latitude
  // mySerial.print(", Longitude: ");
  // mySerial.print(gps.location.lng(), 6);  // Send longitude
  delay(500);
  // mySerial.print("Person went outside");
  mySerial.write(26);  // Send CTRL+Z to indicate the end of the message
  Serial.print("Latitude: ");
  Serial.print(gps.location.lat(), 6);
  Serial.print(", Longitude: ");
  Serial.println(gps.location.lng(), 6);
  Serial.println("Person in outside the boundary");
  Serial.println("Alert sent successfully");
}

void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (GPS_SoftSerial.available())
      gps.encode(GPS_SoftSerial.read());
  } while (millis() - start < ms);
}

bool isInsideBoundary(float lat, float lon) {
  // Check if the given latitude and longitude are inside the defined boundary
  if (lat < boundaryMinLat || lat > boundaryMaxLat || lon < boundaryMinLon || lon > boundaryMaxLon) {
    return false; // Outside the boundary
  } else {
    Serial.print("Latitude: ");
  Serial.print(gps.location.lat(), 6);
  Serial.print(", Longitude: ");
  Serial.println(gps.location.lng(), 6);
  Serial.println("Person is inside the boundary");
  delay(5000);

    return true; 
    // Inside the boundary
  }
}