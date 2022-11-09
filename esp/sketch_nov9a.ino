#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <FS.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

const char *ssid = "ESP8266 Access Point";
const char *password = "password";

// ESP8266WiFiMulti wifiMulti;
ESP8266WebServer server(80);

void handle_root();
void handle_forwards();
void handle_backwards();

void setup() {
  Serial.begin(115200);

  setup_access_point();
  setup_webserver();

  SPIFFS.begin();
}

void setup_access_point() {
  WiFi.softAP(ssid, password);
  Serial.print("Access Point \"");
  Serial.print(ssid);
  Serial.println("\" started");

  Serial.print("IP address:\t");
  Serial.println(WiFi.softAPIP());
}

void setup_webserver() {
  server.on("/", handle_root);
  server.on("/forwards", HTTP_POST, handle_forwards);
  server.on("/backwards", HTTP_POST, handle_backwards);

  server.begin();
}

void loop() {
  server.handleClient();

}

void handle_root() {
  File file = SPIFFS.open("/index.html", "r");
  size_t sent = server.streamFile(file, "text/html");
  file.close();
}

void handle_forwards() {
  Serial.println("F");
}

void handle_backwards() {
  Serial.println("B");

}
