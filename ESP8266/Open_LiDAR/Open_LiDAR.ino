#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// === Sensor I2C Addresses ===
#define AS5600_ADDR   0x36
#define ANGLE_MSB     0x0E
#define ANGLE_LSB     0x0F
#define TFLUNA_ADDR   0x10

// === Motor Pins ===
#define motorPin1 D7
#define motorPin2 D6

// === Wi-Fi & UDP Setup ===
const char* ssid = "LIDAR_ESP";
const char* password = "12345678";
WiFiUDP udp;
const IPAddress remoteIP(192, 168, 4, 2);  // IP of PC connected to ESP AP
const unsigned int remotePort = 12345;

// === State ===
bool running = false;  // Start in idle mode
char incomingPacket[64];

void setup() {
  Serial.begin(115200);
  Wire.begin(D2, D1);

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  stopMotor();  // Ensure motor is off on startup

  Serial.println("Starting WiFi AP...");
  WiFi.softAP(ssid, password);
  delay(2000);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  udp.begin(remotePort);  // Start listening for commands

  setTFLunaFrameRate_I2C(240);
}

// === Configure TF-Luna frame rate ===
void setTFLunaFrameRate_I2C(uint16_t rateHz) {
  uint8_t low  = rateHz & 0xFF;
  uint8_t high = (rateHz >> 8) & 0xFF;

  Wire.beginTransmission(TFLUNA_ADDR); Wire.write(0x26); Wire.write(low);  Wire.endTransmission();
  Wire.beginTransmission(TFLUNA_ADDR); Wire.write(0x27); Wire.write(high); Wire.endTransmission();
  Wire.beginTransmission(TFLUNA_ADDR); Wire.write(0x20); Wire.write(0x01); Wire.endTransmission();

  delay(100);
}

// === Read angle from AS5600 ===
float readAS5600AngleDegrees() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(ANGLE_MSB);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() == 2) {
    uint8_t msb = Wire.read(), lsb = Wire.read();
    int raw = ((msb << 8) | lsb) & 0x0FFF;
    return (raw * 360.0) / 4096.0;
  }
  return -1.0;
}

// === Read distance from TF-Luna ===
int readTFLunaDistanceCM() {
  Wire.beginTransmission(TFLUNA_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(TFLUNA_ADDR, 2);
  if (Wire.available() == 2) {
    uint8_t distL = Wire.read(), distH = Wire.read();
    return (distH << 8) | distL;
  }
  return -1;
}

void loop() {
  // === 1. Listen for control commands ===
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, 64);
    if (len > 0) {
      incomingPacket[len] = '\0';
      Serial.print("Received command: ");
      Serial.println(incomingPacket);

      if (strcmp(incomingPacket, "START") == 0) {
        running = true;
        startMotor();
        Serial.println("System started.");
      } else if (strcmp(incomingPacket, "STOP") == 0) {
        running = false;
        stopMotor();
        Serial.println("System stopped.");
      }
    }
  }

  // === 2. Run LIDAR + Encoder readings if running ===
  if (running) {
    int distance = readTFLunaDistanceCM();
    float angle = readAS5600AngleDegrees();

    if (distance >= 0 && angle >= 0) {
      char msg[32];
      sprintf(msg, "D:%d A:%.2f", distance, angle);
      udp.beginPacket(remoteIP, remotePort);
      udp.write(msg);
      udp.endPacket();
    }
  } else {
    delay(200);  // Sleep a little to save power when idle
  }
}

// === Motor control ===
void startMotor() {
  analogWrite(motorPin1, 200);
  digitalWrite(motorPin2, LOW);
}

void stopMotor() {
  analogWrite(motorPin1, 0);
  digitalWrite(motorPin2, LOW);
}
