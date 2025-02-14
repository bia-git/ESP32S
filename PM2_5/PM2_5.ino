#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// กำหนดขนาดจอ OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// กำหนดขา I2C และ LED
#define SDA_PIN 21
#define SCL_PIN 22
#define LED_PIN 2

// กำหนดขา Serial2 สำหรับ PMS7003
#define PMS_RX 16
#define PMS_TX 17
#define OLED_ADDRESS 0x3C
#define READING_INTERVAL 2000

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

struct PMS7003_Data {
  uint16_t PM1_0;
  uint16_t PM2_5;
  uint16_t PM10;
  bool isValid;
};

struct AirQualityLevel {
  int level;      // ระดับ 1-6
  String status;  // สถานะ
  int blinkCount; // จำนวนครั้งที่กระพริบ
};

// ตัวแปรควบคุม LED
unsigned long lastBlinkTime = 0;
bool ledState = false;
int currentBlinkCount = 0;
int totalBlinkCount = 0;

// ตัวแปรสำหรับค่าเฉลี่ย
float avgPM1_0 = 0;
float avgPM2_5 = 0;
float avgPM10 = 0;
int readCount = 0;
const int AVERAGE_COUNT = 5;

AirQualityLevel getAirQualityLevel(uint16_t pm2_5) {
    AirQualityLevel level;
    
    if (pm2_5 <= 15) {
        level = {1, "GOOD", 0};
    } else if (pm2_5 <= 25) {
        level = {2, "MODERATE", 2};
    } else if (pm2_5 <= 37.5) {
        level = {3, "UNHEALTHY-S", 3};
    } else if (pm2_5 <= 75) {
        level = {4, "UNHEALTHY", 4};
    } else if (pm2_5 <= 100) {
        level = {5, "VERY BAD", 5};
    } else {
        level = {6, "HAZARDOUS", 6};
    }
    
    return level;
}

void updateLED(int blinkCount) {
  static unsigned long lastChangeTime = 0;
  static int currentBlinkCount = 0;
  static bool isOn = false;
  unsigned long currentTime = millis();

  // ถ้าไม่ต้องกระพริบ
  if (blinkCount == 0) {
    digitalWrite(LED_PIN, LOW);
    return;
  }

  // เวลาสำหรับการกระพริบ
  unsigned long onDuration = 200;   // เปิด 200ms
  unsigned long offDuration = 200;  // ปิด 200ms
  unsigned long pauseDuration = 1000; // พัก 1 วินาที ระหว่างรอบการกระพริบ

  for (int i = 0; i < blinkCount; i++) {
    // เปิดไฟ
    digitalWrite(LED_PIN, HIGH);
    delay(onDuration);  // เปิดไฟ 200ms
    // ปิดไฟ
    digitalWrite(LED_PIN, LOW);
    delay(offDuration);  // ปิดไฟ 200ms
  }
  // หลังจากกระพริบครบจำนวนครั้งแล้ว รอ 1 วินาที
  delay(pauseDuration);  // พัก 1 วินาที
}

PMS7003_Data readPMS7003() {
  PMS7003_Data data = {0, 0, 0, false};
  uint8_t buffer[32];
  unsigned long startTime = millis();
  
  while (Serial2.available()) {
    Serial2.read();
  }
  
  while (Serial2.available() < 32) {
    if (millis() - startTime > 1000) {
      return data;
    }
    delay(10);
  }
  
  if (Serial2.readBytes(buffer, 32) == 32) {
    if (buffer[0] == 0x42 && buffer[1] == 0x4D) {
      uint16_t checksum = 0;
      for (int i = 0; i < 30; i++) {
        checksum += buffer[i];
      }
      uint16_t received_checksum = (buffer[30] << 8) | buffer[31];
      
      if (checksum == received_checksum) {
        data.PM1_0 = (buffer[10] << 8) | buffer[11];
        data.PM2_5 = (buffer[12] << 8) | buffer[13];
        data.PM10  = (buffer[14] << 8) | buffer[15];
        data.isValid = true;
      }
    }
  }
  
  return data;
}

void setupOLED() {
  Wire.begin(SDA_PIN, SCL_PIN);
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("OLED not found!"));
    while (1);
  }
  
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(15, 5);
  display.println(F("Air"));
  display.setCursor(15, 25);
  display.println(F("Quality"));
  display.setCursor(15, 45);
  display.println(F("Monitor"));
  display.display();
  delay(2000);
}

void displayAirData(const PMS7003_Data &airData) {
  display.clearDisplay();
  display.setTextSize(1);
  
  // หัวข้อ
  display.setCursor(0, 0);
  display.println(F("Air Quality Monitor"));
  display.drawLine(0, 9, 128, 9, SSD1306_WHITE);
  
  // แสดงค่า PM
  display.setCursor(0, 12);
  display.printf("PM1.0: %d ug/m3\n", airData.PM1_0);
  display.drawLine(0, 21, 128, 21, SSD1306_WHITE);
  
  display.setCursor(0, 24);
  display.printf("PM2.5: %d ug/m3\n", airData.PM2_5);
  display.drawLine(0, 33, 128, 33, SSD1306_WHITE);
  
  display.setCursor(0, 36);
  display.printf("PM10:  %d ug/m3\n", airData.PM10);
  display.drawLine(0, 45, 128, 45, SSD1306_WHITE);
  
  // แสดงสถานะคุณภาพอากาศ
  AirQualityLevel level = getAirQualityLevel(airData.PM2_5);
  display.setCursor(0, 48);
  display.printf("[%d:6] %s", level.level, level.status.c_str());
  
  display.display();
  
  // อัพเดท LED
  updateLED(level.blinkCount);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, PMS_RX, PMS_TX);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  setupOLED();
}

void loop() {
  PMS7003_Data airData = readPMS7003();
  
  if (airData.isValid) {
    // คำนวณค่าเฉลี่ย (ยังคงคำนวณแต่ไม่แสดงผล)
    avgPM1_0 = (avgPM1_0 * readCount + airData.PM1_0) / (readCount + 1);
    avgPM2_5 = (avgPM2_5 * readCount + airData.PM2_5) / (readCount + 1);
    avgPM10 = (avgPM10 * readCount + airData.PM10) / (readCount + 1);
    
    if (readCount < AVERAGE_COUNT - 1) {
      readCount++;
    }
    
    // แสดงข้อมูลที่ Serial Monitor
    Serial.printf("PM1.0: %d, PM2.5: %d, PM10: %d ug/m3\n", 
                 airData.PM1_0, airData.PM2_5, airData.PM10);
    Serial.printf("Avg PM2.5: %.1f ug/m3\n", avgPM2_5);
    
    displayAirData(airData);
  } else {
    Serial.println(F("Waiting for valid PMS7003 data..."));
  }
  
  delay(READING_INTERVAL);
}