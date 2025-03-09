/*
 * Air Quality Monitor with PM2.5, Temperature and Humidity Sensing
 * 
 * ฟังก์ชันการทำงาน:
 * - วัดค่าฝุ่น PM1.0, PM2.5, PM10 จากเซนเซอร์ PMS7003
 * - วัดอุณหภูมิและความชื้นจากเซนเซอร์ AHT10
 * - แสดงผลบนจอ OLED แบบ 3 หน้าจอ
 * - LED แสดงระดับคุณภาพอากาศด้วยการกะพริบ
 * - พัดลมทำงานตามค่าเฉลี่ยของฝุ่น PM2.5
 * การเชื่อมต่อ:
 * - OLED Display: SDA=21, SCL=22
 * - PMS7003: RX=16, TX=17
 * - LED: PIN 2
 * - FAN: PIN 23
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AHTX0.h>

// ===================== ค่าคงที่และการกำหนดค่า =====================
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

// กำหนดขา MINI FAN
#define MINI_FAN_PIN 25

// กำหนดระยะเวลา
#define READING_INTERVAL 10000         // ระยะเวลาในการอ่านค่า (10 วินาที)
#define DISPLAY_SWITCH_INTERVAL 8000   // ระยะเวลาในการสลับหน้าจอ (8 วินาที)
#define AVERAGE_COUNT 5                // จำนวนครั้งในการเก็บค่าเฉลี่ย

// กำหนดค่าระดับ PM2.5 สำหรับแต่ละเกณฑ์คุณภาพอากาศ
const float PM25_GOOD = 15.0;                // ดี
const float PM25_MODERATE = 25.0;            // ปานกลาง
const float PM25_UNHEALTHY_SENSITIVE = 37.5; // ไม่ดีต่อกลุ่มเสี่ยง
const float PM25_UNHEALTHY = 75.0;           // ไม่ดีต่อสุขภาพ
const float PM25_VERY_UNHEALTHY = 100.0;     // ไม่ดีต่อสุขภาพอย่างมาก
// > 100 = อันตราย

// Fan Control Constants
const int PWM_FREQ = 17000;      // KHz frequency for silent operation
const int PWM_RESOLUTION = 7;    // 7-bit resolution (0-100)
const int FAN_PIN = 23;          // Fan Pin

// Fan Speed Percentages (0-100 scale)
const int FAN_SPEED_OFF = 0;
const int FAN_SPEED_LOW = 10;
const int FAN_SPEED_MEDIUM = 30;
const int FAN_SPEED_HIGH = 60;        // 60%
const int FAN_SPEED_VERY_HIGH = 80;   // 80%
const int FAN_SPEED_MAX = 100;        // 100%

// เพิ่มค่าสำหรับการเปลี่ยนความเร็วอย่างนุ่มนวล
const int FAN_RAMP_DELAY = 500;  // ระยะเวลาหน่วงระหว่างการเปลี่ยนความเร็ว (milliseconds)
int targetFanSpeed = 0;          // ความเร็วพัดลมเป้าหมาย
int currentFanSpeed = 0;         // ความเร็วพัดลมปัจจุบัน

// ===================== โครงสร้างข้อมูล =====================
// โครงสร้างข้อมูลสำหรับ PMS7003
struct PMS7003_Data {
  uint16_t PM1_0;
  uint16_t PM2_5;
  uint16_t PM10;
  bool isValid;
};

// โครงสร้างข้อมูลสำหรับระดับคุณภาพอากาศ
struct AirQualityLevel {
  int level;      // ระดับ 1-6
  String status;  // สถานะ
  int blinkCount; // จำนวนครั้งที่กระพริบ
};

// โครงสร้างข้อมูลสำหรับ AHT10
struct AHT10_Data {
  float temperature;
  float humidity;
  bool isValid;
};

// โหมดการแสดงผล
enum DisplayMode {
  DUST_ALL,   // แสดงค่าฝุ่นทั้งหมด
  DUST_AVG,   // แสดงค่าเฉลี่ย PM2.5
  TEMP_HUMID  // แสดงอุณหภูมิและความชื้น
};

// ===================== ตัวแปรสากล =====================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_AHTX0 aht;

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

// ตัวแปรควบคุมการสลับหน้าจอ
DisplayMode currentDisplay = DUST_ALL;
unsigned long lastDisplaySwitch = 0;

// ===================== ฟังก์ชันสำหรับการประมวลผล =====================
// ฟังก์ชันประเมินระดับคุณภาพอากาศจากค่า PM2.5
AirQualityLevel getAirQualityLevel(float pm2_5) {
  AirQualityLevel level;

  if (pm2_5 <= PM25_GOOD) {
      level = {1, "GOOD", 0};
  } else if (pm2_5 <= PM25_MODERATE) {
      level = {2, "MODERATE", 2};
  } else if (pm2_5 <= PM25_UNHEALTHY_SENSITIVE) {
      level = {3, "UNHEALTHY-S", 3};
  } else if (pm2_5 <= PM25_UNHEALTHY) {
      level = {4, "UNHEALTHY", 4};
  } else if (pm2_5 <= PM25_VERY_UNHEALTHY) {
      level = {5, "VERY BAD", 5};
  } else {
      level = {6, "HAZARDOUS", 6};
  }
  
  return level;
}

// ฟังก์ชันควบคุมการกะพริบ LED
void updateLED(int blinkCount) {
  if (blinkCount == 0) {
    digitalWrite(LED_PIN, LOW);
    return;
  }

  unsigned long onDuration = 200;     // ระยะเวลาที่ LED ติด (0.2 วินาที)
  unsigned long offDuration = 200;    // ระยะเวลาที่ LED ดับ (0.2 วินาที)
  unsigned long pauseDuration = 1000; // ระยะเวลาพักระหว่างรอบ (1 วินาที)

  for (int i = 0; i < blinkCount; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(onDuration);
    digitalWrite(LED_PIN, LOW);
    delay(offDuration);
  }
  delay(pauseDuration);
}

// ฟังก์ชันควบคุมพัดลมตามค่าเฉลี่ย PM2.5
void updateFAN(float avgPM2_5) {
  // กำหนดระดับความเร็วพัดลมตามค่าเฉลี่ย PM2.5
  int fanSpeed;
  
  if (avgPM2_5 <= PM25_GOOD) {
    fanSpeed = FAN_SPEED_OFF;  // ปิดพัดลม
  } else if (avgPM2_5 <= PM25_MODERATE) {
    fanSpeed = FAN_SPEED_LOW;  // ความเร็วต่ำ
  } else if (avgPM2_5 <= PM25_UNHEALTHY_SENSITIVE) {
    fanSpeed = FAN_SPEED_MEDIUM;  // ความเร็วปานกลาง
  } else if (avgPM2_5 <= PM25_UNHEALTHY) {
    fanSpeed = FAN_SPEED_HIGH;  // ความเร็วสูง
  } else if (avgPM2_5 <= PM25_VERY_UNHEALTHY) {
    fanSpeed = FAN_SPEED_VERY_HIGH;  // ความเร็วสูงมาก
  } else {
    fanSpeed = FAN_SPEED_MAX;  // ความเร็วสูงสุด
  }
  
  targetFanSpeed = fanSpeed;
  
  // เปิด-ปิด พัดลมเล็ก
  digitalWrite(MINI_FAN_PIN, (targetFanSpeed > 0) ? HIGH : LOW);
  
  // ไม่ต้องทำอะไรถ้าความเร็วเดิมเท่ากับความเร็วใหม่
  if (currentFanSpeed == targetFanSpeed) {
    return;
  }
  
  // ปรับความเร็วอย่างช้าๆ จนถึงค่าเป้าหมาย
  if (currentFanSpeed < targetFanSpeed) {
    // เพิ่มความเร็ว
    for (int i = currentFanSpeed; i <= targetFanSpeed; i++) {
      ledcWrite(FAN_PIN, i);
      delay(FAN_RAMP_DELAY);
    }
  } else {
    // ลดความเร็ว
    for (int i = currentFanSpeed; i >= targetFanSpeed; i--) {
      ledcWrite(FAN_PIN, i);
      delay(FAN_RAMP_DELAY);
    }
  }
  
  currentFanSpeed = targetFanSpeed;
}

// ===================== ฟังก์ชันอ่านค่าเซนเซอร์ =====================
// อ่านค่าจากเซนเซอร์ PMS7003
PMS7003_Data readPMS7003() {
  PMS7003_Data data = {0, 0, 0, false};
  uint8_t buffer[32];
  unsigned long startTime = millis();
  
  // เคลียร์บัฟเฟอร์
  while (Serial2.available()) {
    Serial2.read();
  }
  
  // รอข้อมูลครบ 32 ไบต์
  while (Serial2.available() < 32) {
    if (millis() - startTime > 1000) { // timeout 1 วินาที
      return data;
    }
    delay(10);
  }
  
  // อ่านและตรวจสอบข้อมูล
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

// อ่านค่าจากเซนเซอร์ AHT10
AHT10_Data readAHT10() {
  AHT10_Data data = {0, 0, false};
  sensors_event_t humidity, temp;
  
  if (aht.getEvent(&humidity, &temp)) {
    data.temperature = temp.temperature;
    data.humidity = humidity.relative_humidity;
    data.isValid = true;
  }
  
  return data;
}

// ===================== ฟังก์ชันแสดงผล =====================
// ตั้งค่าจอ OLED เริ่มต้น
void setupOLED() {
  Wire.begin(SDA_PIN, SCL_PIN);
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("OLED not found!"));
    while (1);
  }
  
  // แสดงหน้าจอต้อนรับ
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

// แสดงค่าฝุ่นทั้งหมด
void displayPMData(const PMS7003_Data &airData) {
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
}

// แสดงค่าเฉลี่ย PM2.5
void displayAveragePM(float avgPM2_5) {
  display.clearDisplay();
  display.setTextSize(1);
  
  // หัวข้อ
  display.setCursor(0, 0);
  display.println(F("Average PM2.5"));
  display.drawLine(0, 9, 128, 9, SSD1306_WHITE);
  
  // แสดงจำนวนการอ่านค่า
  display.setCursor(0, 12);
  display.printf("Readings: %d/%d", readCount + 1, AVERAGE_COUNT);
  
  // แสดงค่า PM2.5 เฉลี่ย
  display.setTextSize(3);
  display.setCursor(5, 24);
  display.printf("%.1f", avgPM2_5);
  
  // แสดงหน่วย
  display.setTextSize(1);
  display.setCursor(95, 32);
  display.print(F("ug/m3"));
  
  // เส้นคั่น
  display.drawLine(0, 50, 128, 50, SSD1306_WHITE);
  
  // แสดงระดับอันตราย
  AirQualityLevel level = getAirQualityLevel(avgPM2_5);
  display.setTextSize(1);
  display.setCursor(5, 53);
  display.printf("[%d:6] %s", level.level, level.status.c_str());

  display.display();
}

// แสดงค่าอุณหภูมิและความชื้น
void displayAHTData(const AHT10_Data &ahtData) {
  display.clearDisplay();
  display.setTextSize(1);
  
  // หัวข้อ
  display.setCursor(0, 0);
  display.println(F("Temp & Humidity"));
  display.drawLine(0, 9, 128, 9, SSD1306_WHITE);
  
  // แสดงค่าอุณหภูมิ
  display.setTextSize(1);
  display.setCursor(5, 14);
  display.print(F("Temperature:"));
  display.setTextSize(3);
  display.setCursor(5, 24);
  display.printf("%.1fC", ahtData.temperature);
  
  // เส้นคั่น
  display.drawLine(0, 50, 128, 50, SSD1306_WHITE);
  
  // แสดงค่าความชื้น
  display.setTextSize(1);
  display.setCursor(5, 53);
  display.print(F("Humidity:"));
  display.setCursor(65, 53);
  display.printf("%.1f%%", ahtData.humidity);

  display.display();
}

// ===================== ฟังก์ชันหลัก =====================
void setup() {
  // เริ่มต้น Serial สำหรับ debug
  Serial.begin(115200);
  
  // เริ่มต้น Serial สำหรับ PMS7003
  Serial2.begin(9600, SERIAL_8N1, PMS_RX, PMS_TX);

  // Initialize PWM for fan
  ledcAttach(FAN_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(FAN_PIN, FAN_SPEED_OFF);
  
  // ตั้งค่า LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // ตั้งค่า MINI FAN
  pinMode(MINI_FAN_PIN, OUTPUT);
  digitalWrite(MINI_FAN_PIN, LOW);
  
  // เริ่มต้นเซนเซอร์ AHT10
  if (!aht.begin()) {
    Serial.println("Could not find AHT10 sensor!");
    while (1);
  }
  
  // ตั้งค่าจอ OLED
  setupOLED();
  
  Serial.println("Air Quality Monitor Started");
  Serial.println("Fan will adjust speed based on average PM2.5 readings");
}

void loop() {
  PMS7003_Data airData = readPMS7003();
  AHT10_Data ahtData = readAHT10();
  
  if (airData.isValid) {
    // คำนวณค่าเฉลี่ย
    avgPM1_0 = (avgPM1_0 * readCount + airData.PM1_0) / (readCount + 1);
    avgPM2_5 = (avgPM2_5 * readCount + airData.PM2_5) / (readCount + 1);
    avgPM10 = (avgPM10 * readCount + airData.PM10) / (readCount + 1);
    
    if (readCount < AVERAGE_COUNT - 1) {
      readCount++;
    }
    
    // แสดงข้อมูลที่ Serial Monitor สำหรับ Debug
    Serial.printf("PM1.0: %d, PM2.5: %d, PM10: %d ug/m3\n", 
                 airData.PM1_0, airData.PM2_5, airData.PM10);
    Serial.printf("Avg PM2.5: %.1f ug/m3\n", avgPM2_5);
    
    if (ahtData.isValid) {
      Serial.printf("Temperature: %.1f°C, Humidity: %.1f%%\n",
                   ahtData.temperature, ahtData.humidity);
    }
    
    // สลับการแสดงผลระหว่างหน้าต่างๆ
    if (millis() - lastDisplaySwitch >= DISPLAY_SWITCH_INTERVAL) {
      // สลับไปหน้าถัดไป
      switch(currentDisplay) {
        case DUST_ALL:
          currentDisplay = DUST_AVG;
          break;
        case DUST_AVG:
          currentDisplay = TEMP_HUMID;
          break;
        case TEMP_HUMID:
          currentDisplay = DUST_ALL;
          break;
      }
      lastDisplaySwitch = millis();
    }
    
    // แสดงผลตามโหมดปัจจุบัน
    switch(currentDisplay) {
      case DUST_ALL:
        displayPMData(airData);
        break;
      case DUST_AVG:
        displayAveragePM(avgPM2_5);
        break;
      case TEMP_HUMID:
        displayAHTData(ahtData);
        break;
    }
    
    // อัพเดท LED ตามระดับคุณภาพอากาศของค่าปัจจุบัน
    AirQualityLevel level = getAirQualityLevel(airData.PM2_5);
    updateLED(level.blinkCount);
    
    // อัพเดทพัดลมตามค่าเฉลี่ยของ PM2.5
    updateFAN(avgPM2_5);
    
    // แสดงสถานะพัดลม
    Serial.printf("Fan Speed: %d%% (PM2.5 Avg: %.1f)\n", currentFanSpeed, avgPM2_5);
  } else {
    Serial.println(F("Waiting for valid PMS7003 data..."));
  }
  
  delay(READING_INTERVAL);
}

/*
 * คำแนะนำการใช้งานและการแก้ไขปัญหา:
 * 
 * 1. การติดตั้ง:
 *    - ต้องติดตั้ง Library ต่อไปนี้:
 *      - Adafruit GFX Library
 *      - Adafruit SSD1306
 *      - Adafruit AHTX0
 *    - ตรวจสอบการต่อสายให้ถูกต้องตามที่กำหนดในส่วน define
 * 
 * 2. การปรับแต่ง:
 *    - สามารถปรับเวลาในการสลับหน้าจอได้ที่ DISPLAY_SWITCH_INTERVAL
 *    - ปรับเวลาในการอ่านค่าได้ที่ READING_INTERVAL
 *    - ปรับจำนวนครั้งในการเก็บค่าเฉลี่ยได้ที่ AVERAGE_COUNT
 *    - ปรับค่าระดับ PM2.5 สำหรับแต่ละเกณฑ์ได้ที่ตัวแปร PM25_*
 *    - ปรับความเร็วพัดลมในแต่ละระดับได้ที่ตัวแปร FAN_SPEED_*
 * 
 * 3. การแก้ไขปัญหา:
 *    - หากจอ OLED ไม่แสดงผล ให้ตรวจสอบการต่อสาย SDA/SCL และ address
 *    - หากไม่พบ PMS7003 ให้ตรวจสอบการต่อสาย TX/RX
 *    - หากไม่พบ AHT10 ให้ตรวจสอบการต่อสาย I2C
 *    - หากพัดลมไม่ทำงาน ตรวจสอบการต่อขา FAN_PIN และ MINI_FAN_PIN
 * 
 * 4. เพิ่มเติม:
 *    - พัดลมจะปรับความเร็วตามค่าเฉลี่ย PM2.5 (ไม่ใช่ค่าปัจจุบัน)
 *    - การปรับความเร็วพัดลมจะเป็นแบบค่อยๆ เพิ่มหรือลด
 *    - ทั้ง PWM และ MINI_FAN จะทำงานสัมพันธ์กัน
 */