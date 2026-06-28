#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include "Adafruit_SHT31.h"

// --- ĐỊNH NGHĨA KHUNG DỮ LIỆU ---
#define FRAME_HEADER 0xAA // Header quy định tạm thời
#define FRAME_END    0xFF // Byte kết thúc
#define NUMBER_OF_MEASUREMENTS 3 // So lan do du lieu 
#define LORA_NSS 8
#define LORA_RST 7
#define LORA_DIO0  2 

#define SENSOR_POWER_PIN 5
#define SOIL_PIN A0 

// -- Dinh nghia byte header ----
#define HEADER_ADV    0xA1
#define HEADER_ACK    0xA2
#define HEADER_DATA   0xB1
#define FRAME_END     0xFF 

#define SLOT_DURATION_SEC 5

// --- BIẾN CẤU HÌNH (GIAI ĐOẠN 1) ---
bool isConfigured = false; 
uint8_t mySensorID = 0x05; 
uint8_t targetRouterID = 0x01; 
volatile uint32_t timeSlotSeconds = 0; // Thời gian ngủ (giây) được Gateway cấp
uint8_t myTdmaSlot = 2; 

// --- BIẾN ĐẾM THỜI GIAN (GIAI ĐOẠN 2) ---
volatile uint32_t secondsCounter = 0;  // Biến đếm số giây đã ngủ

// Khởi tạo đối tượng cảm biến SHT31/SHT30
Adafruit_SHT31 sht30 = Adafruit_SHT31();

//Cònfiguration SleepTime
void setupTimer2_RTC()
{
  cli(); // Tắt ngắt toàn cục để cài đặt an toàn
  
  TCCR2A = 0;   // Tắt Timer 2
  TCCR2B = 0;   // Tắt Timer 2
  
  // Kich hoat che do Asynchronous Timer 2 (Dùng thạch anh ngoài 32.768kHz ở chân 9, 10)
  ASSR |= (1 << AS2); 
  
  TCNT2 = 0;   // Thiết lập bộ đếm (TCNT2) về 0
  // Cài đặt Prescaler = 128 -> (32768 / 128 = 256 xung/giây). Timer 8-bit tràn ở 256 -> Tràn 1 lần/giây.
  TCCR2B |= (1 << CS22) | (1 << CS20); 
  
  // Chờ cho các thanh ghi cập nhật xong (Yêu cầu bắt buộc của datasheet)
  while (ASSR & ((1 << TCN2UB) | (1 << OCR2AUB) | (1 << TCR2AUB) | (1 << TCR2BUB)));

  TIMSK2 |= (1 << TOIE2); // Bật ngắt tràn Timer 2
  sei(); // Bật lại ngắt toàn cục
}


ISR(TIMER2_OVF_vect) {
  secondsCounter++; // Trình phục vụ ngắt
}

// Hàm chuẩn hóa giá trị về 0-100% (Ví dụ cho độ ẩm đất đo bằng Analog)
uint8_t normalizeData(int rawValue, int minVal, int maxVal)
{
  if (rawValue <= minVal) return 0;
  if (rawValue >= maxVal) return 100;
  return map(rawValue, minVal, maxVal, 0, 100);
}

void enterPowerSaveSleep(){
  // 1. Tắt chống rò dòng qua chân I2C
  TWCR &= ~(1<<TWEN); // Tắt phần cứng I2C
  pinMode(A4, OUTPUT); digitalWrite(A4, LOW); // Ép SDA về GND
  pinMode(A5, OUTPUT); digitalWrite(A5, LOW); // Ép SCL về GND

  // Ham dieu khien chu trinh hoat dong
  // Tat tat ca cac ngoai vi (ADC, SPI, I2C...)
  power_all_disable(); 
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  sleep_enable();
  sleep_cpu();      // Bắt đầu NGỦ 
  sleep_disable();  // THỨC DẬY (do ngắt Timer 2)
}

void joinNetworkPhase() {
  uint8_t advFrame[3] = {HEADER_ADV, mySensorID, targetRouterID};
  
  while (!isConfigured) {
    // 1. Gửi gói tin xin gia nhập (ADV_Node)
    LoRa.beginPacket();
    LoRa.write(advFrame, 3);
    LoRa.endPacket();

    // 2. Lắng nghe chờ phản hồi trong vòng 5 giây
    LoRa.receive(); 
    unsigned long startWait = millis();
    bool ackReceived = false;
    
    while (millis() - startWait < 5000) {
      int packetSize = LoRa.parsePacket();
      if (packetSize >= 6) {
        uint8_t header = LoRa.read();
        uint8_t rxSensID = LoRa.read();
        uint8_t rxRoutID = LoRa.read();
        
        // Kiểm tra đúng là gói ACK dành cho mình không
        if (header == HEADER_ACK && rxSensID == mySensorID && rxRoutID == targetRouterID) {
          uint8_t cycleHi = LoRa.read();
          uint8_t cycleLo = LoRa.read();
          
          timeSlotSeconds = (cycleHi << 8) | cycleLo; // Ghép 2 byte thời gian
          myTdmaSlot = LoRa.read();
          
          isConfigured = true;
          ackReceived = true;
          break; // Thoát vòng lặp lắng nghe
        }
      }
    }
    
    // 3. Nếu không nhận được, chờ 3 giây rồi gửi lại (chống nghẽn mạng)
    if (!ackReceived) {
      delay(3000); 
    }
  }
  
  // Thành công lấy được cấu hình -> Ép LoRa đi ngủ ngay
  LoRa.sleep();
  secondsCounter = 0; // Reset đếm giờ để bắt đầu ngủ
}

//Giai doan 2: Do dac va truyen du lieu
void processAndTransmitPhase() {
  // 1. Đánh thức MCU và cấp điện cho cảm biến
  power_all_enable();
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, HIGH);
  delay(50); // Chờ điện áp ổn định
  
  // Khởi động lại I2C và SHT30
  Wire.begin();
  sht30.begin(0x44); 
  
  float totalTemp = 0, totalHum = 0;
  int totalSoilRaw = 0;
  
  // 2. Đo đạc (SHT30 + Analog)
  for (int i = 0; i < NUMBER_OF_MEASUREMENTS; i++) {
    totalTemp += sht30.readTemperature(); 
    totalHum  += sht30.readHumidity();
    totalSoilRaw += analogRead(SOIL_PIN); 
    delay(15); 
  }
  
  // Ngắt nguồn cảm biến ngay khi đo xong để tiết kiệm pin
  digitalWrite(SENSOR_POWER_PIN, LOW);
  
  // 3. Chuẩn hóa dữ liệu
  uint8_t finalTemp = (uint8_t)(totalTemp / (float)NUMBER_OF_MEASUREMENTS);
  uint8_t finalHum  = (uint8_t)(totalHum / (float)NUMBER_OF_MEASUREMENTS);
  int avgSoilRaw    = totalSoilRaw / NUMBER_OF_MEASUREMENTS;
  uint8_t finalSoil = normalizeData(avgSoilRaw, 300, 800); 
  
  // 4. Đóng gói Frame
  uint8_t dataFrame[7];
  dataFrame[0] = HEADER_DATA;
  dataFrame[1] = mySensorID;
  dataFrame[2] = targetRouterID;
  dataFrame[3] = finalTemp;
  dataFrame[4] = finalHum;
  dataFrame[5] = finalSoil;
  dataFrame[6] = FRAME_END;
  
  // 5. Gửi dữ liệu qua LoRa
  for (int i = 0; i < 3; i++) {
    LoRa.beginPacket();
    LoRa.write(dataFrame, 7);
    LoRa.endPacket();
    delay(50); 
  }
  
  // 6. Ép LoRa về chế độ Sleep
  LoRa.sleep();
}

void setup(){
 LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) { // Tần số 433MHz
    while (1); // Treo máy nếu không tìm thấy module LoRa
  }
  
  // Thiết lập chân nguồn cảm biến
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, LOW);

  setupTimer2_RTC();
}

void loop() {
    if (!isConfigured) 
    {
      joinNetworkPhase();
    } 
    else 
    { 
      uint32_t actualWakeUpTime = timeSlotSeconds + (myTdmaSlot * SLOT_DURATION_SEC);
      if (secondsCounter >= timeSlotSeconds) {
        secondsCounter = 0; 
        processAndTransmitPhase();
      }
      enterPowerSaveSleep();
    }
}