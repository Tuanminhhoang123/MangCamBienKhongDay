#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include "LowPower.h"

// Cấu hình chân SPI & Ngắt cho LoRa SX1278 trên mạch ATmega328P
#define LORA_SS    10  
#define LORA_RST   4   
#define LORA_DIO0  2   

#define SOIL_EN_PIN   3   // Chân kích nguồn cảm biến đất (PD3)
#define SOIL_ADC_PIN  A0  // Chân đọc Analog độ ẩm đất (PC0)

Adafruit_SHT31 sht30 = Adafruit_SHT31();

#pragma pack(push, 1)
typedef struct {
    uint8_t  header;       // 0x03
    uint8_t  sensor_id;    // ID nút con này (Ví dụ: 0x24)
    uint8_t  router_id;    // Gửi đích danh cho Nút Lai trung gian (0x01)
    int16_t  temperature;  // Nhiệt độ x 100
    uint16_t humidity;     // Độ ẩm khí x 100
    uint8_t  soil_moisture;// % Độ ẩm đất
} LoRa_Data_Frame_t;
#pragma pack(pop)

LoRa_Data_Frame_t sensorData;
volatile bool txDone = false;

void onTxDone() {
    txDone = true; 
}

void setup() {
    ADCSRA = 0; // Tắt bộ ADC của chip lúc khởi động để giảm dòng tiêu thụ
    pinMode(SOIL_EN_PIN, OUTPUT);
    digitalWrite(SOIL_EN_PIN, LOW); 

    Wire.begin();
    sht30.begin(0x44); 

    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    if (!LoRa.begin(4335E5)) { while (1); } // Treo nếu lỗi phần cứng LoRa
    
    LoRa.setSpreadingFactor(8);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setSyncWord(0x12); 
    LoRa.onTxDone(onTxDone);
}

void loop() {
    // 1. CẤP ĐIỆN VÀ ĐỌC CẢM BIẾN ĐẤT
    digitalWrite(SOIL_EN_PIN, HIGH);
    delay(20); 
    ADCSRA |= (1 << ADEN); // Bật bộ ADC lên để đọc
    delay(5);
    int soil_raw = analogRead(SOIL_ADC_PIN);
    ADCSRA = 0; // Đọc xong tắt ngay bộ ADC
    digitalWrite(SOIL_EN_PIN, LOW); // Ngắt điện cảm biến đất để tiết kiệm pin

    // ĐỌC CẢM BIẾN KHÔNG KHÍ SHT30
    float t = sht30.readTemperature();
    float h = sht30.readHumidity();

    // 2. ĐÓNG GÓI DỮ LIỆU
    sensorData.header = 0x03;
    sensorData.sensor_id = 0x24; // Số 36 mã Hex
    sensorData.router_id = 0x01; // Gửi cho ông Nút Lai
    
    if (!isnan(t)) sensorData.temperature = (int16_t)(t * 100);
    if (!isnan(h)) sensorData.humidity = (uint16_t)(h * 100);
    sensorData.soil_moisture = map(soil_raw, 1023, 0, 0, 100); 

    // 3. PHÁT SÓNG
    txDone = false;
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&sensorData, sizeof(sensorData));
    LoRa.endPacket(true); 

    while (!txDone) {
        LowPower.idle(SLEEP_15MS, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_ON, USART0_OFF, TWI_OFF);
    }
    LoRa.sleep(); // Ép IC LoRa ngủ sâu

    // 4. ĐƯA VI ĐIỀU KHIỂN NGỦ SÂU 15 PHÚT (Vòng lặp 112 lần x 8 giây)
    for (int i = 0; i < 112; i++) {
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    }
}