#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>

// --- KHAI BÁO CHÂN PHẦN CỨNG (Mạch ATmega328P) ---
#define LORA_SS    10
#define LORA_RST   4
#define LORA_DIO0  2

#define SOIL_EN_PIN   3   // Chân kích nguồn cảm biến đất
#define SOIL_ADC_PIN  A0  // Chân đọc Analog độ ẩm đất

#define MY_ROUTER_ID  0x01 // Định danh của Nút Lai này
#define GATEWAY_ID    0x00 // Định danh Trạm trung tâm ESP32

Adafruit_SHT31 sht30 = Adafruit_SHT31();

// --- CẤU TRÚC GÓI TIN 8-BYTE ---
#pragma pack(push, 1)
typedef struct {
    uint8_t  header;       
    uint8_t  sensor_id;    
    uint8_t  router_id;    
    int16_t  temperature;  
    uint16_t humidity;     
    uint8_t  soil_moisture;
} LoRa_Data_Frame_t;
#pragma pack(pop)

LoRa_Data_Frame_t frameData;

// Biến quản lý thời gian không dùng hàm Delay chặn luồng
unsigned long previousMillis = 0;
const unsigned long interval = 900000; // Chu kỳ tự đo: 15 phút (900.000 mili-giây)

void setup() {
    Serial.begin(9600);
    
    // Khởi tạo cảm biến
    pinMode(SOIL_EN_PIN, OUTPUT);
    digitalWrite(SOIL_EN_PIN, LOW); 
    Wire.begin();
    sht30.begin(0x44); 

    // Khởi tạo LoRa
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    if (!LoRa.begin(4335E5)) {
        Serial.println("Loi: Khong tim thay module LoRa!");
        while (1); 
    }
    
    // Cấu hình RF đồng bộ toàn mạng
    LoRa.setSpreadingFactor(8);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setSyncWord(0x12); 

    // Bắt buộc đưa LoRa vào chế độ Lắng nghe liên tục (Không ngủ)
    LoRa.receive(); 
}

void loop() {
    // =========================================================
    // NHIỆM VỤ 1: LÀM ROUTER (LẮNG NGHE & CHUYỂN TIẾP LIÊN TỤC)
    // =========================================================
    int packetSize = LoRa.parsePacket();
    if (packetSize == sizeof(LoRa_Data_Frame_t)) {
        LoRa.readBytes((uint8_t*)&frameData, packetSize);

        // Lọc sóng: Chỉ nhận gói tin nếu có Nút con gọi đích danh Nút Lai này
        if (frameData.header == 0x03 && frameData.router_id == MY_ROUTER_ID) {
            Serial.println("Da nhan duoc goi tin tu Nut con! Dang chuyen tiep...");
            
            // Đổi địa chỉ đích của gói tin thành Gateway
            frameData.router_id = GATEWAY_ID; 
            
            // Bắn sóng chuyển tiếp đi
            LoRa.beginPacket();
            LoRa.write((uint8_t*)&frameData, sizeof(frameData));
            LoRa.endPacket(); 

            // Gửi xong lập tức quay lại trạng thái lắng nghe để không bỏ lót sóng
            LoRa.receive(); 
        }
    }

    // =========================================================
    // NHIỆM VỤ 2: LÀM SENSOR (TỰ ĐO VÀ PHÁT THEO CHU KỲ 15 PHÚT)
    // =========================================================
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        // Bật cảm biến đất & đọc dữ liệu
        digitalWrite(SOIL_EN_PIN, HIGH);
        delay(25); 
        int soil_raw = analogRead(SOIL_ADC_PIN);
        digitalWrite(SOIL_EN_PIN, LOW); 

        // Đọc cảm biến không khí SHT30
        float t = sht30.readTemperature();
        float h = sht30.readHumidity();

        // Đóng gói dữ liệu của CHÍNH NÚT NÀY
        frameData.header = 0x03;
        frameData.sensor_id = MY_ROUTER_ID; // Tự xưng ID là Nút Lai (0x01)
        frameData.router_id = GATEWAY_ID;   // Bắn thẳng lên Gateway (0x00)
        
        if (!isnan(t)) frameData.temperature = (int16_t)(t * 100);
        if (!isnan(h)) frameData.humidity = (uint16_t)(h * 100);
        frameData.soil_moisture = map(soil_raw, 1023, 0, 0, 100); 

        // Bắn dữ liệu đi
        Serial.println("Dang gui du lieu moi truong cua chinh minh...");
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&frameData, sizeof(frameData));
        LoRa.endPacket();

        // Xong việc tự đo, tiếp tục đưa Anten về trạng thái nghe ngóng
        LoRa.receive(); 
    }
}