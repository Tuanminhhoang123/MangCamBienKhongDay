#include <SPI.h>
#include <LoRa.h>

// --- KHAI BÁO CHÂN PHẦN CỨNG (Dành riêng cho ESP32 WROOM 32) ---
// Các chân này được map theo chuẩn VSPI của ESP32
#define LORA_SS    5   // Chân IO5
#define LORA_RST   14  // Chân IO14
#define LORA_DIO0  26  // Chân IO26

#define MY_GATEWAY_ID 0x00 // Định danh của Trạm Trung Tâm

// --- CẤU TRÚC GÓI TIN 8-BYTE (Phải khớp 100% với các nút dưới) ---
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

LoRa_Data_Frame_t rxFrame;

void setup() {
    // Khởi tạo cổng giao tiếp UART với máy tính ở tốc độ 115200 baud
    Serial.begin(115200); 
    while (!Serial); // Chờ cổng Serial ổn định

    Serial.println("Khoi dong He thong Gateway LoRa ESP32...");

    // Cấu hình chân và khởi tạo module LoRa
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    if (!LoRa.begin(4335E5)) {
        Serial.println("Loi: Khong tim thay chip LoRa SX1278 tren ESP32!");
        while (1); // Treo hệ thống nếu lỗi phần cứng
    }
    
    // Cấu hình RF phải đồng bộ với các nút phát
    LoRa.setSpreadingFactor(8);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setSyncWord(0x12);

    Serial.println("--- GATEWAY DA SAN SANG NHAN DU LIEU ---");
}

void loop() {
    // Kiểm tra liên tục xem có sóng vô tuyến bay tới không
    int packetSize = LoRa.parsePacket();
    
    // Nếu có gói tin và kích thước đúng bằng 8 bytes của struct
    if (packetSize == sizeof(LoRa_Data_Frame_t)) {
        
        // Rút dữ liệu từ bộ đệm LoRa nhét vào biến rxFrame
        LoRa.readBytes((uint8_t*)&rxFrame, packetSize);

        // BỘ LỌC CUỐI CÙNG: Chỉ nhận gói tin của mạng mình (0x03) và gửi đích danh cho mình (0x00)
        if (rxFrame.header == 0x03 && rxFrame.router_id == MY_GATEWAY_ID) {
            
            // Chia lại cho 100 để trả về số thực (float) cho nhiệt độ, độ ẩm
            float temp_real = rxFrame.temperature / 100.0;
            float hum_real = rxFrame.humidity / 100.0;

            // In dữ liệu ra màn hình máy tính thật đẹp và rõ ràng
            Serial.print("[LORA RX] Nhan tu Node: 0x");
            Serial.print(rxFrame.sensor_id, HEX); // In ID dưới dạng Hex (VD: 0x01 hoặc 0x24)
            Serial.print(" | Nhiet do: "); Serial.print(temp_real); Serial.print(" C");
            Serial.print(" | Do am khi: "); Serial.print(hum_real); Serial.print(" %");
            Serial.print(" | Do am dat: "); Serial.print(rxFrame.soil_moisture); Serial.println(" %");
        }
    }
}