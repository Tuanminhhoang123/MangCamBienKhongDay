#include <Arduino.h>
#include "driver/spi_master.h"
#include <LoRa.h> 
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" 
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"


// ==========================================
// 1. ĐỊNH NGHĨA CẤU TRÚC DỮ LIỆU DÙNG CHUNG
// ==========================================
typedef struct {
    uint8_t  node_id;        // Định danh nút cảm biến (1 ÷ 100)
    float    temperature;    // Nhiệt độ môi trường (°C)
    float    air_humidity;   // Độ ẩm không khí (%RH)
    float    soil_humidity;  // Độ ẩm đất (%Vol)
} SensorData_t;

// Kết quả phân tích tưới tiêu
typedef struct {
    bool  need_irrigation;   // true = cần tưới
    int   duration_seconds;  // Thời gian tưới đề xuất (giây)
    char  reason[128];       // Lý do quyết định (để log/debug)
} IrrigationDecision_t;
// Trạng thái máy bơm toàn cục
static bool is_pump_on = false;

// Khai báo Handle quản lý Hàng đợi liên lõi
QueueHandle_t xLoRaQueue = NULL;

// ==========================================
// 2. ĐỊNH NGHĨA PHẦN CỨNG SƠ ĐỒ CHÂN SPI (ESP32)
// ==========================================
#define LORA_SCK_PIN    18
#define LORA_MISO_PIN   19
#define LORA_MOSI_PIN   23
#define LORA_CS_PIN     5    // Chân chọn chip NSS / CS
#define LORA_RST_PIN    14   // Chân Reset cứng của SX1278
#define LORA_DIO0_PIN   2    // Chân ngắt báo tín hiệu IRQ từ SX1278

// ==========================================
// 3. TASK 1: XỬ LÝ LORA (CHẠY TRÊN CORE 0)
// ==========================================
void vLoRaTask(void *pvParameters) {
    Serial.printf("[Core %d] LoRa Task đang khởi tạo phần cứng...\n", xPortGetCoreID());
    
    // --- GIAI ĐOẠN KHỞI TẠO CẤU HÌNH LORA (CHẠY 1 LẦN) ---
    // 1. Chỉ định cấu hình chân kết nối vật lý cho thư viện Sandeep Mistry
    LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
    
    // 2. Kích hoạt module hoạt động ở tần số ISM 433 MHz
    if (!LoRa.begin(433E6)) {
        Serial.println("[Core 0] Lỗi nghiêm trọng: Không tìm thấy mạch LoRa SX1278 qua Bus SPI!");
        vTaskDelete(NULL); // Tự hủy Task nếu lỗi phần cứng để tránh crash hệ thống
    }
    
    // 3. Cấu hình các thông số truyền dẫn vô tuyến (RF Tuning) theo đồ án
    LoRa.setSpreadingFactor(7);     // Hệ số trải phổ SF7 (Tối ưu tốc độ và khoảng cách)
    LoRa.setSignalBandwidth(125E3); // Băng thông chuẩn mạng 125kHz
    LoRa.setCodingRate4(5);          // Tỷ lệ sửa lỗi hình học CR 4/5 chống nhiễu không trung
    LoRa.setSyncWord(0xF3);         // Mã định danh mạng độc quyền (Nodes & Gateway bắt buộc trùng mã này)
    LoRa.setTxPower(20);            // Đẩy công suất tối đa 20dBm để phủ sóng phạm vi rộng
    
    Serial.println("[Core 0] Khởi tạo & Cấu hình mạng LoRa 433MHz thành công!");
    
    // Khai báo biến tạm phục vụ xử lý
    SensorData_t sensorPacket;

    // --- GIAI ĐOẠN VÒNG LẶP KIỂM TRA ĐỒNG BỘ (SUPERLOOP ĐA TÁC VỤ) ---
    while (1) {
        // Kiểm tra dung lượng byte của gói tin rơi vào bộ đệm của mạch SX1278
        int packetSize = LoRa.parsePacket();
        
        // Nếu parsePacket trả về giá trị > 0 nghĩa là có gói tin mới từ các Node đẩy về
        if (packetSize > 0) {
            Serial.printf("[Core 0] Phát hiện gói tin mới! Kích thước: %d byte\n", packetSize);
            
            // Đọc byte đầu tiên làm ID định danh của Nút (Node_ID)
            if (LoRa.available()) {
                sensorPacket.node_id = LoRa.read();
            }
            
            // Xử lý đọc luồng Byte thô tiếp theo để giải mã ngược ra các giá trị số thực
            // Cách làm này giúp nén dung lượng gói tin vô tuyến nhỏ hơn rất nhiều so với gửi dạng chuỗi Text
            if (LoRa.available() >= 6) { // Đảm bảo còn đủ 6 byte cho 3 biến float dữ liệu
                
                // Giải mã biến Nhiệt độ (2 byte)
                int16_t raw_temp = (LoRa.read() << 8) | LoRa.read();
                sensorPacket.temperature = (float)raw_temp / 10.0;
                
                // Giải mã biến Độ ẩm không khí (2 byte)
                uint16_t raw_air_hum = (LoRa.read() << 8) | LoRa.read();
                sensorPacket.air_humidity = (float)raw_air_hum / 10.0;
                
                // Giải mã biến Độ ẩm đất (2 byte)
                uint16_t raw_soil_hum = (LoRa.read() << 8) | LoRa.read();
                sensorPacket.soil_humidity = (float)raw_soil_hum / 10.0;
                
                Serial.printf("[Core 0] Giải mã thành công từ Node %d -> Temp: %.1f°C, Air: %.1f%%, Soil: %.1f%%\n", 
                              sensorPacket.node_id, sensorPacket.temperature, sensorPacket.air_humidity, sensorPacket.soil_humidity);
                
                // --- ĐẨY DỮ LIỆU SANG CORE 1 QUA QUEUE ---
                // Sử dụng hàm an toàn bộ nhớ xQueueSendToBack của FreeRTOS
                // Thời gian chờ tối đa nếu Queue bị đầy là 10 mili-giây
                if (xQueueSendToBack(xLoRaQueue, &sensorPacket, pdMS_TO_TICKS(10)) == pdTRUE) {
                    Serial.println("[Core 0] Đã đẩy cấu trúc Struct vào hàng đợi RAM liên lõi.");
                } else {
                    Serial.println("[Core 0] Lỗi! Hàng đợi xLoRaQueue bị tràn, Core 1 xử lý Internet không kịp!");
                }
            }
        }

        // LỆNH GIẢI PHÓNG XUNG NHỊP CỰC KỲ QUAN TRỌNG TRONG FREERTOS
        // Delay 1 mili-giây để Watchdog Timer hệ thống không bị kích hoạt lỗi treo chip,
        // đồng thời giải phóng quyền ưu tiên cho các tác vụ nền (IDLE Task) hoạt động.
        vTaskDelay(pdMS_TO_TICKS(1));
    }

}

// ==========================================
// 4. THÔNG SỐ CẤU HÌNH HẠ TẦNG TRUYỀN THÔNG
// ==========================================
#define FARM_WIFI_SSID      "Farm_WiFi_Station"   // SSID của Wi-Fi AP nông trại
#define FARM_WIFI_PASS      "FarmPassword123"     // Mật khẩu Wi-Fi AP
#define CLOUD_MQTT_URL      "mqtt://192.168.1.100"// Địa chỉ IP/Domain của MQTT Broker Cloud
#define MQTT_PUB_TOPIC      "farm/field/sensors"  // Topic phân phối chuỗi JSON lên mây

static const char *TAG = "GATEWAY_CLOUD_CORE1";

// ==========================================
// 5. NGƯỠNG PHÂN TÍCH DỰ BÁO TƯỚI TIÊU
// ==========================================
#define SOIL_HUM_LOW        35.0f   // Độ ẩm đất thấp → cần tưới (%)
#define SOIL_HUM_HIGH       70.0f   // Độ ẩm đất cao  → dừng tưới (%)
#define TEMP_HIGH           35.0f   // Nhiệt độ cao   → tăng nhu cầu tưới (°C)
#define AIR_HUM_LOW         40.0f   // Độ ẩm không khí thấp → tăng nhu cầu tưới (%)

#define MQTT_PUB_PUMP_TOPIC "farm/control/pump"  // Topic phát lệnh điều khiển máy bơm

// HÀM PHÂN TÍCH NGƯỠNG VÀ ĐƯA RA QUYẾT ĐỊNH TƯỚI
IrrigationDecision_t analyze_irrigation(const SensorData_t *data) {
    IrrigationDecision_t decision;
    memset(&decision, 0, sizeof(decision));

    // TRƯỜNG HỢP 1: Đất quá khô → tưới bắt buộc
    if (data->soil_humidity < SOIL_HUM_LOW) {

        decision.need_irrigation = true;
        decision.duration_seconds = 120; // Mặc định tưới 2 phút

        // Nếu thêm nhiệt độ cao + không khí khô → tưới lâu hơn
        if (data->temperature > TEMP_HIGH && data->air_humidity < AIR_HUM_LOW) {
            decision.duration_seconds = 300; // Tưới 5 phút
            snprintf(decision.reason, sizeof(decision.reason),
                "Đất khô (%.1f%%) + Nhiệt cao (%.1f°C) + Không khí khô (%.1f%%) → Tưới 5 phút",
                data->soil_humidity, data->temperature, data->air_humidity);

        } else if (data->temperature > TEMP_HIGH) {
            decision.duration_seconds = 240; // Tưới 4 phút
            snprintf(decision.reason, sizeof(decision.reason),
                "Đất khô (%.1f%%) + Nhiệt cao (%.1f°C) → Tưới 4 phút",
                data->soil_humidity, data->temperature);

        } else {
            snprintf(decision.reason, sizeof(decision.reason),
                "Đất khô (%.1f%%) → Tưới 2 phút",
                data->soil_humidity);
        }

    // TRƯỜNG HỢP 2: Đất đủ ẩm → không tưới
    } else if (data->soil_humidity >= SOIL_HUM_HIGH) {
        decision.need_irrigation = false;
        snprintf(decision.reason, sizeof(decision.reason),
            "Đất đủ ẩm (%.1f%%) → Không cần tưới",
            data->soil_humidity);

    // TRƯỜNG HỢP 3: Đất ở ngưỡng trung gian → xét thêm điều kiện
    } else {
        if (data->temperature > TEMP_HIGH && data->air_humidity < AIR_HUM_LOW) {
            // Nhiệt cao + khô → tưới nhẹ dù đất chưa quá khô
            decision.need_irrigation = true;
            decision.duration_seconds = 60; // Tưới 1 phút
            snprintf(decision.reason, sizeof(decision.reason),
                "Đất trung bình (%.1f%%) nhưng Nhiệt cao + Không khí khô → Tưới nhẹ 1 phút",
                data->soil_humidity);
        } else {
            decision.need_irrigation = false;
            snprintf(decision.reason, sizeof(decision.reason),
                "Đất trung bình (%.1f%%) điều kiện bình thường → Chưa cần tưới",
                data->soil_humidity);
        }
    }

    return decision;
}

// Định nghĩa cờ sự kiện Wi-Fi
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool is_mqtt_connected = false;

// ==========================================
// 6. TASK 2: XỬ LÝ CLOUD (CHẠY TRÊN CORE 1)
// ==========================================
// 6.1. TRÌNH XỬ LÝ SỰ KIỆN PHẦN CỨNG WI-FI & CẤP PHÁT IP (CALLBACK EVENT)
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect(); // Antenna phát lệnh kết nối vật lý với Wi-Fi AP
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        is_mqtt_connected = false;
        ESP_LOGW(TAG, "Mất sóng Wi-Fi AP nông trại! Đang tự động kết nối lại vô hạn...");
        esp_wifi_connect(); // Thiết lập giải thuật tự động hồi phục kết nối tầng vật lý
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Đã bắt tay mạng thành công. IP nội bộ: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT); // Bật cờ cho phép chạy luồng cao hơn
    }
}

// 6.2. TRÌNH XỬ LÝ SỰ KIỆN GIAO THỨC MQTT TẦNG ỨNG DỤNG (CALLBACK EVENT)
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, 
                               int32_t event_id, void *event_data) {
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Kết nối thành công! Đã thông suốt đường truyền với Cloud MQTT Broker.");
            is_mqtt_connected = true; // Dựng cờ báo trạng thái sẵn sàng phát tin
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "Cảnh báo: Mất liên lạc kết nối với Cloud MQTT Broker!");
            is_mqtt_connected = false; // Hạ cờ ngăn chặn Core 1 bốc dữ liệu khỏi Queue vô ích
            break;
        case MQTT_EVENT_DATA:{
            // event để đọc payload nhận về
            esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
            ESP_LOGI(TAG, "Nhận dữ liệu MQTT - Topic: %.*s, Data: %.*s",
                     event->topic_len, event->topic,
                     event->data_len, event->data);
            break;           
        }
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "Lỗi MQTT! Kiểm tra kết nối Broker.");
            break;
            
        // Gộp tất cả event còn lại không cần xử lý vào đây
        case MQTT_EVENT_BEFORE_CONNECT:
        case MQTT_EVENT_DELETED:
        case MQTT_EVENT_PUBLISHED:
        case MQTT_EVENT_SUBSCRIBED:
        case MQTT_EVENT_UNSUBSCRIBED:
        case MQTT_EVENT_ANY:       
        default:
            break;
    }
}

// 6.3. CHƯƠNG TRÌNH CON KHỞI TẠO TẬP TRUNG TẦNG TRUYỀN THÔNG (CHẠY 1 LẦN)
void gateway_network_init(void) {
    // Khởi tạo phân vùng lưu trữ token phần cứng trong chip ESP32
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    s_wifi_event_group = xEventGroupCreate();

    // Khởi tạo nhân lõi TCP/IP Stack (LwIP Core)
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta(); // Khởi tạo Driver trạm thu STA

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Đăng ký các hàm phục vụ ngắt sự kiện mạng cho hệ thống điều phối
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    //  Cấu hình WiFi an toàn (tránh lỗi struct)
    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));

    // Copy dữ liệu an toàn dùng strncpy
    strncpy((char*)wifi_config.sta.ssid, FARM_WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, FARM_WIFI_PASS, sizeof(wifi_config.sta.password) - 1);

    //Khai báo các trường bổ sung để tránh lỗi thiếu thông số
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start()); // Kích hoạt khối phát sóng Antenna

    ESP_LOGI(TAG, "Đang quét dải tần dò tìm Wi-Fi AP nông trại...");
    // Đóng băng tiến trình tối đa 10 giây để chờ nhận IP ổn định từ Router Wi-Fi
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(10000));

    // --- CẤU HÌNH THỰC THỂ MQTT CLIENT ---
    esp_mqtt_client_config_t mqtt_cfg;
    memset(&mqtt_cfg, 0, sizeof(mqtt_cfg));
    mqtt_cfg.uri = CLOUD_MQTT_URL;
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client); // Kích hoạt luồng chạy ngắt giữ nhịp (Keep-Alive) MQTT Broker
}

// 6.4. LUỒNG TÁC VỤ CHÍNH
void vCloudTask(void *pvParameters) {
    printf("Cloud Task đang khởi động trên Core: %d\n", xPortGetCoreID());
    
    gateway_network_init();

    SensorData_t rxData;
    char mqtt_payload[256];

    while (1) {
        // KIỂM TRA MẠNG
        if (!is_mqtt_connected) {
        ESP_LOGW(TAG, "[Core 1] MQTT chưa sẵn sàng, chờ kết nối...");
        vTaskDelay(pdMS_TO_TICKS(2000)); // Chờ 2 giây rồi kiểm tra lại
        continue;
        }
        // ĐỌC HÀNG ĐỢI: Core 1 rơi vào trạng thái ngủ chặn (Blocked) tuyệt đối nếu Queue trống.
        // Tham số pdMS_TO_TICKS(5000):  Kiểm tra mạng định kỳ (Cứ 5 giây thoát ra 1 lần)
        if (xQueueReceive(xLoRaQueue, &rxData, pdMS_TO_TICKS(5000)) == pdTRUE) {
            
            // Thực hiện chuyển đổi biến số Struct thành định dạng chuỗi văn bản JSON payload
            snprintf(mqtt_payload, sizeof(mqtt_payload),
                     "{\"node_id\":%d,\"temp\":%.1f,\"air_hum\":%.1f,\"soil_hum\":%.1f}",
                     rxData.node_id, rxData.temperature, rxData.air_humidity, rxData.soil_humidity);

            // Bắn gói tin MQTT JSON lên Cloud Server với chất lượng dịch vụ QoS 1
            // Cấp độ QoS 1 bắt buộc có phản hồi PUBACK, đảm bảo tin nhắn không bị rơi rụng
            int message_packet_id = esp_mqtt_client_publish(mqtt_client, MQTT_PUB_TOPIC, mqtt_payload, 0, 1, 0);

            if (message_packet_id >= 0) {
                printf("[Core 1] Publish MQTT QoS 1 thành công. Gói tin ID: %d\n", message_packet_id);
            } else {
                printf("[Core 1] Lỗi! Không thể đẩy gói tin lên mạng Internet\n");
            }

            // PHÂN TÍCH DỰ BÁO TƯỚI TIÊU
            IrrigationDecision_t decision = analyze_irrigation(&rxData);
            ESP_LOGI(TAG, "[TƯỚI TIÊU] Node %d → %s", rxData.node_id, decision.reason);

            // Xử lý lệnh BẬT máy bơm
            if (decision.need_irrigation && !is_pump_on) {
                is_pump_on = true;

                char pump_payload[128];
                snprintf(pump_payload, sizeof(pump_payload),
                        "{\"pump\":1,\"duration\":%d,\"node_id\":%d}",
                        decision.duration_seconds, rxData.node_id);

                int pump_packet_id = esp_mqtt_client_publish(mqtt_client, MQTT_PUB_PUMP_TOPIC, pump_payload, 0, 1, 0);
                if (pump_packet_id >= 0) {
                    ESP_LOGI(TAG, "[BƠM] BẬT máy bơm %d giây thành công. Gói tin ID: %d", 
                            decision.duration_seconds, pump_packet_id);
                } else {
                    ESP_LOGE(TAG, "[BƠM] Lỗi! Không thể gửi lệnh BẬT bơm lên Broker");
                }

            // Xử lý lệnh TẮT máy bơm
            } else if (!decision.need_irrigation && is_pump_on) {
                is_pump_on = false;

                int pump_packet_id = esp_mqtt_client_publish(mqtt_client, MQTT_PUB_PUMP_TOPIC,
                                                            "{\"pump\":0}", 0, 1, 0);
                if (pump_packet_id >= 0) {
                    ESP_LOGI(TAG, "[BƠM] TẮT máy bơm thành công. Gói tin ID: %d", pump_packet_id);
                } else {
                    ESP_LOGE(TAG, "[BƠM] Lỗi! Không thể gửi lệnh TẮT bơm lên Broker");
                }

            // Trạng thái bơm không đổi
            } else {
                ESP_LOGI(TAG, "[BƠM] Giữ nguyên trạng thái: %s", is_pump_on ? "ĐANG BẬT" : "ĐANG TẮT");
            }
        }
    }
}

// ==========================================
// 7. HÀM KHỞI TẠO CHÍNH (MAIN ENTRY POINT)
// ==========================================
extern "C" void app_main(void) {
        Serial.begin(115200);
        Serial.println("=== HỆ THỐNG TRẠM GATEWAY TRUNG TÂM ESP32 ===");

        // Khởi tạo FreeRTOS Queue lưu trữ tối đa 10 gói dữ liệu cảm biến Struct
        xLoRaQueue = xQueueCreate(10, sizeof(SensorData_t));

        if (xLoRaQueue != NULL) {
        printf("Khởi tạo hàng đợi xLoRaQueue thành công!\n");

        // ÉP CHẠY SONG SONG TRÊN LÕI KÉP (Dual-Core Pinning)
        // Hàm xTaskCreatePinnedToCore() chỉ định đích danh lõi vật lý thực thi tác vụ:
        
        // Khởi chạy vLoRaTask trên CORE 0 (Độ ưu tiên 5, bộ nhớ Stack 4096 từ lực)
        xTaskCreatePinnedToCore(vLoRaTask, "LoRa_Task", 4096, NULL, 5, NULL, 0);

        // Khởi chạy vCloudTask trên CORE 1 (Độ ưu tiên 3, bộ nhớ Stack 8192 từ lực gánh Stack mạng)
        xTaskCreatePinnedToCore(vCloudTask, "Cloud_Task", 8192, NULL, 3, NULL, 1);
        
        } else {
            printf("Lỗi nghiêm trọng: Không thể cấp phát bộ nhớ RAM cho Hàng đợi!\n");
        }
    
}
