/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __attribute__((packed)) {
    uint8_t  header;       // 1 byte: 0x03
    uint8_t  sensor_id;    // 1 byte: ID Nút cảm biến (0x24 = Nút số 36)
    uint8_t  router_id;    // 1 byte: ID Nút cha (0x01)
    int16_t  temperature;  // 2 byte: Nhiệt độ x100
    uint16_t humidity;     // 2 byte: Độ ẩm x100
    uint8_t  soil_moisture;// 1 byte: Độ ẩm đất
} LoRa_Data_Frame_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
#define MY_ROUTER_ID 0x01          // Định danh của Nút Router này

LoRa_Data_Frame_t rxFrame;         // Biến lưu trữ gói tin vừa bắt được
volatile uint8_t txDoneFlag = 0;   // Cờ báo hiệu phát xong
volatile uint8_t rxDoneFlag = 0;   // Cờ báo hiệu nhận xong
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ====================================================================
// DRIVER DHT22 (SỬ DỤNG BỘ ĐẾM DWT)
// ====================================================================
#define DHT_PORT GPIOB
#define DHT_PIN  GPIO_PIN_12

void DWT_Delay_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us) {
    uint32_t startTick = DWT->CYCCNT;
    uint32_t delayTicks = us * (SystemCoreClock / 1000000);
    while (DWT->CYCCNT - startTick < delayTicks);
}

void DHT22_Set_Output(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT_PORT, &GPIO_InitStruct);
}

void DHT22_Set_Input(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT_PORT, &GPIO_InitStruct);
}

uint8_t DHT22_Read_Data(int16_t *temperature, uint16_t *humidity) {
    uint8_t data[5] = {0}, timeout = 0;
    DHT22_Set_Output();
    HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_RESET); delay_us(1200);
    HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_SET);   delay_us(30);
    DHT22_Set_Input();

    while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_SET) { if(++timeout > 100) return 0; delay_us(1); }
    timeout=0;
    while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_RESET) { if(++timeout > 100) return 0; delay_us(1); }
    timeout=0;
    while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_SET) { if(++timeout > 100) return 0; delay_us(1); }

    for (uint8_t i = 0; i < 5; i++) {
        for (uint8_t j = 0; j < 8; j++) {
            timeout = 0;
            while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_RESET) { if(++timeout > 100) return 0; delay_us(1); }
            delay_us(40);
            if (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_SET) {
                data[i] = (data[i] << 1) | 0x01; timeout = 0;
                while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_SET) { if(++timeout > 100) return 0; delay_us(1); }
            } else {
                data[i] = (data[i] << 1);
            }
        }
    }
    if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) == data[4]) {
        *humidity = (data[0] << 8) | data[1];
        *temperature = (data[2] << 8) | data[3];
        return 1;
    }
    return 0;
}

// ====================================================================
// DRIVER LORA SX1278 (433.5MHz, SF8, BW 125kHz)
// ====================================================================
#define SX1278_NSS_LOW()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define SX1278_NSS_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define SX1278_RST_LOW()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET)
#define SX1278_RST_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET)

void SX1278_WriteReg(uint8_t regAddr, uint8_t data) {
    SX1278_NSS_LOW();
    uint8_t txAddr = regAddr | 0x80;
    HAL_SPI_Transmit(&hspi1, &txAddr, 1, 100);
    HAL_SPI_Transmit(&hspi1, &data, 1, 100);
    SX1278_NSS_HIGH();
}

uint8_t SX1278_ReadReg(uint8_t regAddr) {
    uint8_t rxData = 0, txAddr = regAddr & 0x7F;
    SX1278_NSS_LOW();
    HAL_SPI_Transmit(&hspi1, &txAddr, 1, 100);
    HAL_SPI_Receive(&hspi1, &rxData, 1, 100);
    SX1278_NSS_HIGH();
    return rxData;
}

void SX1278_Init(void) {
    SX1278_RST_LOW(); HAL_Delay(10); SX1278_RST_HIGH(); HAL_Delay(10);
    SX1278_WriteReg(0x01, 0x80 | 0x00); HAL_Delay(10);
    SX1278_WriteReg(0x06, 0x6C);
    SX1278_WriteReg(0x07, 0x60);
    SX1278_WriteReg(0x08, 0x00);
    SX1278_WriteReg(0x1D, 0x72);
    SX1278_WriteReg(0x1E, 0x84);
    SX1278_WriteReg(0x39, 0x12);
    SX1278_WriteReg(0x01, 0x80 | 0x01);
}

void SX1278_Transmit(LoRa_Data_Frame_t *frame) {
    uint8_t payloadSize = sizeof(LoRa_Data_Frame_t);
    SX1278_WriteReg(0x01, 0x80 | 0x01);
    SX1278_WriteReg(0x40, 0x40);
    SX1278_WriteReg(0x0D, SX1278_ReadReg(0x0E));
    SX1278_WriteReg(0x22, payloadSize);
    uint8_t *dataPtr = (uint8_t *)frame;
    for (uint8_t i = 0; i < payloadSize; i++) SX1278_WriteReg(0x00, dataPtr[i]);
    SX1278_WriteReg(0x01, 0x80 | 0x03);
} // <--- Dấu ngoặc nhọn cực kỳ quan trọng để tách biệt các hàm!

// --- HÀM CHUYỂN MODULE SANG LẮNG NGHE LIÊN TỤC (RX CONTINUOUS) ---
void SX1278_StartReceive(void) {
    SX1278_WriteReg(0x01, 0x80 | 0x01);
    SX1278_WriteReg(0x40, 0x00);
    SX1278_WriteReg(0x01, 0x80 | 0x05);
}

// --- HÀM RÚT DỮ LIỆU TỪ BỘ ĐỆM SAU KHI NHẬN THÀNH CÔNG ---
void SX1278_ReadPayload(LoRa_Data_Frame_t *frame) {
    uint8_t rxBytes = SX1278_ReadReg(0x13);
    uint8_t rxAddr = SX1278_ReadReg(0x10);

    SX1278_WriteReg(0x0D, rxAddr);

    uint8_t *dataPtr = (uint8_t *)frame;
    for (uint8_t i = 0; i < rxBytes; i++) {
        dataPtr[i] = SX1278_ReadReg(0x00);
    }
}
/* USER CODE END 0 */
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
    DWT_Delay_Init();
    SX1278_Init();
    SX1278_StartReceive(); // Kích hoạt bộ thu sóng
    /* USER CODE END 2 */

  /* Infinite loop */
    /* USER CODE BEGIN WHILE */
      while (1)
      {
          // KHI CÓ SÓNG LORA BAY TỚI
          if (rxDoneFlag == 1) {
              rxDoneFlag = 0; // Hạ cờ để bắt đầu xử lý

              // 1. Kéo dữ liệu vào biến rxFrame
              SX1278_ReadPayload(&rxFrame);

              // 2. BỘ LỌC ĐỊA CHỈ: Chỉ nhận gói tin đúng chuẩn của mảng Cảm biến và đúng ID Router này
              if (rxFrame.header == 0x03 && rxFrame.router_id == MY_ROUTER_ID) {

                  // Trễ nhỏ 150ms để tránh đụng độ sóng vô tuyến với Nút Cảm Biến vừa phát
                  HAL_Delay(150);

                  // 3. ĐỊNH TUYẾN: Đổi địa chỉ đích để bắn về Gateway (ID 0x00)
                  rxFrame.router_id = 0x00;

                  // 4. CHUYỂN TIẾP (FORWARD)
                  txDoneFlag = 0;
                  SX1278_Transmit(&rxFrame);

                  while (txDoneFlag == 0) {
                      __NOP(); // CPU chờ trong trạng thái nghỉ nhẹ
                  }
              }

              // 5. Quay trở lại chế độ lắng nghe ngay lập tức
              SX1278_StartReceive();
          }

      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
      }
      /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_1) {
        uint8_t irqFlags = SX1278_ReadReg(0x12); // Đọc thanh ghi trạng thái

        // NẾU LÀ SỰ KIỆN PHÁT XONG (Bit 3)
        if ((irqFlags & 0x08) != 0) {
            txDoneFlag = 1;
        }

        // NẾU LÀ SỰ KIỆN NHẬN XONG (Bit 6)
        if ((irqFlags & 0x40) != 0) {
            rxDoneFlag = 1;
        }

        SX1278_WriteReg(0x12, 0xFF); // Xóa toàn bộ cờ ngắt
    }
}
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
