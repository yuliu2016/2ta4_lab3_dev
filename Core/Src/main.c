/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

#include "stm32f429i_discovery_lcd.h"
#include "stdio.h"
#include "i2c_at24c64.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define COLUMN(x) ((x) * (BSP_LCD_GetFont()->Width))
#define I2c3_Handle hi2c3
#define EEPROM_ADDRESS  0xA0
// Memory location to write to in the device
#define memLocation 0x000A

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c3;

LTDC_HandleTypeDef hltdc;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi5;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */

extern sFONT Font20;
extern uint8_t *clock100_ptr;

RTC_DateTypeDef rtcDate;
RTC_TimeTypeDef rtcTime;

static const char MONTHS[12][4] ={
    "Jan", "Feb", "Mar",
    "Apr", "May", "Jun",
    "Jul", "Aug", "Sep",
    "Oct", "Nov", "Dec"};

enum {
  INIT_TEST_EEPROM,
  DISPLAY_PREV_TIMES_OFF,
  DISPLAY_PREV_TIMES_ON,
  EDIT_HOUR,
  EDIT_MINUTE,
  EDIT_SECOND,
  EDIT_MONTH,
  EDIT_DAY,
  EDIT_YEAR
} programState;

// Allows keeping track of time continuously
// This is reset to 000A on power cycle
// (Also it doesn't track overflow)
uint16_t eepromPtr = memLocation;

// Button software debouncing
uint32_t prevBtn1Tick = 0;
uint32_t prevBtn2Tick = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_DMA2D_Init(void);
static void MX_GPIO_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C3_Init(void);
static void MX_LTDC_Init(void);
static void MX_SPI5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

void LCD_DisplayString(uint16_t line, uint16_t col, uint8_t *ptr);
void LCD_DisplayInt(uint16_t line, uint16_t col, int n);
void LCD_DisplayFloat(uint16_t line, uint16_t col, float f, int digits);
void LCD_DisplayAnalogClock();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Lab3_UpdateDateAndTime() {
  // Get the time
  HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
  //https://stackoverflow.com/a/50212103
  HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);
}

uint8_t EEPROM_Read(uint16_t location) {
  return I2C_ByteRead(&hi2c3, EEPROM_ADDRESS, location);
}

void EEPROM_Write(uint16_t location, uint8_t data) {
  I2C_ByteWrite(&hi2c3, EEPROM_ADDRESS, location, data);
}

void Lab3_ClearPrevTimesDisplay() {
  BSP_LCD_ClearStringLine(1);
  BSP_LCD_ClearStringLine(2);
  BSP_LCD_ClearStringLine(3);
}

void Lab3_StoreTime() {
  Lab3_UpdateDateAndTime();
  EEPROM_Write(eepromPtr++, rtcTime.Hours);
  EEPROM_Write(eepromPtr++, rtcTime.Minutes);
  EEPROM_Write(eepromPtr++, rtcTime.Seconds);
  Lab3_ClearPrevTimesDisplay();
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  LCD_DisplayString(1, 2, (uint8_t *) "Stored: #");
  LCD_DisplayInt(1, 11, (eepromPtr - memLocation) / 3);
}

void Lab3_LoadPrevTimes() {

  Lab3_ClearPrevTimesDisplay();
  BSP_LCD_SetTextColor(LCD_COLOR_DARKCYAN);
  LCD_DisplayString(1, 2, (uint8_t *) "Last 2 times:");

  char buf[18];
  uint8_t h, m, s;

  // second-last time
  if (eepromPtr - memLocation >= 6) {
    h = EEPROM_Read(eepromPtr - 6);
    m = EEPROM_Read(eepromPtr - 5);
    s = EEPROM_Read(eepromPtr - 4);
    sprintf(buf, "%d:%02d:%02d", h, m, s);
    LCD_DisplayString(2, 3, (uint8_t *) buf);
  } else {
    // not recorded
    LCD_DisplayString(2, 3, (uint8_t *) "-------");
  }

  // last time
  if (eepromPtr - memLocation >= 3) {
    h = EEPROM_Read(eepromPtr - 3);
    m = EEPROM_Read(eepromPtr - 2);
    s = EEPROM_Read(eepromPtr - 1);
    sprintf(buf, "%d:%02d:%02d", h, m, s);
    LCD_DisplayString(3, 3, (uint8_t *) buf);
  } else {
    // not recorded
    LCD_DisplayString(3, 3, (uint8_t *) "-------");
  }
}

void Lab3_DisplayDate() {
  char buf[5];

  sprintf(buf, "%s ", MONTHS[rtcDate.Month - 1]);
  if (programState == EDIT_MONTH) {
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
  } else {
    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  }
  LCD_DisplayString(14, 3, (uint8_t *) buf);


  sprintf(buf, "%02d", rtcDate.Date);
  if (programState == EDIT_DAY) {
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
  } else {
    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  }
  LCD_DisplayString(14, 7, (uint8_t *) buf);

  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  LCD_DisplayString(14, 9, (uint8_t *) ",");

  sprintf(buf, "20%02d", rtcDate.Year);
  if (programState == EDIT_YEAR) {
     BSP_LCD_SetTextColor(LCD_COLOR_RED);
   } else {
     BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
   }
   LCD_DisplayString(14, 10, (uint8_t *) buf);
}

void Lab3_DisplayTime() {
  char buf[4];
  sprintf(buf, "%2d", rtcTime.Hours);

  if (programState == EDIT_HOUR) {
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
  } else {
    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  }
  LCD_DisplayString(13, 4, (uint8_t *) buf);

  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  LCD_DisplayString(13, 6, (uint8_t *) ":");

  sprintf(buf, "%02d", rtcTime.Minutes);
  if (programState == EDIT_MINUTE) {
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
  } else {
    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  }
  LCD_DisplayString(13, 7, (uint8_t *) buf);

  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  LCD_DisplayString(13, 9, (uint8_t *) ":");

  sprintf(buf, "%02d", rtcTime.Seconds);
  if (programState == EDIT_SECOND) {
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
  } else {
    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  }
  LCD_DisplayString(13, 10, (uint8_t *) buf);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

  if (programState == INIT_TEST_EEPROM) {
    // Disable all buttons during the EEPROM test state
    return;
  }

  if (GPIO_Pin == KEY_BUTTON_PIN) {
    Lab3_DisplayDate();
    Lab3_StoreTime();
    // Reset the state
    programState = DISPLAY_PREV_TIMES_OFF;
  }
  if (GPIO_Pin == GPIO_PIN_1) {

    // Debouncing
    uint32_t tick = HAL_GetTick();
    if (tick - prevBtn1Tick < 400) return;
    prevBtn1Tick = tick;

    // Button 1 is used to either
    // - Toggle displaying the previous times
    // - Set the time
    switch (programState) {
      case DISPLAY_PREV_TIMES_OFF:
        // show the last two recorded times
        Lab3_LoadPrevTimes();
        programState = DISPLAY_PREV_TIMES_ON;
        break;
      case DISPLAY_PREV_TIMES_ON:
        Lab3_ClearPrevTimesDisplay();
        programState = DISPLAY_PREV_TIMES_OFF;
        break;

      // The following cases each increments the respective value by 1
      case EDIT_HOUR:
        rtcTime.Hours = (rtcTime.Hours + 1) % 24;
        Lab3_DisplayTime();
        HAL_RTC_SetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
        break;
      case EDIT_MINUTE:
        rtcTime.Minutes = (rtcTime.Minutes + 1) % 60;
        Lab3_DisplayTime();
        HAL_RTC_SetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
        break;
      case EDIT_SECOND:
        rtcTime.Seconds = (rtcTime.Seconds + 1) % 60;
        Lab3_DisplayTime();
        HAL_RTC_SetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
        break;
      case EDIT_MONTH:
        rtcDate.Month = (rtcDate.Month + 1) % 12;
        Lab3_DisplayDate();
        HAL_RTC_SetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);
        break;
      case EDIT_DAY:
        rtcDate.Date = (rtcDate.Date + 1) % 31;
        Lab3_DisplayDate();
        HAL_RTC_SetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);
        break;
      case EDIT_YEAR:
        rtcDate.Year = (rtcDate.Year + 1) % 100;
        Lab3_DisplayDate();
        HAL_RTC_SetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);
        break;
      default: return;
    }
	}
  if (GPIO_Pin == GPIO_PIN_2) {

    // Debouncing
    uint32_t tick = HAL_GetTick();
    if (tick - prevBtn2Tick < 400) return;
    prevBtn2Tick = tick;

    // Button 2 is used for changing the part of date/time being set
    switch (programState) {
      case DISPLAY_PREV_TIMES_OFF: // Fallthrough
      case DISPLAY_PREV_TIMES_ON:
        programState = EDIT_HOUR;
        Lab3_DisplayTime();
        break;
      case EDIT_HOUR:
        programState = EDIT_MINUTE;
        Lab3_DisplayTime();
        break;
      case EDIT_MINUTE:
        programState = EDIT_SECOND;
        Lab3_DisplayTime();
        break;
      case EDIT_SECOND:
        programState = EDIT_MONTH;
        Lab3_DisplayDate();
        break;
      case EDIT_MONTH:
        programState = EDIT_DAY;
        Lab3_DisplayDate();
        break;
      case EDIT_DAY:
        programState = EDIT_YEAR;
        Lab3_DisplayDate();
        break;
      case EDIT_YEAR:
        programState = DISPLAY_PREV_TIMES_OFF;
        break;
      default: return;
    }
	}
}


void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  // This alarm runs as every second from an interrupt

  if (programState == INIT_TEST_EEPROM) {
    // don't override eeprom text output with clock
    return;
  }

  Lab3_UpdateDateAndTime();
  Lab3_DisplayTime(0);

  if (programState == DISPLAY_PREV_TIMES_OFF) {
    if (HAL_GPIO_ReadPin(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_PIN)) {
      Lab3_DisplayDate();
    } else {
      // Stop displaying the date on line 14
      BSP_LCD_ClearStringLine(14);
    }
  }

  LCD_DisplayAnalogClock();
}


void Lab3_TestEEPROM() {
  // Clear the display

  BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
  LCD_DisplayString(6, 2, (uint8_t *) "Testing EEPROM....");

  HAL_Delay(1000);   //display for 1 second

  BSP_LCD_ClearStringLine(5);
  BSP_LCD_ClearStringLine(6);
  BSP_LCD_ClearStringLine(7);

  // The following variables are for testging I2C_EEPROM

  uint8_t data1 =0x67,  data2=0x68;
  uint8_t readData=0x00;
  char AA[34]= "efghijklmnopqstuvefghijklmnopqstuv";
  uint8_t * bufferdata=(uint8_t *)AA;
  int i;
  uint8_t readMatch=1;
  uint32_t EE_status;

  // *********************Testing I2C EEPROM------------------
  EE_status = I2C_ByteWrite(&I2c3_Handle, EEPROM_ADDRESS, memLocation, data1);
  if (EE_status == HAL_OK)
    LCD_DisplayString(0, 0, (uint8_t*) "w data1 OK");
  else
    LCD_DisplayString(0, 0, (uint8_t*) "w data1 failed");

  EE_status = I2C_ByteWrite(&I2c3_Handle, EEPROM_ADDRESS, memLocation + 1,
      data2);
  if (EE_status == HAL_OK)
    LCD_DisplayString(1, 0, (uint8_t*) "w data2 OK");
  else
    LCD_DisplayString(1, 0, (uint8_t*) "w data2 failed");

  readData = I2C_ByteRead(&I2c3_Handle, EEPROM_ADDRESS, memLocation);
  if (data1 == readData) {
    LCD_DisplayString(3, 0, (uint8_t*) "r data1 success");
  } else {
    LCD_DisplayString(3, 0, (uint8_t*) "r data1 mismatch");
  }
  LCD_DisplayInt(3, 14, readData);

  readData = I2C_ByteRead(&I2c3_Handle, EEPROM_ADDRESS, memLocation + 1);
  if (data2 == readData) {
    LCD_DisplayString(4, 0, (uint8_t*) "r data2 success");
  } else {
    LCD_DisplayString(4, 0, (uint8_t*) "r data2 mismatch");
  }
  LCD_DisplayInt(4, 14, readData);

  EE_status = I2C_BufferWrite(&I2c3_Handle, EEPROM_ADDRESS, memLocation,
      bufferdata, 34);
  if (EE_status == HAL_OK)
    LCD_DisplayString(6, 0, (uint8_t*) "w buffer OK");
  else
    LCD_DisplayString(6, 0, (uint8_t*) "W buffer failed");

  for (i = 0; i <= 33; i++) {
    readData = I2C_ByteRead(&I2c3_Handle, EEPROM_ADDRESS, memLocation + i);
    HAL_Delay(5); // Just for display effect. For EEPROM read, do not need dalay
    // BUT :  if here delay longer time, the floowing display will have trouble,???

    BSP_LCD_DisplayChar(COLUMN(i % 16), LINE(8 + 2 * (int )(i / 16)),
        (char) readData);
    BSP_LCD_DisplayChar(COLUMN(i % 16), LINE(9 + 2 * (int )(i / 16)),
        bufferdata[i]);
    if (bufferdata[i] != readData)
      readMatch = 0;
  }

  if (readMatch == 0)
    LCD_DisplayString(15, 0, (uint8_t*) "r buffer mismatch");
  else
    LCD_DisplayString(15, 0, (uint8_t*) "r buffer success");
  // ******************************testing I2C EEPROM*****************************/

  HAL_Delay(1000);
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
  MX_DMA2D_Init();
  MX_GPIO_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  // Initialize the LCD
  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER, LCD_FRAME_BUFFER);
  BSP_LCD_LayerDefaultInit(LCD_FOREGROUND_LAYER, LCD_FRAME_BUFFER);
  BSP_LCD_SelectLayer(LCD_FOREGROUND_LAYER);
  BSP_LCD_DisplayOn();

  // Configure graphics
  BSP_LCD_SetFont(&Font20);

  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_LCD_SetTextColor(LCD_COLOR_DARKMAGENTA);
  LCD_DisplayString(5, 3, (uint8_t *) "MT2TA4 LAB3");

  programState = INIT_TEST_EEPROM;
  Lab3_TestEEPROM();

  programState = DISPLAY_PREV_TIMES_OFF;
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_LCD_SetTextColor(LCD_COLOR_DARKMAGENTA);
  LCD_DisplayString(5, 3, (uint8_t *) "MT2TA4 LAB3");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 7;
  hltdc.Init.VerticalSync = 3;
  hltdc.Init.AccumulatedHBP = 14;
  hltdc.Init.AccumulatedVBP = 5;
  hltdc.Init.AccumulatedActiveW = 334;
  hltdc.Init.AccumulatedActiveH = 245;
  hltdc.Init.TotalWidth = 340;
  hltdc.Init.TotalHeigh = 247;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 0;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 0;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 0;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 0;
  pLayerCfg.ImageHeight = 0;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 0;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 0;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg1.Alpha = 0;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = 0;
  pLayerCfg1.ImageWidth = 0;
  pLayerCfg1.ImageHeight = 0;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

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
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x9;
  sTime.Minutes = 0x19;
  sTime.Seconds = 0x29;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
  sDate.Month = RTC_MONTH_MARCH;
  sDate.Date = 0x1;
  sDate.Year = 0x22;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x2;
  sAlarm.AlarmTime.Minutes = 0x32;
  sAlarm.AlarmTime.Seconds = 0x48;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_ALL;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 16;
  SdramTiming.ExitSelfRefreshDelay = 16;
  SdramTiming.SelfRefreshTime = 16;
  SdramTiming.RowCycleDelay = 16;
  SdramTiming.WriteRecoveryTime = 16;
  SdramTiming.RPDelay = 16;
  SdramTiming.RCDDelay = 16;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */

#define FWIDTH (BSP_LCD_GetFont()->Width)
#define FHEIGHT (BSP_LCD_GetFont()->Height)

void LCD_DisplayString(uint16_t line, uint16_t col, uint8_t *ptr)
{
  while (*ptr) {
    BSP_LCD_DisplayChar(FWIDTH * col, FHEIGHT * line, *ptr);
    col++;
    if ((col + 1) * FWIDTH >= BSP_LCD_GetXSize()) {
      col = 0;
      line++;
    }
    ptr++;
  }
}

void LCD_DisplayInt(uint16_t line, uint16_t col, int n)
{
  char lcd_buffer[15];
  sprintf(lcd_buffer, "%d", n);
  LCD_DisplayString(line, col, (uint8_t *) lcd_buffer);
}

void LCD_DisplayFloat(uint16_t line, uint16_t col, float f, int digits)
{
  char lcd_buffer[15];
  sprintf(lcd_buffer, "%.*f", digits, f);
  LCD_DisplayString(line, col, (uint8_t *) lcd_buffer);
}

void LCD_DisplayAnalogClock() {

  const uint16_t cx = 120, cy = 188;

  uint8_t
      h = rtcTime.Hours,
      m = rtcTime.Minutes,
      s = rtcTime.Seconds;

  BSP_LCD_DrawBitmap(cx - 50, cy - 50, clock100_ptr);

  uint16_t x2, y2;
  float angle;

  // hours hand
  angle = ((h % 12) + m / 60.0 - 3.0) * 2 * M_PI / 12.0;
  x2 = cx + (int) (cos(angle) * 24);
  y2 = cy + (int) (sin(angle) * 24);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_DrawLine(cx, cy, x2, y2);

  // minutes hand
  angle = (m + s / 60.0 - 15.0) * 2 * M_PI / 60.0;
  x2 = cx + (int) (cos(angle) * 36);
  y2 = cy + (int) (sin(angle) * 36);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_DrawLine(cx, cy, x2, y2);

  // seconds hand
  angle = (s - 15.0) * 2 * M_PI / 60.0;
  x2 = cx + (int) (cos(angle) * 42);
  y2 = cy + (int) (sin(angle) * 42);

  BSP_LCD_SetTextColor(LCD_COLOR_RED);
  BSP_LCD_DrawLine(cx, cy, x2, y2);

  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
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

#ifdef  USE_FULL_ASSERT
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

