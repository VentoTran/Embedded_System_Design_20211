/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "MQTTSim800.h"
#include "lcd.h"
#include "i2c.h"
#include "timers.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  bool state;
  GPIO_TypeDef* GPIO_Port;
  uint16_t Pin;
} Power;

typedef struct
{
  double voltage;
} Level;

typedef struct
{
  float Temperature;
  float Humidity;
  uint8_t AHT10_RX_Data[6];
  uint32_t AHT10_ADC_Raw;
} Node;

typedef enum
{
  SLEEPING = 0,
  RUNNING = 1
} Mode;

typedef enum
{
  ADAPTER = 0,
  BATTERY = 1,
  BOTH = 2
} PowerMode;

typedef struct
{
  bool SIM;
  bool GPRS;
  bool MQTT;
  bool SENDING;
} SIM800_St;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AHT10_ADDRESS_1 (0x38 << 1) // 0b1110000; Adress[7-bit]Wite/Read[1-bit]
#define AHT10_ADDRESS_2 (0x39 << 1) // 0b1110000; Adress[7-bit]Wite/Read[1-bit]

#define TimeOut 60000
#define MeasurePeriod 2000
#define ActionPeriod  30000
#define VCC   (3.333)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern SPI_HandleTypeDef hspi2;
extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart3;
extern SIM800_t SIM800;
extern SIM800_St SIM800_Status;

Power Adapter = {false, GPIOB, GPIO_PIN_8};
Power Batt = {false, GPIOB, GPIO_PIN_9};
Power Pump = {false, GPIOC, GPIO_PIN_13};
Node Node_1 = {0, 0, {0}, 0};
Node Node_2 = {0, 0, {0}, 0};
float DirtHumd = 0;
Level LipoBatt = {0};
Level Solar = {0};
PowerMode PowerSate = BOTH;
char StrgPC[40] = {0};

Mode DeviceState = RUNNING;
uint8_t CurrentPage = 0;

myButton_t Button_1 = {175, 185, 0, 35, 26, ILI9341_WHITE, false, NULL};
myButton_t Button_2 = {250, 185, 0, 50, 26, ILI9341_WHITE, false, NULL};

osThreadId_t SENSORHandle;
const osThreadAttr_t SENSOR_attributes = {
  .name = "SENSORs",
  .stack_size = 100 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};

/* USER CODE END Variables */
/* Definitions for SIM */
osThreadId_t SIMHandle;
const osThreadAttr_t SIM_attributes = {
  .name = "SIM",
  .stack_size = 150 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LCD */
osThreadId_t LCDHandle;
const osThreadAttr_t LCD_attributes = {
  .name = "LCD",
  .stack_size = 150 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for IRQ */
osThreadId_t IRQHandle;
const osThreadAttr_t IRQ_attributes = {
  .name = "IRQ",
  .stack_size = 100 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Timer01 */
osTimerId_t Timer01Handle;
const osTimerAttr_t Timer01_attributes = {
  .name = "Timer01"
};
/* Definitions for Timer02 */
osTimerId_t Timer02Handle;
const osTimerAttr_t Timer02_attributes = {
  .name = "Timer02"
};
/* Definitions for Timer03 */
osTimerId_t Timer03Handle;
const osTimerAttr_t Timer03_attributes = {
  .name = "Timer03"
};
/* Definitions for BinarySem01 */
osSemaphoreId_t BinarySem01Handle;
const osSemaphoreAttr_t BinarySem01_attributes = {
  .name = "BinarySem01"
};
/* Definitions for BinarySem02 */
osSemaphoreId_t BinarySem02Handle;
const osSemaphoreAttr_t BinarySem02_attributes = {
  .name = "BinarySem02"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Sensors_Task(void *argument);
/* USER CODE END FunctionPrototypes */

void MQTT_Task(void *argument);
void Display_Task(void *argument);
void IRQ_Task(void *argument);
void Action_Timer(void *argument);
void LCD_TimeOut(void *argument);
void Measure_Timer(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of BinarySem01 */
  BinarySem01Handle = osSemaphoreNew(1, 0, &BinarySem01_attributes);

  /* creation of BinarySem02 */
  BinarySem02Handle = osSemaphoreNew(1, 1, &BinarySem02_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of Timer01 */
  Timer01Handle = osTimerNew(Action_Timer, osTimerPeriodic, NULL, &Timer01_attributes);

  /* creation of Timer02 */
  Timer02Handle = osTimerNew(LCD_TimeOut, osTimerOnce, NULL, &Timer02_attributes);

  /* creation of Timer03 */
  Timer03Handle = osTimerNew(Measure_Timer, osTimerPeriodic, NULL, &Timer03_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of SIM */
  SIMHandle = osThreadNew(MQTT_Task, NULL, &SIM_attributes);

  /* creation of LCD */
  LCDHandle = osThreadNew(Display_Task, NULL, &LCD_attributes);

  /* creation of IRQ */
  IRQHandle = osThreadNew(IRQ_Task, NULL, &IRQ_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* creation of ACT */
  SENSORHandle = osThreadNew(Sensors_Task, NULL, &SENSOR_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_MQTT_Task */
/**
* @brief Function implementing the SIM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MQTT_Task */
void MQTT_Task(void *argument)
{
  /* USER CODE BEGIN MQTT_Task */
  SIM800.sim.apn = "m3-world";
  SIM800.sim.apn_user = "mms";
  SIM800.sim.apn_pass = "mms";
  SIM800.mqttServer.host = "theodoihensuyen.vn";
  SIM800.mqttServer.port = 1883;
  SIM800.mqttClient.username = "";
  SIM800.mqttClient.clientID = "testPub";
  SIM800.mqttClient.pass = "";
  SIM800.mqttClient.keepAliveInterval = 120;

  bool ping = false;

  osDelay(5000);
  MQTT_Init();
  osDelay(1000);

  //uint32_t MQTT_Task_Time = HAL_GetTick();
  /* Infinite loop */
  for(;;)
  {
    if (osSemaphoreAcquire(BinarySem02Handle, portMAX_DELAY) == osOK)
    {
      if((SIM800.mqttServer.connect == 0) || (SIM800_Status.SIM == 0) || (SIM800_Status.GPRS == 0) || (SIM800_Status.MQTT == 0))
      {
        MQTT_Init();
      }
      else if (!ping)
      {
        SIM800_Status.SENDING = true;
        MQTT_PubFloat("MANDevices/Roof_Garden/Node_1_Environment/Temperature", Node_1.Temperature, 2);
        MQTT_PubUint32("MANDevices/Roof_Garden/Node_1_Environment/Humidity", (int)Node_1.Humidity);
        MQTT_PubFloat("MANDevices/Roof_Garden/Node_2_Environment/Temperature", Node_2.Temperature, 2);
        MQTT_PubUint32("MANDevices/Roof_Garden/Node_2_Environment/Humidity", (int)Node_2.Humidity);
        MQTT_PubFloat("MANDevices/Roof_Garden/Dirt_Enviroment/Humidity", DirtHumd, 3);
        MQTT_PubFloat("MANDevices/Roof_Garden/Device_Power/LipoBatt", LipoBatt.voltage, 3);
        MQTT_PubFloat("MANDevices/Roof_Garden/Device_Power/Solar", Solar.voltage, 3);
        SIM800_Status.SENDING = false;
        ping = true;
        
      }
      else if (ping)
      {
        //  ----------- Keep connection -----------
        MQTT_PingReq();
        ping = false;
      }

      //  ----------- Enter block state ------------
      osDelay(30000);
    }
  }
  /* USER CODE END MQTT_Task */
}

/* USER CODE BEGIN Header_Display_Task */
/**
  * @brief  Function implementing the Display thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Display_Task */
void Display_Task(void *argument)
{
  /* USER CODE BEGIN Display_Task */
  
  ILI9341_Unselect();
  ILI9341_TouchUnselect();
  ILI9341_Init();
  ILI9341_FillScreen(ILI9341_BLACK);

  char StrgTemp[6];
  char StrgHumd[3];
  char StrgVolt[6];
  
  ILI9341_WriteString(50, 10, "SIM: ", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
  if (SIM800_Status.SIM)  {ILI9341_WriteString(85, 10, "OK", Font_7x10, ILI9341_GREEN, ILI9341_BLACK);}
  else  {ILI9341_WriteString(85, 10, "nOK", Font_7x10, ILI9341_RED, ILI9341_BLACK);}

  ILI9341_WriteString(130, 10, "GPRS: ", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
  if (SIM800_Status.GPRS)  {ILI9341_WriteString(172, 10, "OK", Font_7x10, ILI9341_GREEN, ILI9341_BLACK);}
  else  {ILI9341_WriteString(172, 10, "nOK", Font_7x10, ILI9341_RED, ILI9341_BLACK);}

  ILI9341_WriteString(220, 10, "MQTT: ", Font_7x10, ILI9341_WHITE, ILI9341_BLACK);
  if (SIM800_Status.MQTT)  {ILI9341_WriteString(262, 10, "OK", Font_7x10, ILI9341_GREEN, ILI9341_BLACK);}
  else  {ILI9341_WriteString(262, 10, "nOK", Font_7x10, ILI9341_RED, ILI9341_BLACK);}

  ILI9341_DrawLine(160, 27, 160, 100, ILI9341_WHITE);

  ILI9341_WriteString(50, 30, "NODE 1", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);
  ILI9341_WriteString(10, 55, "Temp: ", Font_11x18, ILI9341_ORANGE, ILI9341_BLACK);
  ftoa(Node_1.Temperature, StrgTemp, 2);
  ILI9341_WriteString(76, 55, StrgTemp, Font_11x18, ILI9341_ORANGE, ILI9341_BLACK);
  ILI9341_WriteString(132, 55, "o", Font_7x10, ILI9341_ORANGE, ILI9341_BLACK);
  ILI9341_WriteString(140, 55, "C", Font_11x18, ILI9341_ORANGE, ILI9341_BLACK);
  ILI9341_WriteString(10, 80, "Humd: ", Font_11x18, ILI9341_LIGHTBLUE, ILI9341_BLACK);
  intToStr((int)Node_1.Humidity, StrgHumd, 2);
  ILI9341_WriteString(76, 80, StrgHumd, Font_11x18, ILI9341_LIGHTBLUE, ILI9341_BLACK);
  ILI9341_WriteString(100, 80, "%", Font_11x18, ILI9341_LIGHTBLUE, ILI9341_BLACK);

  ILI9341_WriteString(210, 30, "NODE 2", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);
  ILI9341_WriteString(170, 55, "Temp: ", Font_11x18, ILI9341_ORANGE, ILI9341_BLACK);
  ftoa(Node_2.Temperature, StrgTemp, 2);
  ILI9341_WriteString(236, 55, StrgTemp, Font_11x18, ILI9341_ORANGE, ILI9341_BLACK);
  ILI9341_WriteString(293, 55, "o", Font_7x10, ILI9341_ORANGE, ILI9341_BLACK);
  ILI9341_WriteString(300, 55, "C", Font_11x18, ILI9341_ORANGE, ILI9341_BLACK);
  ILI9341_WriteString(170, 80, "Humd: ", Font_11x18, ILI9341_LIGHTBLUE, ILI9341_BLACK);
  intToStr((int)Node_2.Humidity, StrgHumd, 2);
  ILI9341_WriteString(236, 80, StrgHumd, Font_11x18, ILI9341_LIGHTBLUE, ILI9341_BLACK);
  ILI9341_WriteString(260, 80, "%", Font_11x18, ILI9341_LIGHTBLUE, ILI9341_BLACK);

  ILI9341_WriteString(44, 110, "Dirt Voltage: ", Font_11x18, ILI9341_BROWN, ILI9341_BLACK);
  ftoa(DirtHumd, StrgVolt, 3);
  ILI9341_WriteString(209, 110, StrgVolt, Font_11x18, ILI9341_BROWN, ILI9341_BLACK);
  ILI9341_WriteString(264, 110, " V", Font_11x18, ILI9341_BROWN, ILI9341_BLACK);

  ILI9341_WriteString(44, 135, "Lipo Voltage: ", Font_11x18, ILI9341_RED, ILI9341_BLACK);
  ftoa(LipoBatt.voltage, StrgVolt, 3);
  ILI9341_WriteString(209, 135, StrgVolt, Font_11x18, ILI9341_RED, ILI9341_BLACK);
  ILI9341_WriteString(264, 135, " V", Font_11x18, ILI9341_RED, ILI9341_BLACK);

  ILI9341_WriteString(44, 160, "Solar Voltage: ", Font_11x18, ILI9341_YELLOW, ILI9341_BLACK);
  ftoa(Solar.voltage, StrgVolt, 3);
  ILI9341_WriteString(209, 160, StrgVolt, Font_11x18, ILI9341_YELLOW, ILI9341_BLACK);
  ILI9341_WriteString(264, 160, " V", Font_11x18, ILI9341_YELLOW, ILI9341_BLACK);

  ILI9341_WriteString(110, 190, "Pump: ", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);
  if (Button_1.state == true) 
  {
    ILI9341_FillRectangle(Button_1.pos_x, Button_1.pos_y, Button_1.shape_w, Button_1.shape_h, ILI9341_GREEN);
    ILI9341_WriteString(180, 190, "ON", Font_11x18, ILI9341_BLACK, ILI9341_GREEN);
  }
  else  
  {
    ILI9341_FillRectangle(Button_1.pos_x, Button_1.pos_y, Button_1.shape_w, Button_1.shape_h, ILI9341_RED);
    ILI9341_WriteString(176, 190, "OFF", Font_11x18, ILI9341_BLACK, ILI9341_RED);
  }
  if (Button_2.state == true) 
  {
    ILI9341_FillRectangle(Button_2.pos_x, Button_2.pos_y, Button_2.shape_w, Button_2.shape_h, ILI9341_GREEN);
    ILI9341_WriteString(253, 190, "AUTO", Font_11x18, ILI9341_BLACK, ILI9341_GREEN);
  }
  else  
  {
    ILI9341_FillRectangle(Button_2.pos_x, Button_2.pos_y, Button_2.shape_w, Button_2.shape_h, ILI9341_RED);
    ILI9341_WriteString(253, 190, "AUTO", Font_11x18, ILI9341_BLACK, ILI9341_RED);
  }

  ILI9341_WriteString(83, 220, "Using: ", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);
  ILI9341_WriteString(160, 220, "...", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);
  
  //---------------------------------------------------------- Check State ---------------------------------------------------------

  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 1)   {Adapter.state = true;}
  else  {Adapter.state = false;}
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == 1)  {Batt.state = true;}
  else  {Batt.state = false;}

  if (Adapter.state && Batt.state)        {PowerSate = BOTH;}
  else if (Adapter.state && !Batt.state)  {PowerSate = ADAPTER;}
  else if (!Adapter.state && Batt.state)  {PowerSate = BATTERY;}

  if (PowerSate != BATTERY) {HAL_GPIO_WritePin(GPIO_SIM_GPIO_Port, GPIO_SIM_Pin, 1);}

  //-------------------------------------------------------- Timers Start ------------------------------------------------------------
  osTimerStart(Timer01Handle, ActionPeriod);
  osTimerStart(Timer02Handle, TimeOut);
  osTimerStart(Timer03Handle, MeasurePeriod);

  // /* Infinite loop */
  for(;;)
  {
    vTaskSuspend(LCDHandle);
    osDelay(200);

    if (SIM800_Status.SIM)  {ILI9341_WriteString(85, 10, "OK ", Font_7x10, ILI9341_GREEN, ILI9341_BLACK);}
    else  {ILI9341_WriteString(85, 10, "nOK", Font_7x10, ILI9341_RED, ILI9341_BLACK);}
    if (SIM800_Status.GPRS)  {ILI9341_WriteString(172, 10, "OK ", Font_7x10, ILI9341_GREEN, ILI9341_BLACK);}
    else  {ILI9341_WriteString(172, 10, "nOK", Font_7x10, ILI9341_RED, ILI9341_BLACK);}
    if (SIM800_Status.MQTT)  {ILI9341_WriteString(262, 10, "OK ", Font_7x10, ILI9341_GREEN, ILI9341_BLACK);}
    else  {ILI9341_WriteString(262, 10, "nOK", Font_7x10, ILI9341_RED, ILI9341_BLACK);}
    if (SIM800_Status.SENDING == true)
    {
      ILI9341_DrawLine(306, 10, 301, 15, ILI9341_WHITE);
      ILI9341_DrawLine(306, 10, 311, 15, ILI9341_WHITE);
      ILI9341_DrawLine(306, 10, 306, 20, ILI9341_WHITE);
    }
    else  {ILI9341_WriteString(301, 10, " ", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);}
    ftoa(Node_1.Temperature, StrgTemp, 2);
    ILI9341_WriteString(76, 55, StrgTemp, Font_11x18, ILI9341_ORANGE, ILI9341_BLACK);
    intToStr((int)Node_1.Humidity, StrgHumd, 2);
    ILI9341_WriteString(76, 80, StrgHumd, Font_11x18, ILI9341_LIGHTBLUE, ILI9341_BLACK);
    ftoa(Node_2.Temperature, StrgTemp, 2);
    ILI9341_WriteString(236, 55, StrgTemp, Font_11x18, ILI9341_ORANGE, ILI9341_BLACK);
    intToStr((int)Node_2.Humidity, StrgHumd, 2);
    ILI9341_WriteString(236, 80, StrgHumd, Font_11x18, ILI9341_LIGHTBLUE, ILI9341_BLACK);
    ftoa(DirtHumd, StrgVolt, 3);
    ILI9341_WriteString(209, 110, StrgVolt, Font_11x18, ILI9341_BROWN, ILI9341_BLACK);
    ftoa(LipoBatt.voltage, StrgVolt, 3);
    ILI9341_WriteString(209, 135, StrgVolt, Font_11x18, ILI9341_RED, ILI9341_BLACK);
    ftoa(Solar.voltage, StrgVolt, 3);
    ILI9341_WriteString(209, 160, StrgVolt, Font_11x18, ILI9341_YELLOW, ILI9341_BLACK);
    if (Button_1.state == true) 
    {
      ILI9341_FillRectangle(Button_1.pos_x, Button_1.pos_y, Button_1.shape_w, Button_1.shape_h, ILI9341_GREEN);
      ILI9341_WriteString(180, 190, "ON", Font_11x18, ILI9341_BLACK, ILI9341_GREEN);
    }
    else  
    {
      ILI9341_FillRectangle(Button_1.pos_x, Button_1.pos_y, Button_1.shape_w, Button_1.shape_h, ILI9341_RED);
      ILI9341_WriteString(176, 190, "OFF", Font_11x18, ILI9341_BLACK, ILI9341_RED);
    }
    if (Button_2.state == true) 
    {
      ILI9341_FillRectangle(Button_2.pos_x, Button_2.pos_y, Button_2.shape_w, Button_2.shape_h, ILI9341_GREEN);
      ILI9341_WriteString(253, 190, "AUTO", Font_11x18, ILI9341_BLACK, ILI9341_GREEN);
    }
    else  
    {
      ILI9341_FillRectangle(Button_2.pos_x, Button_2.pos_y, Button_2.shape_w, Button_2.shape_h, ILI9341_RED);
      ILI9341_WriteString(253, 190, "AUTO", Font_11x18, ILI9341_BLACK, ILI9341_RED);
    }
    if      (PowerSate == ADAPTER)  {ILI9341_WriteString(160, 220, "Adapter", Font_11x18, ILI9341_GREEN, ILI9341_BLACK);}
    else if (PowerSate == BATTERY)  {ILI9341_WriteString(160, 220, "Battery", Font_11x18, ILI9341_RED, ILI9341_BLACK);}
    else if (PowerSate == BOTH)     
    {
      if (Adapter.state && Batt.state) {ILI9341_WriteString(160, 220, "Both   ", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);}
      else if (!Adapter.state && Batt.state) {ILI9341_WriteString(160, 220, "Battery", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);}
      else if (Adapter.state && !Batt.state) {ILI9341_WriteString(160, 220, "Adapter", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);}
    }
    if (Adapter.state && Batt.state)  {ILI9341_WriteString(250, 220, ">>>", Font_11x18, ILI9341_BLACK, ILI9341_WHITE);}
    else  {ILI9341_WriteString(250, 220, "   ", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);}
    
  }
  /* USER CODE END Display_Task */
}

/* USER CODE BEGIN Header_IRQ_Task */
/**
* @brief Function implementing the IRQ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IRQ_Task */
void IRQ_Task(void *argument)
{
  /* USER CODE BEGIN IRQ_Task */
  uint16_t x, y;
  uint32_t currentTick = 0, lastTick = 0;

  /* Infinite loop */
  for(;;)
  {
//  -------------------------------------- Wait for event touch -----------------------------------------
    osDelay(100);
    if (osSemaphoreAcquire(BinarySem01Handle, portMAX_DELAY) == osOK)
    {
      osDelay(100);
//  -------------------------------------- Check for debounce -------------------------------------------
      currentTick = HAL_GetTick();
      if (HAL_GPIO_ReadPin(T_IRQ_GPIO_Port, T_IRQ_Pin) == 0 && ((currentTick - lastTick) >= 500))
      {
        lastTick = currentTick;
//  ------------------------------------- Read Touch Coordinate ------------------------------------------
        HAL_SPI_DeInit(&hspi2);
        hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
        HAL_SPI_Init(&hspi2);
        while(ILI9341_TouchGetCoordinates(&x, &y) != true);
        HAL_SPI_DeInit(&hspi2);
        hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
        HAL_SPI_Init(&hspi2);
//  ------------------------------------------ Reset Timer TimeOut ----------------------------------------
        osTimerStop(Timer02Handle);
        xTimerReset(Timer02Handle, 100);
//  --------------------------------------- Check button being pressed ------------------------------------
        if (DeviceState == SLEEPING)
        {
          DeviceState = RUNNING;
          HAL_GPIO_WritePin(LCD_LED_GPIO_Port, LCD_LED_Pin, 1);
          LCDHandle = osThreadNew(Display_Task, NULL, &LCD_attributes);
        }
        else
        {
          if ((x >= Button_1.pos_y) && 
              (x <= (Button_1.pos_y + Button_1.shape_h)) && 
              (y >= Button_1.pos_x) && 
              (y <= (Button_1.pos_x + Button_1.shape_w)) &&
              (PowerSate != BATTERY))
          {
            if (Button_1.state == false)
            {
              Button_1.state = true;
              HAL_GPIO_WritePin(Pump.GPIO_Port, Pump.Pin, 1);
            }
            else
            {
              Button_1.state = false;
              HAL_GPIO_WritePin(Pump.GPIO_Port, Pump.Pin, 0);
            }
            if (Button_1.state == true) 
            {
              ILI9341_FillRectangle(175, 185, 35, 26, ILI9341_GREEN);
              ILI9341_WriteString(180, 190, "ON", Font_11x18, ILI9341_BLACK, ILI9341_GREEN);
            }
            else  
            {
              ILI9341_FillRectangle(175, 185, 35, 26, ILI9341_RED);
              ILI9341_WriteString(176, 190, "OFF", Font_11x18, ILI9341_BLACK, ILI9341_RED);
            }
          }

          if ((x >= Button_2.pos_y) && 
              (x <= (Button_2.pos_y + Button_2.shape_h)) && 
              (y >= Button_2.pos_x) && 
              (y <= (Button_2.pos_x + Button_2.shape_w)) &&
              (PowerSate != BATTERY))
          {
            if (Button_2.state == false)  {Button_2.state = true;}
            else  {Button_2.state = false;}
            if (Button_2.state == true) 
            {
              ILI9341_FillRectangle(Button_2.pos_x, Button_2.pos_y, Button_2.shape_w, Button_2.shape_h, ILI9341_GREEN);
              ILI9341_WriteString(253, 190, "AUTO", Font_11x18, ILI9341_BLACK, ILI9341_GREEN);
            }
            else  
            {
              ILI9341_FillRectangle(Button_2.pos_x, Button_2.pos_y, Button_2.shape_w, Button_2.shape_h, ILI9341_RED);
              ILI9341_WriteString(253, 190, "AUTO", Font_11x18, ILI9341_BLACK, ILI9341_RED);
            }
          }

          if (Adapter.state && Batt.state && (x >= 220) && (x <= 239) && (y >= 250) && (y <= 283))
          {
            if (PowerSate == ADAPTER)
            {
              ILI9341_WriteString(160, 220, "Both   ", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);
              HAL_GPIO_WritePin(GPIO_SIM_GPIO_Port, GPIO_SIM_Pin, 1);
              HAL_GPIO_WritePin(GPIO_SOLAR_GPIO_Port, GPIO_SOLAR_Pin, 0);
              HAL_GPIO_WritePin(GPIO_ADAPTER_GPIO_Port, GPIO_ADAPTER_Pin, 0);
              PowerSate = BOTH;
            }
            else if (PowerSate == BATTERY)
            {
              ILI9341_WriteString(160, 220, "Adapter", Font_11x18, ILI9341_GREEN, ILI9341_BLACK);
              HAL_GPIO_WritePin(GPIO_SIM_GPIO_Port, GPIO_SIM_Pin, 1);
              osDelay(100);
              HAL_GPIO_WritePin(GPIO_ADAPTER_GPIO_Port, GPIO_ADAPTER_Pin, 0);
              osDelay(100);
              HAL_GPIO_WritePin(GPIO_SOLAR_GPIO_Port, GPIO_SOLAR_Pin, 1);
              PowerSate = ADAPTER;
            }
            else if (PowerSate == BOTH)
            {
              ILI9341_WriteString(160, 220, "Battery", Font_11x18, ILI9341_RED, ILI9341_BLACK);
              HAL_GPIO_WritePin(GPIO_ADAPTER_GPIO_Port, GPIO_ADAPTER_Pin, 1);
              osDelay(100);
              HAL_GPIO_WritePin(GPIO_SIM_GPIO_Port, GPIO_SIM_Pin, 0);
              if (Button_2.state == true)
              {
                Button_2.state = false;
                ILI9341_FillRectangle(Button_2.pos_x, Button_2.pos_y, Button_2.shape_w, Button_2.shape_h, ILI9341_RED);
                ILI9341_WriteString(253, 190, "AUTO", Font_11x18, ILI9341_BLACK, ILI9341_RED);
              }
              PowerSate = BATTERY;
            }
          }
        
        }
//  ------------------------------- Start the TimeOut again ------------------------------------------------
        osTimerStart(Timer02Handle, TimeOut);
        osTimerStart(Timer03Handle, MeasurePeriod);
      }
    }
  }
  /* USER CODE END IRQ_Task */
}

/* Action_Timer function */
void Action_Timer(void *argument)
{
  /* USER CODE BEGIN Action_Timer */
  if ((Button_2.state == true) && (DirtHumd >= 2.5))
  {
    HAL_GPIO_WritePin(GPIO_PUMP_GPIO_Port, GPIO_PUMP_Pin, 1);
    osDelay(1000);
    HAL_GPIO_WritePin(GPIO_PUMP_GPIO_Port, GPIO_PUMP_Pin, 0);
  }
  if((Adapter.state == true) && (PowerSate == BATTERY) && (LipoBatt.voltage <= 3.8))
  {
    ILI9341_WriteString(160, 220, "Adapter", Font_11x18, ILI9341_GREEN, ILI9341_BLACK);
    HAL_GPIO_WritePin(GPIO_SIM_GPIO_Port, GPIO_SIM_Pin, 1);
    osDelay(100);
    HAL_GPIO_WritePin(GPIO_ADAPTER_GPIO_Port, GPIO_ADAPTER_Pin, 0);
    osDelay(100);
    HAL_GPIO_WritePin(GPIO_SOLAR_GPIO_Port, GPIO_SOLAR_Pin, 1);
    PowerSate = ADAPTER;
  }
  char temp[6];
  strcat(StrgPC, "> ");
  ftoa(Node_1.Temperature, temp, 2);
  strcat(StrgPC, temp);
  strcat(StrgPC, "-");
  ftoa(Node_1.Humidity, temp, 0);
  strcat(StrgPC, temp);
  strcat(StrgPC, "-");
  ftoa(Node_2.Temperature, temp, 2);
  strcat(StrgPC, temp);
  strcat(StrgPC, "-");
  ftoa(Node_2.Humidity, temp, 0);
  strcat(StrgPC, temp);
  strcat(StrgPC, "-");
  ftoa(DirtHumd, temp, 3);
  strcat(StrgPC, temp);
  strcat(StrgPC, "-");
  ftoa(LipoBatt.voltage, temp, 3);
  strcat(StrgPC, temp);
  strcat(StrgPC, "-");
  ftoa(Solar.voltage, temp, 3);
  strcat(StrgPC, temp);
  strcat(StrgPC, "\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)StrgPC, strlen(StrgPC), 10);
  /* USER CODE END Action_Timer */
}

/* LCD_TimeOut function */
void LCD_TimeOut(void *argument)
{
  /* USER CODE BEGIN LCD_TimeOut */
  HAL_GPIO_WritePin(LCD_LED_GPIO_Port, LCD_LED_Pin, 0);
  DeviceState = SLEEPING;
  osThreadTerminate(LCDHandle);
  /* USER CODE END LCD_TimeOut */
}

/* Measure_Timer function */
void Measure_Timer(void *argument)
{
  /* USER CODE BEGIN Measure_Timer */
  osThreadResume(SENSORHandle);
  /* USER CODE END Measure_Timer */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == T_IRQ_Pin)
  {
    osSemaphoreRelease(BinarySem01Handle);
  }
}


void Sensors_Task(void *argument)
{
  //----------------------------------------------------- I2C -----------------------------------------------------------
  uint8_t AHT10_TmpHum_Cmd[3] = {0xAC, 0x33, 0x00};
  //----------------------------------------------------- ADC --------------------------------------------------
  ADC_ChannelConfTypeDef sConfig = {0};
  uint16_t ADC_t[3];

  for(;;)
  {
    osThreadResume(SENSORHandle);

    //----------------------------------------------------- I2C -----------------------------------------------------------
    MX_I2C1_Init();

    //----------------------------------------------------- I2C 1 ---------------------------------------------------------
    //HAL_I2C_Master_Transmit_IT(&hi2c1, AHT10_ADDRESS_1, (uint8_t*)AHT10_TmpHum_Cmd, 3);
    HAL_I2C_Master_Transmit(&hi2c1, AHT10_ADDRESS_1, (uint8_t*)AHT10_TmpHum_Cmd, 3, 10);
    osDelay(500);
    //HAL_I2C_Master_Receive_IT(&hi2c1, AHT10_ADDRESS_1, (uint8_t*)Node_1.AHT10_RX_Data, 6);
    HAL_I2C_Master_Receive(&hi2c1, AHT10_ADDRESS_1, (uint8_t*)Node_1.AHT10_RX_Data, 6, 10);
    osDelay(200);
    if(~Node_1.AHT10_RX_Data[0] & 0x80)
    {
      /* Convert to Temperature in °C */
      Node_1.AHT10_ADC_Raw = (((uint32_t)Node_1.AHT10_RX_Data[3] & 15) << 16) | ((uint32_t)Node_1.AHT10_RX_Data[4] << 8) | Node_1.AHT10_RX_Data[5];
      Node_1.Temperature = (float)(Node_1.AHT10_ADC_Raw * 200.00 / 1048576.00) - 50.00;

      /* Convert to Relative Humidity in % */
      Node_1.AHT10_ADC_Raw = ((uint32_t)Node_1.AHT10_RX_Data[1] << 12) | ((uint32_t)Node_1.AHT10_RX_Data[2] << 4) | (Node_1.AHT10_RX_Data[3] >> 4);
      Node_1.Humidity = (float)(Node_1.AHT10_ADC_Raw*100.00/1048576.00);
    }
    //----------------------------------------------------- I2C 2 --------------------------------------------------------
    HAL_I2C_Master_Transmit(&hi2c1, AHT10_ADDRESS_2, (uint8_t*)AHT10_TmpHum_Cmd, 3, 10);
    osDelay(500);
    HAL_I2C_Master_Receive(&hi2c1, AHT10_ADDRESS_2, (uint8_t*)Node_2.AHT10_RX_Data, 6, 10);
    osDelay(200);
    if(~Node_2.AHT10_RX_Data[0] & 0x80)
    {
      /* Convert to Temperature in °C */
      Node_2.AHT10_ADC_Raw = (((uint32_t)Node_2.AHT10_RX_Data[3] & 15) << 16) | ((uint32_t)Node_2.AHT10_RX_Data[4] << 8) | Node_2.AHT10_RX_Data[5];
      Node_2.Temperature = (float)(Node_2.AHT10_ADC_Raw * 200.00 / 1048576.00) - 50.00;

      /* Convert to Relative Humidity in % */
      Node_2.AHT10_ADC_Raw = ((uint32_t)Node_2.AHT10_RX_Data[1] << 12) | ((uint32_t)Node_2.AHT10_RX_Data[2] << 4) | (Node_2.AHT10_RX_Data[3] >> 4);
      Node_2.Humidity = (float)(Node_2.AHT10_ADC_Raw * 100.00 / 1048576.00);
    }
    
    //----------------------------------------------------- ADC -----------------------------------------------------------
    //------------------------------------------Channel 0----------------------------------------------

    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)  {Error_Handler();}
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    ADC_t[0] = HAL_ADC_GetValue(&hadc1);
    DirtHumd = (ADC_t[0] * VCC / 4095) - 0.175;
    HAL_ADC_Stop(&hadc1);
    
    //-------------------------------------------Channel 1--------------------------------------------
    sConfig.Channel = ADC_CHANNEL_1;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)  {Error_Handler();}
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    ADC_t[1] = HAL_ADC_GetValue(&hadc1);
    Solar.voltage = ADC_t[1] * VCC * 2.232 / 4095;
    HAL_ADC_Stop(&hadc1);

    //-------------------------------------------Channel 3--------------------------------------------
    sConfig.Channel = ADC_CHANNEL_3;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)  {Error_Handler();}
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    ADC_t[2] = HAL_ADC_GetValue(&hadc1);
    LipoBatt.voltage = ADC_t[2] * VCC * 3.963 / 4095;
    HAL_ADC_Stop(&hadc1);

    //------------------------------------------------------- Check State --------------------------------------------
    
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 1)   {Adapter.state = true;}
    else  {Adapter.state = false;}
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == 1)  {Batt.state = true;}
    else  {Batt.state = false;}

    //------------------------------------------------ Done Measuring -> Print new data ------------------------------
    osSemaphoreRelease(BinarySem02Handle);
    if (DeviceState == RUNNING) {osThreadResume(LCDHandle);}

  }
}
/* USER CODE END Application */

