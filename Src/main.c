/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "adc.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "AP.h"
#define UART0_OutString(STRING) printf(STRING)
#define UART0_OutUHex(value) printf("%x", value);
///* Private function prototypes -----------------------------------------------*/
//#ifdef __GNUC__
//  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
//     set to 'Yes') calls __io_putchar() */
//  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
//PUTCHAR_PROTOTYPE  
int fputc(int ch, FILE *f)
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100);


    return ch;
}
/* USER CODE END PFP */
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//--------------------------------------------------
// 1. Initialize GATT (add services, characteristics, CCCD’s, etc.). See Section 9.5.
// 2. Initialize GAP (advertisement data, connection parameters, etc.). See Section 9.3.
// 3. Advertise and optionally wait for a connection. See Section 9.4.
// 4. Respond to GATT requests and send notifications / indications as desired. See Section 9.6

//----------------very simple application processor-----------
// first characteristic is called "Data" and is a read/write parameter exchanged with the phone
uint8_t Data;        // read/write parameter
uint16_t Handle1;    // number associated with this characteristic
// second characteristic is called "Switches" and is a read only parameter that returns switch value (0-3)
uint16_t Handle2;    // number associated with this characteristic
// third characteristic is called "LEDs" and is a write-only parameter that sets the LED (0-7)
uint16_t Handle3;    // number associated with this characteristic
// fourth characteristic is called "Count" and is a local counter sent periodically to the phone
uint8_t light;
uint16_t Handle4;    // number associated with this characteristic
uint16_t CCCDhandle; // number associated with this CCCD, Client Characteristic Configuration Descriptor
uint16_t CCCDvalue;  // 0 means inactive, 1 means please notify

#define RECVSIZE 128
extern uint8_t RecvBuf[RECVSIZE];

const uint8_t NPI_GetStatusMsg[] =   {SOF,0x00,0x00,0x55,0x06,0x53};
const uint8_t NPI_GetVersionMsg[] =  {SOF,0x00,0x00,0x35,0x03,0x36};
const uint8_t NPI_AddServiceMsg[] = {
  SOF,3,0x00,     // length = 3
  0x35,0x81,      // SNP Add Service
  0x01,           // Primary Service
  0xF0,0xFF,      // UUID
  0xB9};          // FCS (calculated by AP_SendMessageResponse)

const uint8_t NPI_RegisterMsg[] = {   
  SOF,0x00,0x00,  // length = 0
  0x35,0x84,      // SNP Register Service
  0x00};          // FCS (calculated by AP_SendMessageResponse)

// call Set Advertisement twice 0, 2
const uint8_t NPI_SetAdvertisementMsg[] = {   
  SOF,11,0x00,    // length = 11
  0x55,0x43,      // SNP Set Advertisement Data
  0x01,           // Not connected Advertisement Data
  0x02,0x01,0x06, // GAP_ADTYPE_FLAGS,DISCOVERABLE | no BREDR
  0x06,0xFF,      // length, manufacturer specific
  0x0D ,0x00,     // Texas Instruments Company ID
  0x03,           // TI_ST_DEVICE_ID
  0x00,           // TI_ST_KEY_DATA_ID
  0x00,           // Key state
  0xEE};          // FCS (calculated by AP_SendMessageResponse)

const uint8_t NPI_SetAdvertisementDataMsg[] = {   
  SOF,27,0x00,    // length = 27
  0x55,0x43,      // SNP Set Advertisement Data
  0x00,           // Scan Response Data
  16,0x09,        // length, type=LOCAL_NAME_COMPLETE
  'N','O','R','A','I','N',' ',' ',' ',' ',' ',' ',' ',' ',' ',
// connection interval range
  0x05,           // length of this data
  0x12,           // GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE
  0x50,0x00,      // DEFAULT_DESIRED_MIN_CONN_INTERVAL
  0x20,0x03,      // DEFAULT_DESIRED_MAX_CONN_INTERVAL
// Tx power level
  0x02,           // length of this data
  0x0A,           // GAP_ADTYPE_POWER_LEVEL
  0x00,           // 0dBm
  0x77};          // FCS (calculated by AP_SendMessageResponse)
  
const uint8_t NPI_StartAdvertisementMsg[] = {   
  SOF,14,0x00,    // length = 14
  0x55,0x42,      // SNP Start Advertisement
  0x00,           // Connectable Undirected Advertisements
  0x00,0x00,      // Advertise infinitely.
  0x64,0x00,      // Advertising Interval (100 * 0.625 ms=62.5ms)
  0x00,           // Filter Policy RFU
  0x00,           // Initiator Address Type RFU
  0x00,0x01,0x00,0x00,0x00,0xC5, // RFU
  0x02,           // Advertising will restart with connectable advertising when a connection is terminated
  0xBB};          // FCS (calculated by AP_SendMessageResponse)

uint8_t NPI_ReadConfirmationMsg[] = {   
  SOF,0x08,0x00,  // length = 8 (7+data length, filled in dynamically)
  0x55,0x87,      // SNP Characteristic Read Confirmation (0x87)
  0x00,           // Success
  0x00,0x00,      // handle of connection always 0
  0x00,0x00,      // Handle of the characteristic value attribute being read (filled in dynamically
  0x00,0x00,      // offset, ignored, assumes small chucks of data
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // actual data (filled in dynamically)
  0x00};          // FCS (calculated by AP_SendMessageResponse)

uint8_t NPI_WriteConfirmationMsg[] = {   
  SOF,0x03,0x00,  // length = 3
  0x55,0x88,      // SNP Characteristic Write Confirmation
  0x00,           // Success
  0x00,0x00,      // handle of connection always 0
  0xDE};          // FCS (calculated by AP_SendMessageResponse)

uint8_t NPI_CCCDUpdatedConfirmationMsg[] = {   
  SOF,0x03,0x00,  // length = 3
  0x55,0x8B,      // SNP CCCD Updated Confirmation (0x8B)
  0x00,           // Success
  0x00,0x00,      // handle of connection always 0
  0xDD};          // FCS (calculated by AP_SendMessageResponse)

uint8_t NPI_SendNotificationIndicationMsg[] = {   
  SOF,0x07,0x00,  // length = 7 to 14 depending on data size
  0x55,0x89,      // SNP Send Notification Indication (0x89))
  0x00,0x00,      // handle of connection always 0
  0x00,0x00,      // Handle of the characteristic value attribute to notify / indicate (filled in dynamically
  0x00,           // RFU
  0x01,           // Indication Request type
  0x00,0,0,0,0,0,0,0, // 1 to 8 bytes of data filled in dynamically
  0xDD};      // FCS (calculated by AP_SendMessageResponse)

void OutValue(char *label,uint32_t value){ 
  UART0_OutString(label);
  UART0_OutUHex(value);
}

const uint8_t NPI_AddCharValue1[] = {   
  SOF,0x08,0x00,  // length = 8
  0x35,0x82,      // SNP Add Characteristic Value Declaration
  0x03,           // GATT Read+Write Permission
  0x0A,0x00,      // GATT Read+Write Properties
  0x00,           // RFU
  0x00,0x02,      // Maximum length of the attribute value=512
  0xF1,0xFF,      // UUID
  0xBA};          // FCS (calculated by AP_SendMessageResponse)
const uint8_t NPI_AddCharDescriptor1[] = {   
  SOF,11,0x00,    // length = 11
  0x35,0x83,      // SNP Add Characteristic Descriptor Declaration
  0x80,           // User Description String
  0x01,           // GATT Read Permissions
  0x05,0x00,      // Maximum Possible length of the user description string
  0x05,0x00,      // Initial length of the user description string
  'D','a','t','a',0, // Initial user description string
  0x0C};          // FCS (calculated by AP_SendMessageResponse)

const uint8_t NPI_AddCharValue2[] = {   
  SOF,0x08,0x00,  // length = 8
  0x35,0x82,      // SNP Add Characteristic Value Declaration
  0x01,           // GATT Read Permission
  0x02,0x00,      // GATT Read Properties
  0x00,           // RFU
  0x00,0x02,      // Maximum length of the attribute value=512
  0xF2,0xFF,      // UUID
  0xB3};          // FCS (calculated by AP_SendMessageResponse)
const uint8_t NPI_AddCharDescriptor2[] = {   
  SOF,15,0x00,    // length = 15
  0x35,0x83,      // SNP Add Characteristic Descriptor Declaration
  0x80,           // User Description String
  0x01,           // GATT Read Permissions
  0x09,0x00,      // Maximum Possible length of the user description string
  0x09,0x00,      // Initial length of the user description string
  'S','w','i','t','c','h','e','s',0, // Initial user description string
  0x0F};          // FCS (calculated by AP_SendMessageResponse)

const uint8_t NPI_AddCharValue3[] = {   
  SOF,0x08,0x00,  // length = 8
  0x35,0x82,      // SNP Add Characteristic Value Declaration
  0x02,           // GATT Write Permission
  0x08,0x00,      // GATT Write Properties
  0x00,           // RFU
  0x00,0x02,      // Maximum length of the attribute value=512
  0xF3,0xFF,      // UUID
  0xBB};          // FCS (calculated by AP_SendMessageResponse)
const uint8_t NPI_AddCharDescriptor3[] = {   
  SOF,11,0x00,    // length = 11
  0x35,0x83,      // SNP SNP Add Characteristic Descriptor Declaration
  0x80,           // User Description String
  0x01,           // GATT Read Permissions
  0x05,0x00,      // Maximum Possible length of the user description string
  0x05,0x00,      // Initial length of the user description string
  'L','E','D','s',0, // Initial user description string
  0x0E};          // FCS (calculated by AP_SendMessageResponse)

const uint8_t NPI_AddCharValue4[] = {   
  SOF,0x08,0x00,  // length = 8
  0x35,0x82,      // SNP Add Characteristic Value Declaration
  0x00,           // GATT no read, no Write Permission
  0x10,0x00,      // GATT notification Properties
  0x00,           // RFU
  0x00,0x02,      // Maximum length of the attribute value=512
  0xF4,0xFF,      // UUID
  0xA6};          // FCS (calculated by AP_SendMessageResponse)
const uint8_t NPI_AddCharDescriptor4[] = {   
  SOF,12,0x00,    // length = 12
  0x35,0x83,      // SNP Add Characteristic Descriptor Declaration
  0x84,           // User Description String+CCCD
  0x03,           // CCCD parameters read+write
  0x01,           // GATT Read Permissions
  0x06,0x00,      // Maximum Possible length of the user description string
  0x06,0x00,      // Initial length of the user description string
  'C','o','u','n','t',0, // Initial user description string
  0x0E};          // FCS (calculated by AP_SendMessageResponse)

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();

  /* USER CODE BEGIN 2 */
	uart_init_RXinterrupt();
	 volatile int r1; uint16_t h; uint32_t time=0;
  uint8_t responseNeeded;

  UART0_OutString("\n\rVery Simple Application Processor\n\r");
  UART0_OutString("\n\rReset CC2650");
  r1 = AP_Init();

  UART0_OutString("\n\rNPI_GetStatus");
  r1=AP_SendMessageResponse((uint8_t*)NPI_GetStatusMsg,RecvBuf,RECVSIZE);
  UART0_OutString("\n\rNPI_GetVersion");
  r1=AP_SendMessageResponse((uint8_t*)NPI_GetVersionMsg,RecvBuf,RECVSIZE); 
    
  UART0_OutString("\n\rAdd service");
  r1=AP_SendMessageResponse((uint8_t*)NPI_AddServiceMsg,RecvBuf,RECVSIZE); 
  //---------------Characteristic1 is read/write 8 bits---------
  Data = 1;

//  //-----------Characteristic3 is write only 8 bits----------
  UART0_OutString("\n\rAdd CharValue3");
  r1=AP_SendMessageResponse((uint8_t*)NPI_AddCharValue3,RecvBuf,RECVSIZE);
  Handle3 = (RecvBuf[7]<<8)+RecvBuf[6]; // handle for this characteristic
  UART0_OutString("\n\rAdd CharDescriptor3");
  r1=AP_SendMessageResponse((uint8_t*)NPI_AddCharDescriptor3,RecvBuf,RECVSIZE);
  //-----------Characteristic4 is notify 8 bits-------------
  light = 0;
  UART0_OutString("\n\rAdd CharValue4");
  r1=AP_SendMessageResponse((uint8_t*)NPI_AddCharValue4,RecvBuf,RECVSIZE);
  Handle4 = (RecvBuf[7]<<8)+RecvBuf[6]; // handle for this characteristic
  UART0_OutString("\n\rAdd CharDescriptor4");
  r1=AP_SendMessageResponse((uint8_t*)NPI_AddCharDescriptor4,RecvBuf,RECVSIZE);
  CCCDhandle = (RecvBuf[8]<<8)+RecvBuf[7]; // handle for this CCCD
  CCCDvalue = 0;
  //------------Register Service------------
  UART0_OutString("\n\rRegister service");
  r1=AP_SendMessageResponse((uint8_t*)NPI_RegisterMsg,RecvBuf,RECVSIZE);
  //--------------Advertise----------
  UART0_OutString("\n\rSetAdvertisement1");
  r1=AP_SendMessageResponse((uint8_t*)NPI_SetAdvertisementMsg,RecvBuf,RECVSIZE);
  UART0_OutString("\n\rSetAdvertisement2");
  r1=AP_SendMessageResponse((uint8_t*)NPI_SetAdvertisementDataMsg,RecvBuf,RECVSIZE);
  UART0_OutString("\n\rStartAdvertisement");
  r1=AP_SendMessageResponse((uint8_t*)NPI_StartAdvertisementMsg,RecvBuf,RECVSIZE);
  UART0_OutString("\n\rGetStatus");
  r1=AP_SendMessageResponse((uint8_t*)NPI_GetStatusMsg,RecvBuf,RECVSIZE);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
		time++;
    if(AP_RecvStatus()){
      if(AP_RecvMessage(RecvBuf,RECVSIZE)==APOK){
        AP_EchoReceived(APOK);        
        if((RecvBuf[3]==0x55)&&(RecvBuf[4]==0x88)){// SNP Characteristic Write Indication (0x88)
          h = (RecvBuf[8]<<8)+RecvBuf[7]; // handle for this characteristic
          responseNeeded = RecvBuf[9];
          // process possible write indications
          if(h == Handle1){      // Handle1 could be write
            Data = RecvBuf[12];
            OutValue("\n\rWrite Data=",RecvBuf[12]);
          }else if(h == Handle3){// Handle3 could be write
            OutValue("\n\rWrite LED=",RecvBuf[12]);
            //LaunchPad_Output(RecvBuf[12]&0x07);
						if(RecvBuf[12]=='0'){
							HAL_GPIO_WritePin(LD4_GPIO_Port,LD4_Pin,0);
						}else{
							HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin,1);
						}
          }
          if(responseNeeded){
            AP_SendMessage(NPI_WriteConfirmationMsg);
            AP_EchoSendMessage(NPI_WriteConfirmationMsg);
          }
        }

        if((RecvBuf[3]==0x55)&&(RecvBuf[4]==0x8B)){// SNP CCCD Updated Indication (0x8B)
          h = (RecvBuf[8]<<8)+RecvBuf[7]; // handle for this characteristic
          responseNeeded = RecvBuf[9];
          if(CCCDhandle == h){
            CCCDvalue = (RecvBuf[11]<<8)+RecvBuf[10];
          }
          if(responseNeeded){
            AP_SendMessage(NPI_CCCDUpdatedConfirmationMsg);
            AP_EchoSendMessage(NPI_CCCDUpdatedConfirmationMsg);
          }
        }        
      }
    }
    if(time>4000000){
      time = 0;
      //Count++;
      if(CCCDvalue){ 
				light=readLight();
        NPI_SendNotificationIndicationMsg[7] = Handle4&0x0FF; // handle
        NPI_SendNotificationIndicationMsg[8] = Handle4>>8; 
        NPI_SendNotificationIndicationMsg[11] = light;
        OutValue("\n\rSend Count=",light);
        r1=AP_SendMessageResponse(NPI_SendNotificationIndicationMsg,RecvBuf,RECVSIZE);
      }
    }
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
