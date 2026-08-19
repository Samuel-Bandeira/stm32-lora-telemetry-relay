/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    subghz_phy_app.c
  * @author  MCD Application Team
  * @brief   Application of the SubGHz_Phy Middleware
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"

/* USER CODE BEGIN Includes */
#include "stm32_timer.h"
#include "stm32_seq.h"
#include "utilities_def.h"
#include "app_version.h"
#include "subghz_phy_version.h"
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  RX,
  RX_TIMEOUT,
  RX_ERROR,
  TX,
  TX_TIMEOUT,
} States_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef enum{
	BOARD_NOT_DEFINED,
	BOARD_RX,
	BOARD_TX
}board_type_t;

typedef enum{
	SELECT_ROLE,
	SELECT_BANDWITH,
	CONFIG_DONE,
}board_config_t;

/* Configurations */
/*Timeout*/
#define RX_TIMEOUT_VALUE              7000
#define TX_TIMEOUT_VALUE              5000
/* PING string*/
#define PING "PING"
/* PONG string*/
#define PONG "PONG"
/*Size of the payload to be sent*/
/* Size must be greater of equal the PING and PONG*/
#define MAX_APP_BUFFER_SIZE          50
#if (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE)
#error PAYLOAD_LEN must be less or equal than MAX_APP_BUFFER_SIZE
#endif /* (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE) */
/* wait for remote to be in Rx, before sending a Tx frame*/
#define RX_TIME_MARGIN                200
/* LED blink Period*/
#define PROCESS_PERIOD_MS                 3000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Radio events function pointer */
static RadioEvents_t RadioEvents;

/* USER CODE BEGIN PV */
/*Ping Pong FSM states */
static States_t State = RX;
/* App Rx Buffer*/
static uint8_t BufferRx[MAX_APP_BUFFER_SIZE];
/* App Tx Buffer*/
static uint8_t BufferTx[MAX_APP_BUFFER_SIZE];
/* Last  Received Buffer Size*/
uint16_t RxBufferSize = 0;
/* Last  Received packer Rssi*/
int8_t RssiValue = 0;
/* Last  Received packer SNR (in Lora modulation)*/
int8_t SnrValue = 0;


static board_type_t g_board_type = BOARD_NOT_DEFINED;
static board_config_t g_board_config_state = SELECT_ROLE;
static uint32_t g_bandwith_selector_ctn = 0;
static UTIL_TIMER_Object_t process_timer;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/*!
 * @brief Function to be executed on Radio Tx Done event
 */
static void OnTxDone(void);

/**
  * @brief Function to be executed on Radio Rx Done event
  * @param  payload ptr of buffer received
  * @param  size buffer size
  * @param  rssi
  * @param  LoraSnr_FskCfo
  */
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);

/**
  * @brief Function executed on Radio Tx Timeout event
  */
static void OnTxTimeout(void);

/**
  * @brief Function executed on Radio Rx Timeout event
  */
static void OnRxTimeout(void);

/**
  * @brief Function executed on Radio Rx Error event
  */
static void OnRxError(void);

/* USER CODE BEGIN PFP */

/**
  * @brief PingPong state machine implementation
  */
//static void PingPong_Process(void);

static void tx_rx_process();

static void tx_rx_process_timeout_cb(void* p_arg);
static void RadioSend(void);
static void board_init();
static void blink_led(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,uint8_t blinks_ctn,uint32_t duration_ms);
static void config_bandwith(uint8_t b1_input,uint8_t b3_input);

/**
  * @brief PingPong RX configure and process
  */
static void RadioRx(void);


/* USER CODE END PFP */

/* Exported functions ---------------------------------------------------------*/


void SubghzApp_Init(void)
{
  /* USER CODE BEGIN SubghzApp_Init_1 */

  APP_LOG(TS_OFF, VLEVEL_M, "\n\rPING PONG\n\r");
  /* Get SubGHY_Phy APP version*/
  APP_LOG(TS_OFF, VLEVEL_M, "APPLICATION_VERSION: V%X.%X.%X\r\n",
          (uint8_t)(APP_VERSION_MAIN),
          (uint8_t)(APP_VERSION_SUB1),
          (uint8_t)(APP_VERSION_SUB2));

  /* Get MW SubGhz_Phy info */
  APP_LOG(TS_OFF, VLEVEL_M, "MW_RADIO_VERSION:    V%X.%X.%X\r\n",
          (uint8_t)(SUBGHZ_PHY_VERSION_MAIN),
          (uint8_t)(SUBGHZ_PHY_VERSION_SUB1),
          (uint8_t)(SUBGHZ_PHY_VERSION_SUB2));

  board_init();

  if(g_board_type == BOARD_TX){
	  UTIL_TIMER_Create(&process_timer, PROCESS_PERIOD_MS, UTIL_TIMER_PERIODIC, tx_rx_process_timeout_cb, NULL);
	  UTIL_TIMER_Start(&process_timer);
  }
  /* USER CODE END SubghzApp_Init_1 */

  /* Radio initialization */
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init(&RadioEvents);

  /* USER CODE BEGIN SubghzApp_Init_2 */

  /* Radio Set frequency */
  Radio.SetChannel(RF_FREQUENCY);

  /* Radio configuration */
  APP_LOG(TS_OFF, VLEVEL_M, "---------------\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_MODULATION\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_BW=%d kHz\n\r", (1 << g_bandwith_selector_ctn) * 125);
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_SF=%d\n\r", LORA_SPREADING_FACTOR);

  /*fills tx buffer*/
  memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);

  /*register task to to be run in while(1) after Radio IT*/
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), UTIL_SEQ_RFU, tx_rx_process);
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /*starts first process */
#ifdef BOARD_RX
  //HAL_Delay(RX_TIMEOUT_VALUE);
  //RadioRx();
#endif
  /* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
static void blink_led(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,uint8_t blinks_ctn,uint32_t duration_ms){
	for(uint8_t i=0;i<blinks_ctn;i++){
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin,GPIO_PIN_SET);
		HAL_Delay(duration_ms);
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin,GPIO_PIN_RESET);
		HAL_Delay(duration_ms);
	}
}

static void config_role(uint8_t b1_input,uint8_t b2_input){
	if(b1_input == 0){
		APP_LOG(TS_ON, VLEVEL_L, "Board is TX\n\r");
		g_board_type = BOARD_TX;
		g_board_config_state = SELECT_BANDWITH;
		if(g_board_type == BOARD_TX)blink_led(LED3_GPIO_Port, LED3_Pin,1,1000);/* LED_RED */
		return;
	}

	if(b2_input == 0){
		APP_LOG(TS_ON, VLEVEL_L, "Board is RX\n\r");
		g_board_type = BOARD_RX;
		g_board_config_state = SELECT_BANDWITH;
		if(g_board_type == BOARD_RX)blink_led(LED2_GPIO_Port, LED2_Pin,1,1000);/* LED_GREEN */
		return;
	}
}

static void config_bandwith(uint8_t b1_input,uint8_t b3_input){
	static uint8_t b3_last_state = 0;
	static uint8_t first_cycle = 1;

	if(b1_input == 0 && first_cycle == 0){
		g_board_config_state = CONFIG_DONE;
		if(g_board_type == BOARD_TX)blink_led(LED3_GPIO_Port, LED3_Pin,5,100);/* LED_RED */
		if(g_board_type == BOARD_RX)blink_led(LED2_GPIO_Port, LED2_Pin,5,100);/* LED_GREEN */
	}

	if(b3_last_state != b3_input){
		b3_last_state = b3_input;
	}else{
		return;
	}

	if(b3_input == 1)return;

	g_bandwith_selector_ctn++;

	if(first_cycle == 1){
		first_cycle = 0;
		g_bandwith_selector_ctn--;
	}
	if(g_bandwith_selector_ctn > 2)g_bandwith_selector_ctn = 0;

	if(g_board_type == BOARD_TX)blink_led(LED3_GPIO_Port, LED3_Pin,g_bandwith_selector_ctn+1,300);/* LED_RED */
	if(g_board_type == BOARD_RX)blink_led(LED2_GPIO_Port, LED2_Pin,g_bandwith_selector_ctn+1,300);/* LED_GREEN */
}

static void board_init(){
	while(1){
		uint8_t b1_input = HAL_GPIO_ReadPin(BUT1_GPIO_Port, BUT1_Pin);
		uint8_t b2_input = HAL_GPIO_ReadPin(BUT2_GPIO_Port, BUT2_Pin);
		uint8_t b3_input = HAL_GPIO_ReadPin(BUT3_GPIO_Port, BUT3_Pin);
		HAL_Delay(100);

		switch(g_board_config_state){
		case SELECT_ROLE:
			config_role(b1_input,b2_input);
			break;

		case SELECT_BANDWITH:
			config_bandwith(b1_input,b3_input);
			break;

		case CONFIG_DONE:
			return;
		}

	}
}



static void tx_rx_process_timeout_cb(void* p_arg){
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
}

static void OnTxDone(void)
{
  /* USER CODE BEGIN OnTxDone */
  APP_LOG(TS_ON, VLEVEL_L, "OnTxDone\n\r");
  /* Run PingPong process in background*/
  //UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnTxDone */
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  /* USER CODE BEGIN OnRxDone */
  APP_LOG(TS_ON, VLEVEL_L, "OnRxDone\n\r");
  APP_LOG(TS_ON, VLEVEL_L, "RssiValue=%d dBm, SnrValue=%ddB\n\r", rssi, LoraSnr_FskCfo);
  /* Record payload Signal to noise ratio in Lora*/
  SnrValue = LoraSnr_FskCfo;

  /* Update the State of the FSM*/
  State = RX;
  /* Clear BufferRx*/
  memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);
  /* Record payload size*/
  RxBufferSize = size;
  if (RxBufferSize <= MAX_APP_BUFFER_SIZE)
  {
    memcpy(BufferRx, payload, RxBufferSize);
  }
  /* Record Received Signal Strength*/
  RssiValue = rssi;
  /* Record payload content*/
  APP_LOG(TS_ON, VLEVEL_H, "payload. size=%d \n\r", size);
  for (int32_t i = 0; i < PAYLOAD_LEN; i++)
  {
    APP_LOG(TS_OFF, VLEVEL_H, "%02X", BufferRx[i]);
    if (i % 16 == 15)
    {
      APP_LOG(TS_OFF, VLEVEL_H, "\n\r");
    }
  }
  APP_LOG(TS_OFF, VLEVEL_H, "\n\r");
  /* Run PingPong process in background*/
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnRxDone */
}

static void OnTxTimeout(void)
{
  /* USER CODE BEGIN OnTxTimeout */
  APP_LOG(TS_ON, VLEVEL_L, "OnTxTimeout\n\r");
  /* Run PingPong process in background*/
  //UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnTxTimeout */
}

static void OnRxTimeout(void)
{
  /* USER CODE BEGIN OnRxTimeout */
  APP_LOG(TS_ON, VLEVEL_L, "OnRxTimeout\n\r");
  /* Update the State of the FSM*/
  State = RX_TIMEOUT;
  /* Run PingPong process in background*/
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnRxTimeout */
}

static void OnRxError(void)
{
  /* USER CODE BEGIN OnRxError */
  APP_LOG(TS_ON, VLEVEL_L, "OnRxError\n\r");
  /* Update the State of the FSM*/
  State = RX_ERROR;
  /* Run PingPong process in background*/
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnRxError */
}

/* USER CODE BEGIN PrFD */

static void RadioSend(void)
{
  Radio.Sleep();
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, g_bandwith_selector_ctn,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);
  Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);

  Radio.Send(BufferTx, PAYLOAD_LEN);
}

static void RadioRx(void)
{
  Radio.Sleep();
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, g_bandwith_selector_ctn, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
  if (LORA_FIX_LENGTH_PAYLOAD_ON == true)
  {
    Radio.SetMaxPayloadLength(MODEM_LORA, PAYLOAD_LEN);
  }
  else
  {
    Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);
  }

  Radio.Rx(RX_TIMEOUT_VALUE);
}

static void tx_rx_process(){
	if(g_board_type == BOARD_TX){
		Radio.Sleep();

		/* master toggles red led */
		blink_led(LED3_GPIO_Port, LED3_Pin,1,150); /* LED_RED */
		/* Add delay between RX and TX */
		HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN);

		/* master sends PING*/
		APP_LOG(TS_ON, VLEVEL_L, "..."
				"PING"
				"\n\r");
		APP_LOG(TS_ON, VLEVEL_L, "Master Tx start\n\r");
		memcpy(BufferTx, PING, sizeof(PING) - 1);
		RadioSend();
	}

	if(g_board_type == BOARD_RX){
		switch(State){
		case RX_TIMEOUT:
		case RX_ERROR:
			APP_LOG(TS_ON, VLEVEL_L, "Slave Rx start\n\r");
			RadioRx();
			break;

		case RX:
			if (RxBufferSize > 0)
			{
			  if (strncmp((const char *)BufferRx, PING, sizeof(PING) - 1) == 0)
			  {
				blink_led(LED2_GPIO_Port, LED2_Pin,1,150);/* LED_GREEN */
				/* Add delay between RX and TX */
				HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN);
			  }
			}
			RadioRx();
			break;

		default:
			break;
		}
	}
}

//static void PingPong_Process(void)
//{
//  Radio.Sleep();
//
//  switch (State)
//  {
//    case RX:
//
//      if (isMaster == true)
//      {
//        if (RxBufferSize > 0)
//        {
//          if (strncmp((const char *)BufferRx, PONG, sizeof(PONG) - 1) == 0)
//          {
//            UTIL_TIMER_Stop(&process_timer);
//            /* switch off green led */
//            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET); /* LED_GREEN */
//            /* master toggles red led */
//            HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin); /* LED_RED */
//            /* Add delay between RX and TX */
//            HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN);
//            /* master sends PING*/
//            APP_LOG(TS_ON, VLEVEL_L, "..."
//                    "PING"
//                    "\n\r");
//            APP_LOG(TS_ON, VLEVEL_L, "Master Tx start\n\r");
//            memcpy(BufferTx, PING, sizeof(PING) - 1);
//            RadioSend();
//          }
//          else if (strncmp((const char *)BufferRx, PING, sizeof(PING) - 1) == 0)
//          {
//            /* A master already exists then become a slave */
//            isMaster = false;
//            APP_LOG(TS_ON, VLEVEL_L, "Slave Rx start\n\r");
//            RadioRx();
//          }
//          else /* valid reception but neither a PING or a PONG message */
//          {
//            /* Set device as master and start again */
//            isMaster = true;
//            APP_LOG(TS_ON, VLEVEL_L, "Master Rx start\n\r");
//            RadioRx();
//          }
//        }
//      }
//      else
//      {
//        if (RxBufferSize > 0)
//        {
//          if (strncmp((const char *)BufferRx, PING, sizeof(PING) - 1) == 0)
//          {
//            UTIL_TIMER_Stop(&process_timer);
//            /* switch off red led */
//            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); /* LED_RED */
//            /* slave toggles green led */
//            HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); /* LED_GREEN */
//            /* Add delay between RX and TX */
//            HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN);
//            /*slave sends PONG*/
//            APP_LOG(TS_ON, VLEVEL_L, "..."
//                    "PONG"
//                    "\n\r");
//            APP_LOG(TS_ON, VLEVEL_L, "Slave  Tx start\n\r");
//            memcpy(BufferTx, PONG, sizeof(PONG) - 1);
//            RadioSend();
//          }
//          else /* valid reception but not a PING as expected */
//          {
//            /* Set device as master and start again */
//            isMaster = true;
//            APP_LOG(TS_ON, VLEVEL_L, "Master Rx start\n\r");
//            RadioRx();
//          }
//        }
//      }
//      break;
//    case TX:
//      APP_LOG(TS_ON, VLEVEL_L, "Rx start\n\r");
//      RadioRx();
//      break;
//    case RX_TIMEOUT:
//    case RX_ERROR:
//      if (isMaster == true)
//      {
//        /* Send the next PING frame */
//        /* Add delay between RX and TX*/
//        /* add random_delay to force sync between boards after some trials*/
//        HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN + random_delay);
//        APP_LOG(TS_ON, VLEVEL_L, "Master Tx start\n\r");
//        /* master sends PING*/
//        memcpy(BufferTx, PING, sizeof(PING) - 1);
//        RadioSend();
//      }
//      else
//      {
//        APP_LOG(TS_ON, VLEVEL_L, "Slave Rx start\n\r");
//        RadioRx();
//      }
//      break;
//    case TX_TIMEOUT:
//      APP_LOG(TS_ON, VLEVEL_L, "Slave Rx start\n\r");
//      RadioRx();
//      break;
//    default:
//      break;
//  }
//}
/* USER CODE END PrFD */
