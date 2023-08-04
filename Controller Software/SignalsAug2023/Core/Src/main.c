/* USER CODE BEGIN Header */
/**
 * WARNING
 * Code created using STM32CubeIDE from ST Microelectronics.
 * THIS FILE IS A MIX OF MACHINE-GENERATED / MAN-MADE CODE including most of the startup code
 *
 * YOU ARE STRONGLY ADVISED NOT TO EDIT THIS FILE
 *
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include	<stdio.h>
#include	<string.h>
#include	<stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	COM1_RXBUFF_SIZE	100
#define	COM2_RXBUFF_SIZE	100
#define	COM1_TXBUFF_SIZE	2000
#define	COM2_TXBUFF_SIZE	2000

#define	IO_DAISY_CHAIN_BYTE_LEN	8

#define	FOREVER_LOOP_REPEAT_MS	20
#define	SLOW_LOOP_REPEAT_MS	500

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
uint8_t	rx1buf[4], rx2buf[4], comrxtmp1[4], comrxtmp2[4], txt[200];
uint32_t	millisecs = 0,	forever_loop_timer = 0, slow_loop_timer = 0,
		ins_history[(IO_DAISY_CHAIN_BYTE_LEN << 3) + 2] = {0},
		input_test_error_flags = 0;

int		len, f[10] = {0};
uint8_t	spi_tx[IO_DAISY_CHAIN_BYTE_LEN + 2] = {0}, spi_rx[IO_DAISY_CHAIN_BYTE_LEN + 2] = {0};
volatile 	bool	spi_error_flag = false,
					spi_TxRx_flag = false,
					timer_1ms_flag = false,
//					timer_10ms_flag = false,
					u1rx_flag = false,
					u2rx_flag = false,
					u1tx_flag = false,
					u2tx_flag = false,
					can_flag  = false;

/**
 * UART circular buffers with pointers and flags
 */
//	buffers
uint8_t	com1_rxbuff	[COM1_RXBUFF_SIZE + 2] = {0};
uint8_t	com2_rxbuff	[COM2_RXBUFF_SIZE + 2] = {0};
uint8_t	com1_txbuff	[COM1_TXBUFF_SIZE + 2] = {0};
uint8_t	com2_txbuff	[COM2_TXBUFF_SIZE + 2] = {0};

//	pointers
uint32_t	com1_rx_onptr	= 0;
uint32_t	com1_rx_offptr	= 0;
uint32_t	com1_tx_onptr	= 0;
uint32_t	com1_tx_offptr	= 0;

uint32_t	com2_rx_onptr	= 0;
uint32_t	com2_rx_offptr	= 0;
uint32_t	com2_tx_onptr	= 0;
uint32_t	com2_tx_offptr	= 0;

//	flags
bool		com1_rx_full	= false;
bool		com1_rx_empty	= true;
bool		com1_tx_full	= false;
bool		com1_tx_empty	= true;
bool		com1_tx_busy	= false;

bool		com2_rx_full	= false;
bool		com2_rx_empty	= true;
bool		com2_tx_full	= false;
bool		com2_tx_empty	= true;
bool		com2_tx_busy	= false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

bool	getcc	(UART_HandleTypeDef * huart, char * ch)	;
int16_t	onbuff	(UART_HandleTypeDef * huart)	;	//	Return number of characters waiting
void	spi_cs_set	()	;		//	SPI Chip Select line ___--
void	spi_cs_clr	()	;		//	SPI Chip Select line --___
extern	void	Signal_sys_Setup	()	;
extern	void	ForeverLoop	()	;
extern	void	one_ms_update_stuff	()	;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

bool	get_raw_input_bytes	(char * dest)	{
	for	(int i = 0; i < IO_DAISY_CHAIN_BYTE_LEN; i++)
		dest[i] = spi_rx[i];
	return	true;
}

/**
 * bool	set_output_bit	(uint32_t which_out, bool hiorlo)	{
 * Function in "main.c"
 * Number of output BYTES defined as 'IO_DAISY_CHAIN_BYTE_LEN' defined in "main.c", (probably 8)
 * Number of outputs = BYTES * 8 (numbered 0 to 63 when IO_DAISY_CHAIN_BYTE_LEN == 8)
 *
 * Function returns 'false' if 'which_output' contains invalid output number, returns 'true' otherwise
 * Using hiorlo 'true' turns output on, 'false' off.
 */
bool	set_output_bit	(uint32_t which_output, bool hiorlo)	{	//	Big Endian storage - high output numbers in low numbered bytes
	if	(which_output >= (IO_DAISY_CHAIN_BYTE_LEN << 3)){//	'which_output' must be in range of numof inputs
		input_test_error_flags |= 0x10;		//	Set global error flag
		return	false;
	}
	int	which_byte 	= IO_DAISY_CHAIN_BYTE_LEN - (which_output >> 3) - 1;
	int	which_bit	= 1 << (which_output & 0x07);						//	Higher numbered outputs in higher numbered bit positions
	if	(hiorlo)	{	//	'true'
		spi_tx[which_byte] |= which_bit;	//	set bit to '1'
	}
	else	{	//	'false'
		spi_tx[which_byte] &= ~which_bit;	//	clear bit to '0'
	}
	return	true;
}


/**
 * bool	get_input_bit_debounced
 * A 'De-Bounce' implementation
 * 			This is the function to call to access inputs.
 *
 * 	Inputs are scanned (and outputs updated) regularly, probably 50 times per second.
 *
 * Most recent 31 input[n] bit reads stored in uint32_t ins_history[n], newest in bit 31, oldest bit 0
 * (chose 31 over 32 so that number positive regardless of 'int' ot 'uint')
 * function takes odd number for samples and tests this many from most recent back in time
 * returns 'true' for more '1's than '0's, false for more '0's than '1's
 * On error sets bits in 'input_test_error_flags' and returns false
 */
bool	get_input_bit_debounced	(uint32_t which_input, uint32_t how_many)	{
	uint32_t threshold, ones_count = 0, test_bit = 0x40000000L;
	if	(which_input >= (IO_DAISY_CHAIN_BYTE_LEN << 3))	{	//	'which_in' must be in range of numof inputs
		input_test_error_flags |= 1;
		return	false;
	}
	if	((how_many < 1) || (how_many > 31))	{	//	test for range error
		input_test_error_flags |= 2;
		return	false;
	}
	if	((how_many & 1) == 0)	{	//	check for 'how_many' is even, should be odd
		how_many--;					//	smallest even number possible here was 2, round down to next lower odd
		input_test_error_flags |= 4;	//	set error flag but proceed
	}								//	'how_many' now always odd number
	ones_count	= 0;
	test_bit	= 0x40000000L;	//	First bit to test is bit 30
	threshold = how_many / 2;	//	when how_many == 1, threshold == 0
								//	when how_many == 3, threshold == 1 etc
	while	(how_many--)	{
		if	((ins_history[which_input] & test_bit) != 0)
			ones_count++;
		test_bit >>= 1;		//	test bit30 first, next bit 29 etc
	}
	if	(ones_count > threshold)
		return	true;
	return	false;
}

bool	get_input_bit	(uint32_t which_in)	{	//	Read latest with no debounce
	return	false;
}


void	spi_cs_set	()	{	//	Have Checked which pin used on pcb
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void	spi_cs_clr	(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

/*int16_t	onbuff	(UART_HandleTypeDef * huart)	{
	int	rv = 0;
	if	(huart == &huart1)	{
	rv = com1_rx_onptr - com1_rx_offptr;
	if	(rv < 0)
		rv += COM1_RXBUFF_SIZE;
	}
	if	(huart == &huart2)	{
	rv = com2_rx_onptr - com2_rx_offptr;
	if	(rv < 0)
		rv += COM2_RXBUFF_SIZE;
	}
	return	rv;
}*/

bool	getcc	(UART_HandleTypeDef * huart, char * ch)	{
	if	(huart == &huart1)	{
		if	(com1_rx_empty)
			return	false;
		*ch = (char)com1_rxbuff[com1_rx_offptr++];
		if	(com1_rx_offptr >= COM1_RXBUFF_SIZE)
			com1_rx_offptr = 0;
		if	(com1_rx_onptr == com1_rx_offptr)
			com1_rx_empty	= true;
		return	true;
	}
	if	(huart == &huart2)	{
		if	(com2_rx_empty)
			return	false;
		*ch = (char)com2_rxbuff[com2_rx_offptr++];
		if	(com2_rx_offptr >= COM2_RXBUFF_SIZE)
			com2_rx_offptr = 0;
		if	(com2_rx_onptr == com2_rx_offptr)
			com2_rx_empty	= true;
		return	true;
	}
	return	false;
}

bool	get_tx_busy_flag	(UART_HandleTypeDef * huart)	{
	if	(huart == &huart1)
		return	com1_tx_busy;
	if	(huart == &huart2)
		return	com2_tx_busy;
	return	false;
}

void	set_tx_busy_flag	(UART_HandleTypeDef * huart)	{	//	gets cleared to false in HAL_UART_TxCpltCallback(
	if	(huart == &huart1)	{							//	do this immediately before HAL_UART_Transmit_DMA
		com1_tx_busy = true;
	}
	if	(huart == &huart2)	{
		com2_tx_busy = true;
	}
}
CAN_TxHeaderTypeDef	TxHeader;
CAN_RxHeaderTypeDef	RxHeader;

//uint32_t	TxMailbox;

uint8_t	TxData[8];
uint8_t	RxData[8];

uint32_t	cancount = 0;//, stdid[4] = {0};

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)	//	Not this one
{
	cancount++;
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
//	stdid[0] = RxHeader.StdId;
}

void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan)	//	Not this one
{
	cancount++;
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO1, &RxHeader, RxData);
//	stdid[1] = RxHeader.StdId;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)	//	Yes this one
{
	cancount++;
	can_flag = true;
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
//	stdid[2] = RxHeader.StdId;
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)	//	Yes this one
{
	cancount++;
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO1, &RxHeader, RxData);
//	stdid[2] = RxHeader.StdId;
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_SPI1_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  // SPI CS pin should default high
  spi_cs_set	();

  HAL_TIM_Base_Start_IT(&htim6);	//	Need this to start timer

  HAL_UART_Receive_DMA	(&huart1, com1_rxbuff, 1);	//	Kick-start receive mechanisms, interrupt occurs on 1sr char rx'd
  HAL_UART_Receive_DMA	(&huart2, com2_rxbuff, 1);

  HAL_CAN_Start(&hcan1);

  if	(HAL_OK != HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING))
	  cancount++;

  if	(HAL_OK != HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING))
	  cancount++;
  /*
  TxHeader.DLC = 1;	//	Specifies length of data to send, bytes
  TxHeader.ExtId = 0;	//	Norm not extended
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x103;
  TxHeader.TransmitGlobalTime = DISABLE;

  TxData[0] = 0xf3;

  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
*/


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Signal_sys_Setup	();

  while (1)
  {
	  //	do fastest stuff here
	  //	** Why are u1rx and u2rx not treated the same ? **
	  if(u1rx_flag){
		  HAL_UART_Receive_DMA(&huart1, comrxtmp1, 1);
		  u1rx_flag = false;	//		  HAL_UART_Transmit_DMA(&huart1, comrxtmp1, 1);
	  }
	  if(u2rx_flag){	//	Interrupt handler has stored read on circ buff
		  HAL_UART_Receive_DMA(&huart2, com2_rxbuff + com2_rx_onptr, 1);
		  u2rx_flag = false;	//		  HAL_UART_Transmit_DMA(&huart2, comrxtmp2, 1);
	  }
	  if	(spi_TxRx_flag)	{		//	set 'true' in HAL_SPI_TxRxCpltCallback
		  spi_TxRx_flag = false;	//	clr flag to false

//		  ins_history_update	();	//
		  /**
		   * void	ins_history_update	()	{
		   * Run this soon after completion of I/O read/write using HAL_SPI_TransmitReceive
		   * Reads all input bits read in over SPI currently stored in spi_rx[]
		   * Last 31 reads of each input line are stored in bits 30 downto 0 of uint32_t ins_history[numbered]
		   * most recent bit in bit 30 so that read of ins_history[n] can be used compared to some threshold
		   */
//		  void	ins_history_update	()	{
		  	int	input_number = 0;
		  	uint8_t		j;
		  	for	(int i = 0; i < IO_DAISY_CHAIN_BYTE_LEN; i++)	{
		  		j = 1;
		  		while	(j)	{
		  			ins_history[input_number] >>= 1;	//	Oldest input reads pushed right to lsbs
		  			if	((spi_rx[i] & j) != 0)
		  				ins_history[input_number] |= 0x40000000L;	//	set bit 30
		  			j <<= 1;	//	do for bit positions 0 to 7
		  			input_number++;
		  		}
		  	}
//		  }

		  	//	Input updating now complete

		  ForeverLoop	();			//	Go here asap after spi completes
	  }		//	End of if (spi_TxRx_flag)

	  if(timer_1ms_flag)	{
		  timer_1ms_flag = false;	//	timers updated in interrupt handler
//		  millisecs++;		//	Global count of milli seconds since power-up
//		  forever_loop_timer++;
//		  slow_loop_timer++;

		  if	(forever_loop_timer >= FOREVER_LOOP_REPEAT_MS)
		  {
			  forever_loop_timer -= FOREVER_LOOP_REPEAT_MS;	//	This is where SPI comms is initiated.
			  	  	  	  	  	  	  	  	  	  	  	  	//	Will take some time to complete.
			  spi_cs_clr();	//	SPI Chip select active low

			  spi_tx[0] = 0x55;		//	Just for something for the scope to see

			  spi_cs_set();	//	SPI Chip select hi again as also connected to HC597 PL\ as well as STCP.
			  //	STCP is __-- edge trig, PL\ is async.  Therefore, to get latest data, need TWO
			  //	--_--_--. First clocks latest pin data into input registers.
			  //	Second --_-- clock transfers latest pin data to output shift register.
			  __NOP();
//			  wait_us	(2);
			  spi_cs_clr();	//	SPI Chip select active low
			  __NOP();
			  spi_cs_set();	//	SPI Chip select active low

			  HAL_SPI_TransmitReceive_IT	(&hspi1, spi_tx, spi_rx, IO_DAISY_CHAIN_BYTE_LEN);
			  	  //	Once SPI comms has completed, Interrupt handler sets spi_TxRx_flag = true;
			  	  //	Test spi_TxRx_flag in fast area above to call ForeverLoop()
/*			  spi_tx[0]++;
			  if	(spi_tx[0] == 0)	{
				  spi_tx[1]++;
				  if	(spi_tx[1] == 0)	{
					  spi_tx[2]++;
					  if	(spi_tx[2] == 0)
						  spi_tx[3]++;
				  }
			  }*/
		  }
		  if	(slow_loop_timer >= SLOW_LOOP_REPEAT_MS)
		  {
			  slow_loop_timer -= SLOW_LOOP_REPEAT_MS;
//		  }
//			  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//			  len = sprintf	((char*)txt, "onbuf1 %d, onbuf2 %d, onptr %ld, "
//					  "txbusycnt %d, spirx %d, rxd %d %d, TxRx_flag %d\r\n",
//					  onbuff(&huart1), onbuff(&huart2), com2_rx_onptr, f[2], f[6], spi_rx[0], spi_rx[1], f[7]);
/*			  if	(onbuff2() > 15)	{
				  tmp[0] = (int8_t)get2();
				  tmp[1] = (int8_t)get2();
				  tmp[2] = (int8_t)get2();
				  tmp[3] = '\r';
				  tmp[4] = '\n';
//				  HAL_UART_Transmit_DMA	(&huart2, tmp, 5);
			  }*/
//			  HAL_UART_Transmit_DMA	(&huart2, txt, len);

		  }	//	End of if	(slow_loop_timer >= SLOW_LOOP_REPEAT_MS)
		  one_ms_update_stuff	();
	  }	//	End of if(timer_1ms_flag)	{
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_LOOPBACK;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef	canfilterconfig;	//	Need to configure filters here
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 1;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x0446 << 5;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x0444 << 5;	//	'1' bit to compare filter bit to incoming bit
  canfilterconfig.FilterMaskIdLow = 0x0000;			//	'0' do not test bit in this position
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
//  canfilterconfig.SlaveStartFilterBank = 26;	//	Meaningless as only 1 CAN on L432KC
  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  canfilterconfig.FilterBank = 6;
  canfilterconfig.FilterIdHigh = 0x0123 << 5;
  canfilterconfig.FilterMaskIdHigh = 0x0123 << 5;
  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  canfilterconfig.FilterBank = 10;
  canfilterconfig.FilterIdHigh = 0x0700 << 5;
  canfilterconfig.FilterMaskIdHigh = 0x0700 << 5;
  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 32 - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000 - 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart2.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//	Callback: timer has reset
void	HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{	//	check for timer 6
	if	(htim == &htim6)	{
		timer_1ms_flag = true;
		millisecs++;		//	Global count of milli seconds since power-up
		forever_loop_timer++;
		slow_loop_timer++;
	}
}

/*void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)	{
}
*/

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)	{
	if(huart == &huart1)	{
		u1rx_flag = true;	//	Flag triggers HAL_UART_Receive_DMA(&huart1, rxtmp, 1); OUT of interrupt context
		com1_rx_empty	= false;
		//	move to circ buff here
		com1_rx_onptr++;
		if	(com1_rx_onptr >= COM1_RXBUFF_SIZE)
			com1_rx_onptr = 0;
		if	(com1_rx_onptr == com1_rx_offptr)
			com1_rx_full	= true;			//	onptr now ready for HAL_UART_Receive_DMA()
	}
	if(huart == &huart2)	{
		f[1]++;
		u2rx_flag = true;
		com2_rx_empty	= false;
		//	move to circ buff here
		com2_rx_onptr++;
		if	(com2_rx_onptr >= COM2_RXBUFF_SIZE)
			com2_rx_onptr = 0;
		if	(com2_rx_onptr == com2_rx_offptr)
			com2_rx_full	= true;
//		HAL_UART_Receive_DMA(&huart2, comrxtmp2, 1);	Doesn't work from interrupt context
	}
}

//void HAL_UART_DMATxCpltCallback(UART_HandleTypeDef *huart)	//	This called as well as HalfCplt
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)	//	This called as well as HalfCplt
{
	if(huart == &huart1)	{
		com1_tx_busy = false;
//		&huart1->gState = HAL_UART_STATE_READY;
	}
//    CLEAR_BIT(huart->Instance->CR1, USART_CR1_TXEIE);

	if(huart == &huart2)	{
		com2_tx_busy = false;
	}
//	huart->gState = HAL_UART_STATE_READY;
}

/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)	//	This called as well as HalfCplt
{
}
*/

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	f[4]++;
}


// This is called when SPI transmit is done. Should not happen as using TransmitReceive_IT
void HAL_SPI_TxCpltCallback (SPI_HandleTypeDef * hspi)
{
	spi_cs_clr();	//	SPI Chip select active low
	__NOP();
	spi_cs_set();	//	SPI Chip select active low This clock updates HC595 storage registers
	spi_error_flag = true;
}

// This is called when SPI receive is done. Should not happen as using TransmitReceive_IT
void HAL_SPI_RxCpltCallback (SPI_HandleTypeDef * hspi)
{
	spi_cs_set();
	spi_error_flag = true;
}

void HAL_SPI_TxRxCpltCallback (SPI_HandleTypeDef * hspi)
{
	spi_cs_set();			//	SPI chip select line
	spi_TxRx_flag = true;
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
