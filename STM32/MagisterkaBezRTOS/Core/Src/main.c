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
#include "stdio.h"
#include "ST7735.h"
#include "GFX_FUNCTIONS.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

uint16_t Flaga, Vtryb = 0;
char result[10];
double Val ;
float tryb;

double CR = 9910;
double PI = 3.14159265358979323846264338328;
double c = 0.995E-6;

double ref = 0;
double rx[7];
float r7[]={507.4, 2173.0, 5053.0, 10020.0, 100900.0, 468600.0, 1003000.0};
int ind = 0;


int controlPin[] = {S0_Pin, S1_Pin, S2_Pin};
    int muxChannel[8][3]={
      {0,0,0}, //kanał 0
      {1,0,0}, //kanał 1
      {0,1,0}, //kanał 2
      {1,1,0}, //kanał 3
      {0,0,1}, //kanał 4
      {1,0,1}, //kanał 5
      {0,1,1}, //kanał 6
      {1,1,1}, //kanał 7
    };



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ADC_Select_Rref ();
void ADC_Select_Rread ();
void ADC_Select_ModePin ();
void ADC_Select_Cread ();
void ADC_Select_Cout ();


void ADC_SetActiveChannel(ADC_HandleTypeDef *hadc, uint32_t AdcChannel);


void deactivate_perif();
void Vref();
void delay_us (uint16_t us);
uint16_t pulse_in (GPIO_TypeDef *port, uint16_t pin, uint16_t timeout);
void set_gpio(GPIO_TypeDef *port, uint16_t pin, uint32_t Mode, uint32_t Pull );
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
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim2);

  ST7735_Init(0);
  fillScreen(BLACK);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  uint8_t oldFlaga;


	  ADC_Select_ModePin();

	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 10);
	  Vtryb = HAL_ADC_GetValue(&hadc1);
	  HAL_ADC_Stop(&hadc1);

	  tryb = (Vtryb*3.3)/4095;

	  if(tryb<0.1){
	  	  Flaga = 1;
	   }
	  else if(tryb < 1.3){
		  Flaga = 2;
	  }
	  else if(tryb < 2.3){
		  Flaga = 3;
	  }
	  else if(tryb < 3.5){
		  Flaga = 4;
	  }

	  if(Flaga != oldFlaga){
		oldFlaga = Flaga;

	  	fillScreen(BLACK);
	  	deactivate_perif();
	  }

	  if(Flaga == 1){

		  ST7735_WriteString(7, 35, "MIERNIK", Font_16x26, WHITE, BLACK);
		  ST7735_WriteString(35, 80, "RLC", Font_16x26, WHITE, BLACK);

	  }
	  else if(Flaga == 2){
		  Vref();
		  ST7735_WriteString(25, 10, "REZYSTANCJA", Font_7x10, WHITE, BLACK);

		  set_gpio(GPIOB, S2_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL );
		  set_gpio(GPIOB, S1_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL );
		  set_gpio(GPIOB, S0_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL );
		  set_gpio(GPIOA, Rread_Pin, GPIO_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL );
		  set_gpio(GPIOA, Rref_Pin, GPIO_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL );
		  HAL_Delay(100);

		  double Uwe = (ref * 3.3/4095.0) * ((9990.0+10060.0)/10060.0);
		  ADC_Select_Rread();
		  for(int i = 1; i<8; i++){

			  for(int j = 0; j < 3; j ++){
		          HAL_GPIO_WritePin(GPIOB, controlPin[j], muxChannel[i][j]);
		      }
			  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
		      HAL_Delay(100);
		      HAL_ADC_Start(&hadc1);
		      HAL_ADC_PollForConversion(&hadc1, 10);
		      rx[i-1] = HAL_ADC_GetValue(&hadc1);
		      HAL_ADC_Stop(&hadc1);

		  }

		  ind = 6;
		  for(int i = 7; i >= 0; i--){
			  if(rx[i]>2048){
				  ind=i;
			  }
		  }
		  if(ind == 0){
			  if(rx[0] < 3600){
			  	  ind = 1;
		  	  }
		  }
		  else if(ind == 1){
			  if(rx[1] < 3200){
			  	  ind = 2;
		  	  }
		  }
		  else if(ind == 2){
			  if(rx[2] < 2600){
			  	  ind = 3;
		  	  }
		  }

		  Val = ((Uwe*r7[ind])/(rx[ind]* (3.3 / 4095.0)))-r7[ind];
		  double Val0 = ((Uwe*r7[0])/(rx[0]* (3.3 / 4095.0)))-r7[0];
		  double Val1 = ((Uwe*r7[1])/(rx[1]* (3.3 / 4095.0)))-r7[1];
		  double Val2 = ((Uwe*r7[2])/(rx[2]* (3.3 / 4095.0)))-r7[2];
		  double Val3 = ((Uwe*r7[3])/(rx[3]* (3.3 / 4095.0)))-r7[3];
		  double Val4 = ((Uwe*r7[4])/(rx[4]* (3.3 / 4095.0)))-r7[4];
		  double Val5 = ((Uwe*r7[5])/(rx[5]* (3.3 / 4095.0)))-r7[5];
		  double Val6 = ((Uwe*r7[6])/(rx[6]* (3.3 / 4095.0)))-r7[6];

		  if(Val >2000000 || Val < 1){
			  ST7735_WriteString(10, 50, " -----      ", Font_16x26, WHITE, BLACK);
			  ST7735_WriteString(35, 105, "Ohm  ", Font_16x26, WHITE, BLACK);
		  }

		  else if (Val < 1000){
			  int v1,v2;
			  v1 = (int)Val;
			  v2 = (Val - (int)Val)*10;

			  sprintf(result, "%d.%d     ", v1, v2);
			  ST7735_WriteString(25, 50, result, Font_16x26, WHITE, BLACK);
			  ST7735_WriteString(35, 105, "Ohm  ", Font_16x26, WHITE, BLACK);
		  }
		  else if (Val < 100000){

			  sprintf(result, "%d    ", (int)Val);
			  ST7735_WriteString(25, 50, result, Font_16x26, WHITE, BLACK);
			  ST7735_WriteString(35, 105, "Ohm  ", Font_16x26, WHITE, BLACK);
		  }

		  else if (Val<1000000){
			  Val /=1000;
			  int v1,v2;
			  v1 = (int)Val;
			  v2 = (Val - v1)*10;

			  sprintf(result, "%d.%d     ", v1, v2);
			  ST7735_WriteString(25, 50, result, Font_16x26, WHITE, BLACK);
			  ST7735_WriteString(35, 105, "KOhm  ", Font_16x26, WHITE, BLACK);
		  }
		  else {

			  Val /=1000000;
			  int v1,v2,v3,v4;
			  v1 = (int)Val;
			  v2 = (Val - v1)*10;
			  v3 = (Val - v1)*100-v2;
			  v4 = (Val - v1)*1000-v3;
			  sprintf(result, "%d.%d%d%d     ", v1, v2,v3,v4);
			  ST7735_WriteString(25, 50, result, Font_16x26, WHITE, BLACK);
			  ST7735_WriteString(35, 105, "MOhm  ", Font_16x26, WHITE, BLACK);

		  }


	  }
	  else if(Flaga == 3){
		  Vref();
		  double c63=ref*2*0.632;
		  ST7735_WriteString(30, 10, "POJEMNOSC", Font_7x10, WHITE, BLACK);

		  set_gpio(GPIOA, Cout_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL );
		  set_gpio(GPIOA, Ccharge_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL );
		  set_gpio(GPIOA, Cdischarge_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL );
		  set_gpio(GPIOA, Cread_Pin, GPIO_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL );

		  HAL_GPIO_WritePin(GPIOA, Cdischarge_Pin, 0);
		  HAL_GPIO_WritePin(GPIOA, Cout_Pin, 0);
		  HAL_GPIO_WritePin(GPIOA, Ccharge_Pin, 0);

		  ADC_Select_Cread();

		  uint16_t tmp = 4000;
		  HAL_ADC_Start(&hadc1);
		  while(tmp > 20){		//rozładowanie kondensatora
			  HAL_ADC_PollForConversion(&hadc1, 10);
			  tmp = HAL_ADC_GetValue(&hadc1);
		  }

		  HAL_GPIO_WritePin(GPIOA, Ccharge_Pin, 1);
		  __HAL_TIM_SET_COUNTER(&htim2,0);	//zerowanie timera

		  while(tmp < c63){
			  HAL_ADC_PollForConversion(&hadc1, 10);
			  tmp = HAL_ADC_GetValue(&hadc1);
		  }

		  unsigned long elapsedTime = __HAL_TIM_GET_COUNTER(&htim2);
		  HAL_ADC_Stop(&hadc1);

		  Val = ((float)elapsedTime / (float)CR);

		  if(Val<1){	//jeśli wartość jest poniżej 1uF

			  Val *= 1000;	//zamiana na nF

			  if(Val <90){  //male wartości poniżęj 90nF

			  	set_gpio(GPIOA, Ccharge_Pin, GPIO_MODE_INPUT, GPIO_NOPULL );
			  	set_gpio(GPIOA, Cdischarge_Pin, GPIO_MODE_INPUT, GPIO_NOPULL );

			  	set_gpio(GPIOA, Cread_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL );
			  	HAL_Delay(1);
			  	set_gpio(GPIOA, Cout_Pin, GPIO_MODE_INPUT, GPIO_PULLUP );
			  	unsigned long t;
			  	int digVal;
			  	__HAL_TIM_SET_COUNTER(&htim2,0);
			  	do{
			  		digVal = HAL_GPIO_ReadPin(GPIOA, Cout_Pin);
			  		t = __HAL_TIM_GET_COUNTER(&htim2);
			  	}
			  	while ((digVal < 1) && (t < 400000L));
			  	set_gpio(GPIOA, Cout_Pin, GPIO_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL );

			  	ADC_Select_Cout();
			  	HAL_ADC_Start(&hadc1);
			  	HAL_ADC_PollForConversion(&hadc1, 10);
			  	Val = HAL_ADC_GetValue(&hadc1);
			  	HAL_ADC_Stop(&hadc1);

		  		HAL_GPIO_WritePin(GPIOA, Cread_Pin, 1);
		  		int dischargeTime = (int)(t / 1000L) * 5;
		  		HAL_Delay(dischargeTime);
		  		set_gpio(GPIOA, Cout_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL );
		  		HAL_GPIO_WritePin(GPIOA, Cread_Pin, 0);
		  		HAL_GPIO_WritePin(GPIOA, Cout_Pin, 0);
		  		float cap = (-(float)t / 39.8) / (log(1.0 - (float)Val / (float)4095));

		  		if(cap>=1){
		  			int v1,v2,v3;
		  			v1 = (int)cap;
		  			v2 = (cap - (int)cap)*10;
		  			v3 = (cap - (int)cap)*100;


		  			sprintf(result, "%d.%d%d     ", v1, v2,v3);
		  			ST7735_WriteString(20, 50, result, Font_16x26, WHITE, BLACK);
		  			ST7735_WriteString(14, 105, "  nF  ", Font_16x26, WHITE, BLACK);

		        }
		  		else if(cap>0.15){

		  			cap *=1000;
		  			sprintf(result, " %d     ", (int)cap);
		  			ST7735_WriteString(20, 50, result, Font_16x26, WHITE, BLACK);
		  			ST7735_WriteString(14, 105, "  pF  ", Font_16x26, WHITE, BLACK);
		  			HAL_Delay(200);

		  		}
		        else{

		  			ST7735_WriteString(10, 50, " -----      ", Font_16x26, WHITE, BLACK);
		  	  	  	ST7735_WriteString(19, 105, "  F  ", Font_16x26, WHITE, BLACK);

		        }
			  }//koniec małe watości

			  else{
				  int v1,v2,v3;
				  v1 = (int)Val;
				  v2 = (Val - (int)Val)*10;
				  v3 = (Val - (int)Val)*100;


				  sprintf(result, "%d.%d%d     ", v1, v2,v3);
			  	  ST7735_WriteString(10, 50, result, Font_16x26, WHITE, BLACK);
			  	  ST7735_WriteString(14, 105, "  nF  ", Font_16x26, WHITE, BLACK);
			  }
		  }
		  else{
			  int v1,v2;
			  v1 = (int)Val;
			  v2 = (Val - (int)Val)*10;


			  sprintf(result, "%d.%d     ", v1, v2);
			  ST7735_WriteString(10, 50, result, Font_16x26, WHITE, BLACK);
			  ST7735_WriteString(14, 105, "  uF  ", Font_16x26, WHITE, BLACK);
		  }

	  }

	  else if(Flaga == 4){

		  ST7735_WriteString(20, 10, "INDUKCYJNOSC", Font_7x10, WHITE, BLACK);

		  set_gpio(GPIOA, PulseIN_Pin, GPIO_MODE_INPUT, GPIO_NOPULL );
		  set_gpio(GPIOB, PulseOUT_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL );

	  	  HAL_GPIO_WritePin(GPIOB, PulseOUT_Pin, 1);
	  	  HAL_Delay(5);
	  	  HAL_GPIO_WritePin(GPIOB, PulseOUT_Pin, 0);
	  	  delay_us(50);
	  	  double puls = pulse_in(GPIOA, PulseIN_Pin , 5000);
	  	  double freq;

	  	  if(puls > 0.1){

	  		  freq = 1.E6/(2*puls);  //zamiana na częstotliwość
	  		  Val = 1/(4*c*freq*freq*PI*PI);
	  		  Val *= 1.0E6;

	  	  }
	  	  else{
	  		  Val = 0;
	  	  }

	  	  if(Val>0.1){

	  		  if(Val<10000){

	  			  sprintf(result, " %d     ", (int)Val);
	  			  ST7735_WriteString(20, 50, result, Font_16x26, WHITE, BLACK);
	  	  	  	  ST7735_WriteString(14, 105, "  uH  ", Font_16x26, WHITE, BLACK);

	  		  }

	  	  }
	  	  else{

	  		  ST7735_WriteString(10, 50, " -----      ", Font_16x26, WHITE, BLACK);
	  	  	  ST7735_WriteString(19, 105, "  H  ", Font_16x26, WHITE, BLACK);

	  	  }

	  }

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}
void Vref	(void)
{
	deactivate_perif();
	set_gpio(GPIOB, S2_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL );
	set_gpio(GPIOB, S1_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL );
	set_gpio(GPIOB, S0_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL );
	set_gpio(GPIOA, Rref_Pin, GPIO_MODE_ANALOG_ADC_CONTROL, GPIO_NOPULL );

	HAL_Delay(50);

	for(int j = 0; j < 3; j ++){

		HAL_GPIO_WritePin(GPIOB, controlPin[j], muxChannel[0][j]);
	}

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_Delay(50);
	ADC_Select_Rref ();

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	ref = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	deactivate_perif();
}

void ADC_Select_Rref (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_5;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}


void ADC_Select_Rread (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_ModePin (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    //Error_Handler();
	  }
}

void ADC_Select_Cout (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_8;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}
void ADC_Select_Cread (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_10;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}
void deactivate_perif(void)
{
	set_gpio(GPIOA, Cout_Pin|Cdischarge_Pin|Ccharge_Pin|Cread_Pin|PulseIN_Pin, GPIO_MODE_INPUT, GPIO_NOPULL );
	set_gpio(GPIOB, S2_Pin|S1_Pin|S0_Pin|PulseOUT_Pin, GPIO_MODE_INPUT, GPIO_NOPULL );
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 36;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  //ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV128;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_1LINE;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Cout_Pin Ccharge_Pin CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PulseOUT_Pin S2_Pin DC_Pin */
  GPIO_InitStruct.Pin = DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */


void ADC_SetActiveChannel(ADC_HandleTypeDef *hadc, uint32_t AdcChannel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = AdcChannel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
  {
   Error_Handler();
  }
}

void delay_us ( uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim2,0);  				// set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim2) < us);  	// wait for the counter to reach the us input in the parameter
}


uint16_t pulse_in (GPIO_TypeDef *port, uint16_t pin, uint16_t timeout)
{
	uint16_t tmp;
	__HAL_TIM_SET_COUNTER(&htim2,0);
	while(HAL_GPIO_ReadPin(port, pin) == 1){
		if(__HAL_TIM_GET_COUNTER(&htim2) > timeout) return 0;
	}


	while(HAL_GPIO_ReadPin(port, pin) == 0);
	__HAL_TIM_SET_COUNTER(&htim2,0);  // set the counter value a 0


	while (__HAL_TIM_GET_COUNTER(&htim2) < timeout){
		if(HAL_GPIO_ReadPin(port, pin) == 0){
			tmp = __HAL_TIM_GET_COUNTER(&htim2);
			break;
		}
		tmp = 0;
	}
	return tmp;
}
void set_gpio(GPIO_TypeDef *port, uint16_t pin, uint32_t Mode, uint32_t Pull ){

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Mode = Mode;
	GPIO_InitStruct.Pull = Pull;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(port, &GPIO_InitStruct);

}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

