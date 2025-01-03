/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

#define RING_BUF_SIZE	60

typedef volatile struct RingBuffer {
	char buf[RING_BUF_SIZE];
	int  i_push;		/* pointeur (index) d'écriture */
	int  i_pop;		/* pointeur (index) de lecture */
} RingBuffer;

static RingBuffer rx_buf = {
	.i_push=0,
	.i_pop=0
};

#define buffSize 		100

typedef volatile struct Buffer {
	int buff[buffSize][3];
	int index;
	int size;
} Buffer;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_SIZE 		1
#define txBufSendLEN 	1000 	//Taille buffer d'envoi des données
#define txBufReceiveLEN 10 		//Taille buffer de reception des données
#define ANGLE_MIN 0
#define ANGLE_MAX 180
#define CCR_MIN 6.5
#define CCR_MAX 26
#define NUM_VALUES 20 			// Nombre de valeurs utilisées pour le moyennage
#define GF 2.1  				//Gauge Factor
#define V_ref 5  				//Tension de référence 5V
#define E 69000000000			//module de Young de la lame en aluminium
#define L 0.3				// longueur de la poutre en m
#define B 0.03           		// largeur de la poutre en m
#define H 0.002                 // epaisseur de la poutre en m
#define I 0.00000000002			// moment quadratique de la poutre en m^4 (B*H^3)/12
#define OFFSET 0.012
#define G 9.81                	//pesanteur en m/s^2
#define Y 0.007               	// Position de la jauge en m
#define Z 0.011 				// Distance entre l'axe neutre et la jauge en m
uint16_t VmValues[NUM_VALUES]; // Initialisation des valeurs à 0
uint8_t VmIndex = 0; // Indice pour ajouter les nouvelles valeurs dans le tableau
uint16_t Vm;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//Buffers
char txBufSend[txBufSendLEN];    //envoi des données
char txBufReceive[txBufReceiveLEN];    //reception des données
volatile uint32_t adcResultsDMA[ADC_SIZE];    //récupération des valeur du DMA
int mode=1, servoMode=0, servoPos=0;


char commande[3][33];			//3 listes de 19 char

Buffer bufferCom = {    //Buffer d'écriture permettant de ne pas envoyer des données en interruptions
		.index = 0,
		.size = buffSize
};
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	  HAL_UART_Receive_IT(&huart2, (uint8_t *)txBufReceive, txBufReceiveLEN);
}*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
int put_char(char);
void term_printf_stlink(const char* fmt, ...);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
float servo_writeAngle(int angle);
void computeData(const char* data);
int forceToAngle(float force);
float calculateMasse(float v);
/* USER CODE END PFP */
#define per_deg 5.55  // pulse for 1° Rotation
uint16_t Vm;
char msg[555];
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  HAL_MspInit();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  //MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  float voltage;
  float masse;
  float deformation;
  float force;

  while (1) {
      HAL_ADC_Start(&hadc1);
      //Attends la fin de la conversion pour Vm
      HAL_ADC_PollForConversion(&hadc1, 100);
      // Lit la valeur brute de l'ADC (12 bits)
      Vm = HAL_ADC_GetValue(&hadc1);
      // Conversion de la valeur brute d'ADC en tension
      voltage = ((float)Vm * 3.3 / 4095.0);
      // Arrête l'ADC pour Vm
      HAL_ADC_Stop(&hadc1);

      // Affichage de la tension sur l'IHM
      sprintf(msg,"V:%f\r\n",voltage);
      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

      // Calculer la masse
      masse = calculateMasse(voltage); // en g
	  // Affichage de la masse sur l'IHM
      sprintf(msg,"M:%f\r\n",masse);
      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

      // Calculer la déformation
      deformation = (masse*0.001*G*L)/ (2*E*I);
	  // Affichage de la déformation sur l'IHM
      sprintf(msg,"D:%f\r\n",deformation);
      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

      //Calcul de la force
      force = masse*0.001*G;
	  // Affichage de l'effort sur l'IHM
      sprintf(msg,"F:%f\r\n",force);
      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);


      HAL_Delay(1000);

	    if (mode == 1) {  // Mode manuel
	        float duty = servo_writeAngle(servoPos);  // Calcul du duty cycle
	        htim2.Instance->CCR1 = duty;  // Appliquer le duty cycle

	        // Envoyer l'angle actuel via UART pour synchronisation avec le frontend
	        //term_printf_stlink("Mode: 1, Angle: %d \r\n", servoPos);

	    }
	    else if (mode == 2) {  // Mode automatique
	          // Calculer l'angle du servomoteur en fonction de la force
	          int angleAuto = forceToAngle(force);

	          // Calculer le duty cycle en fonction de l'angle
	          float duty = servo_writeAngle(angleAuto);
	          htim2.Instance->CCR1 = duty;  // Appliquer le duty cycle

	          // Envoyer l'angle actuel via UART pour la synchronisation avec le frontend
	          term_printf_stlink("%d\r\n", angleAuto);
	    }
  }
}
float calculateMasse(float v){
	float masse;
	if(v>=0.61){
		masse=0;
	}
	else if(v>=0.60 && v<0.61){
		masse=5;
	}
	else if(v>=0.59 && v<0.60){
		masse=10;
	}
	else if(v>=0.586 && v<0.59){
		masse=20;
	}
	else if(v>=0.55 && v<0.585){
		masse=50;
	}
	else if(v>=0.44 && v<=0.48){
		masse=100;
	}
	else if(v<0.48){
		masse=100;
	}
	return masse;

}

int forceToAngle(float force){
    float forceMin = 0.0;
    float forceMax = 0.981;  // Force maximale en N (avec une masse de 100 g)
    float angleMin = ANGLE_MIN;  // Angle minimal du servomoteur
    float angleMax = ANGLE_MAX;  // Angle maximal du servomoteur

    // Vérification que la force est dans la plage valide
    if (force < forceMin) force = forceMin;
    if (force > forceMax) force = forceMax;

    // Mapper la force à l'angle
    float angle = angleMin + ((force - forceMin) / (forceMax - forceMin)) * (angleMax - angleMin);
    return (int)angle;  // Retourne l'angle calculé
}


float servo_writeAngle(int angle) {
    if (angle < ANGLE_MIN) angle = ANGLE_MIN;
    if (angle > ANGLE_MAX) angle = ANGLE_MAX;
    float duty = CCR_MIN + (((float)(angle - ANGLE_MIN) / (ANGLE_MAX - ANGLE_MIN)) * (CCR_MAX - CCR_MIN));
    return duty;
}

void computeData(const char* data) {
    char *token = strtok(data, ",");  // Découpage de la commande reçue
    int i = 0;
    while (token) {
        strcpy(commande[i], token);
        i++;
        token = strtok(NULL, ",");
    }

    mode = atoi(commande[0]);  // Premier paramètre : mode

    if (i > 1) {  // Deuxième paramètre : angle manuel
        int angle = atoi(commande[1]);
        if (angle >= ANGLE_MIN && angle <= ANGLE_MAX) {
            servoPos = angle;
        }
    }

    // Optionnel : Envoyer un accusé de réception
    //term_printf_stlink("Mode: %d, Angle: %d \r\n", mode, servoPos);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1599;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 200;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_UART_Receive_IT(&huart2, (uint8_t *)txBufReceive, 1);
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

	GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* GPIO Ports Clock Enable */
	 __HAL_RCC_GPIOA_CLK_ENABLE();
  	 GPIO_InitStruct.Pin = GPIO_PIN_5;
     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
     GPIO_InitStruct.Pull = GPIO_NOPULL;
     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
     GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int put_char(char ch)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
}


int putchar_stlink(char ch)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
	return 0;
}

//=================================================================
//	Helper Functions and Buffer Management
//=================================================================

// Add data to the buffer
static void add_to_buffer(char data) {
    // Check if buffer is not full
    if ((rx_buf.i_push + 1) % RING_BUF_SIZE != rx_buf.i_pop) {
        // Add character to the buffer
        rx_buf.buf[rx_buf.i_push] = data;
        // Increment write index
        rx_buf.i_push = (rx_buf.i_push + 1) % RING_BUF_SIZE;
    }
}

// Clear the buffer
static void clear_buffer() {
    // Reset both read and write indices
    rx_buf.i_push = 0;
    rx_buf.i_pop = 0;
}

void put_string_stlink(char* s)
{
	while(*s != '\0')
	{
		putchar_stlink(*s);
		s++;
	}
}

void term_printf_stlink(const char* fmt, ...)
{
	va_list        ap;
	char          *p;
	char           ch;
	unsigned long  ul;
	unsigned long long ull;
	unsigned long  size;
	unsigned int   sp;
	char           s[34];
	int 		 first=0;


	va_start(ap, fmt);
	while (*fmt != '\0') {
		if (*fmt =='%') {
			size=0; sp=1;
			if (*++fmt=='0') {fmt++; sp=0;}	// parse %04d --> sp=0
			ch=*fmt;
			if ((ch>'0') && (ch<='9')) {	// parse %4d --> size=4
				char tmp[10];
				int i=0;
				while ((ch>='0') && (ch<='9')) {
					tmp[i++]=ch;
					ch=*++fmt;
				}
				tmp[i]='\0';
				size=str2num(tmp,10);
			}
			switch (ch) {
				case '%':
					 putchar_stlink('%');
					break;
				case 'c':
					ch = va_arg(ap, int);
					putchar_stlink(ch);
					break;
				case 's':
					p = va_arg(ap, char *);
					put_string_stlink(p);
					break;
				case 'd':
					ul = va_arg(ap, long);
					if ((long)ul < 0) {
						putchar_stlink('-');
						ul = -(long)ul;
						//size--;
					}
					num2str(s, ul, 10, size, sp);
					put_string_stlink(s);
					break;
				case 'u':
					ul = va_arg(ap, unsigned int);
					num2str(s, ul, 10, size, sp);
					put_string_stlink(s);
					break;
				case 'o':
					ul = va_arg(ap, unsigned int);
					num2str(s, ul, 8, size, sp);
					put_string_stlink(s);
					break;
				case 'p':
					putchar_stlink('0');
					putchar_stlink('x');
					ul = va_arg(ap, unsigned int);
					num2str(s, ul, 16, size, sp);
					put_string_stlink(s);
					break;
				case 'x':
					ul = va_arg(ap, unsigned int);
					num2str(s, ul, 16, size, sp);
					put_string_stlink(s);
					break;
				case 'f':
				    // Récupérer l'argument au format IEEE 754 (32 bits)
				    unsigned long ull = va_arg(ap, unsigned long);

				    // Extraire le signe, l'exposant, et la mantisse
				    int sign = (ull & 0x80000000) >> 31; // Bit de signe
				    int e = (ull & 0x7F800000) >> 23;    // Exposant (biaisé de 127)
				    int m = (ull & 0x007FFFFF);          // Mantisse

				    // Normaliser la mantisse
				    float mf = (float)m / 8388608.0; // Division par 2^23
				    mf += 1.0; // Ajout de 1 pour la normalisation

				    // Calcul de la valeur finale
				    e -= 127; // Suppression du biais de l'exposant
				    // Calcul de la puissance de 2 manuellement
				    float factor = 1.0;
				    if (e > 0) {
				        for (int i = 0; i < e; i++) {
				            factor *= 2.0; // Multiplication par 2 pour e > 0
				        }
				    } else if (e < 0) {
				        for (int i = 0; i < -e; i++) {
				            factor /= 2.0; // Division par 2 pour e < 0
				        }
				    }

				    // Calcul de la valeur réelle
				    float f = mf * factor; // Mantisse * 2^e
				    if (sign == 1) f = -f; // Ajout du signe

				    // Conversion en chaîne avec précision de 5 chiffres après la virgule
				    float2str((char *)s, f, 5);

				    // Afficher la chaîne résultante
				    put_string_stlink((char *)s);
				    break;
				default:
					putchar_stlink(*fmt);
			}
		} else putchar_stlink(*fmt);
		fmt++;
	}
	va_end(ap);
}

//fonction callback de réception de données sur le port série
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	HAL_UART_Receive_IT(&huart2, (uint8_t *)txBufReceive, 1);
	char car_received= (char)txBufReceive[0];
	//char 'e' mean end of commande
	if(car_received == 'e'){
		rx_buf.buf[rx_buf.i_push] = '\0';
		computeData(rx_buf.buf);
		clear_buffer();
	}else{
		add_to_buffer(car_received);
	}
}


//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//  HAL_UART_Receive_IT(&huart2, (uint8_t *)txBufReceive, txBufReceiveLEN);//fonction de communication
//  computeData();
//
//}


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
