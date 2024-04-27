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
#include "transmit_function.h"
#include "decoding_frame.h"
#include "Robot_odom.h"
#include "string.h"
#include "pid_module.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t enc_buff_right[4];
uint8_t enc_buff_left[4];
uint8_t Rx;// chỉ lưu 1byte
uint8_t uart_buff[50];
int uart_flag = 0;
uint8_t uart_len = 0;

int32_t encoder_value_left = 0;
int32_t encoder_value_right = 0;
int32_t pre_encoder_value_left = 0;
int32_t pre_encoder_value_right = 0;
int32_t encoder_difference_left  = 0;
int32_t encoder_difference_right = 0;

uint8_t encoder_buf[30];
char data[100] ="";
uint8_t data_right[11] ="";
uint8_t data_left[11] ="";

int  en_run = 0;
int en_PID = 0;
int count =0;
float phid_pid = 0.0;
float distance_to_goals = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

Robot myRobot;
PID_handleTypedef hpid;
desired_point my_desired_point;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==huart2.Instance){
//		receive_data(Rx);
		if(Rx == '\n'){
			uart_flag =1;
			uart_buff[uart_len++] = Rx;
		}

		else{
			uart_buff[uart_len++]= Rx;
		}

		HAL_UART_Receive_IT(&huart2, &Rx, 1);
	}
}
// todo timer interuppt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM3){
		//doc encoder
		if(count >= 0.1 ){

			encoder_value_left = -1*__HAL_TIM_GET_COUNTER(&htim2);
			encoder_value_right =__HAL_TIM_GET_COUNTER(&htim5);
			encoder_difference_left = encoder_value_left - pre_encoder_value_left;
			encoder_difference_right = encoder_value_right - pre_encoder_value_right;
			pre_encoder_value_right = encoder_value_right ;
			pre_encoder_value_left = encoder_value_left ;
//
			update_Position(&myRobot, encoder_difference_left, encoder_difference_right,0.1);
//			update_Position_base_velocity(&myRobot, encoder_difference_left, encoder_difference_right,0.1);//delta time is 0.1
			//ennable PID calculation
			en_PID =1;

			//end condition
			distance_to_goals =sqrt((myRobot.x-my_desired_point.x_d)*(myRobot.x - my_desired_point.x_d)+(myRobot.y - my_desired_point.y_d)*(myRobot.y-my_desired_point.y_d));
			//sprintf(data,"!distance:%.2f#\n",distance_to_goals);
			//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)data,strlen(data));()abs(myRobot.x - my_desired_point.x_d) < 0.01) &(abs(myRobot.y - my_desired_point.y_d) <0.01
			if((distance_to_goals < 0.03) ){
				HAL_GPIO_WritePin(PE10_EN_DRIVER_GPIO_Port,PE10_EN_DRIVER_Pin,RESET);
				en_run = 0;
			}
			//return encoder value
//			sprintf(data,"!enc_L:%d#encR:%d#\n",encoder_value_left,encoder_value_right);
//			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)data,strlen(data));

			//returm diff encoder value
//			sprintf(data,"!L_diff:%d#R_diff:%d#\n",encoder_difference_left,encoder_difference_right);
//			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)data,strlen(data));

			//return x y phi; !cmd:RUN#x:0.00#y:0.00#phi:0.00#
//			sprintf(data,"!cmd:%s#x:%.2f#y:%.2f#phi:%.2f#\n",myRobot.cmd,myRobot.x,myRobot.y,rad_to_degree(myRobot.theta));
//			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)data,strlen(data));

			//return PWM
//			sprintf(data,"!W_L_PWM:%.2f#W_R_PWM:%.2f#\n",myRobot.v_l_PWM,myRobot.v_r_PWM);
//			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)data,strlen(data));


			if(strcmp(myRobot.cmd,"RUN")== 0 ){
				//return encoder value
	//			sprintf(data,"!enc_L:%d#encR:%d#\n",encoder_value_left,encoder_value_right);
	//			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)data,strlen(data));

				//returm diff encoder value
	//			sprintf(data,"!L_diff:%d#R_diff:%d#\n",encoder_difference_left,encoder_difference_right);
	//			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)data,strlen(data));

//				return x y phi; !cmd:RUN#x:0.00#y:0.00#phi:0.00#
//				sprintf(data,"!cmd:%s#x:%.2f#y:%.2f#phi:%.2f#\n",myRobot.cmd,myRobot.x,myRobot.y,rad_to_degree(myRobot.theta));
//				HAL_UART_Transmit_DMA(&huart2, (uint8_t*)data,strlen(data));


				//return PWM
	//			sprintf(data,"!W_L_PWM:%.2f#W_R_PWM:%.2f#\n",myRobot.v_l_PWM,myRobot.v_r_PWM);
	//			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)data,strlen(data));
				//return Omega
//				sprintf(data,"!omega:%.2f#phi:%.2f#\n",myRobot.omega,rad_to_degree(myRobot.theta));
//				HAL_UART_Transmit_DMA(&huart2, (uint8_t*)data,strlen(data));
			}
//			sprintf(data,"!omega:%.2f#phi:%.2f#\n",myRobot.omega,rad_to_degree(myRobot.theta));
//			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)data,strlen(data));
//			return x y phi; !cmd:RUN#x:0.00#y:0.00#phi:0.00#
			sprintf(data,"!cmd:%s#x:%.2f#y:%.2f#phi:%.2f#\n",myRobot.cmd,myRobot.x,myRobot.y,rad_to_degree(myRobot.theta));
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)data,strlen(data));

			//reset count
			count=0;

		}
		count++;
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();


  /* USER CODE BEGIN 2 */
  //start  uart2 interupt
  // todo
  init_Robot(&myRobot);
  desired_point_init(&my_desired_point);
  HAL_UART_Receive_IT(&huart2, &Rx, 1);
  //ref KP KI KD
//  pid_set_Kp_Ki_Kd(&hpid,1.55, 0.055, 0.025,0.1);
  pid_set_Kp_Ki_Kd(&hpid, 5, 0.13, 0.025,0.1);
  //start PWM TIM1; chanel 1,2 for left motor ; chanel 3,4 for right  motor
  //chanel 1 PE9 ;chanel 2 PE11; chanel 3 PE23 ; chanel 4 PE14;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  //start encoder Mode
  // TIM2 For encoder of left motor , TIM5 for encoder of Right motor
  // TIM2:PA5,PB3 ; TIM5 PA0,PA1
//  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1|TIM_CHANNEL_2);
//  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1|TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1);
  //start interrupt timer 3
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HAL_UART_Transmit(&huart2, tx_buff, sizeof(tx_buff),100);
//	  HAL_Delay(100);
// todo pid

	  // transmit data when stm32 recieved data form PC
	  if(uart_flag == 1){
		  //decoding frame data;
		  split_frame((char*)uart_buff, &my_desired_point);

		  if(strcmp(my_desired_point.cmd_d, "RUN") == 0){
			  strcpy(myRobot.cmd,my_desired_point.cmd_d);
			  HAL_GPIO_WritePin(PE10_EN_DRIVER_GPIO_Port,PE10_EN_DRIVER_Pin,SET);
			  en_run = 1;
		  }
		  else{
			  HAL_GPIO_WritePin(PE10_EN_DRIVER_GPIO_Port,PE10_EN_DRIVER_Pin,RESET);
			  en_run = 0;
		  }
//		  HAL_UART_Transmit_DMA(&huart2, uart_buff, sizeof(uart_buff));


		  //send data to check desire point: !cmd:RUN#x:0.00#y:0.00#phi:0.00#
//		  sprintf(data,"!cmd_d:%s#x_d:%.2f#y_d:%.2f#phi_d:%.2f#\n",my_desired_point.cmd_d,my_desired_point.x_d,my_desired_point.y_d,my_desired_point.phi_d);
//		  HAL_UART_Transmit_DMA(&huart2, (uint8_t*)data,strlen(data));
		  HAL_Delay(100);
		  uart_len = 0;
		  uart_flag =0;
	  }
		 // PID and control motor
	  	  if(en_PID ==1){
		  	  // cal phi desired (rad/s)
			  phid_pid = atan2((my_desired_point.y_d - myRobot.y),(my_desired_point.x_d - myRobot.x));
			  hpid.setpoint = phid_pid;
			  hpid.current = myRobot.theta;
		  	  //calculate output PID
		  	  myRobot.omega = calculate_pid_output(&hpid);//rad/s

		  	  //calspeed : write a function in pid_module to call angular vel of 2 motor anf convert it to PWM signal
		  	  myRobot.v_r = cal_speed_right_motor(myRobot.omega, myRobot.v);// m/s
		  	  myRobot.v_l = cal_speed_left_motor(myRobot.omega, myRobot.v);//m/s

		  	  //limint vr vl
		  	  if(abs(myRobot.v_r) > v_max){
		  		  if(myRobot.v_r >= 0 )
		  			  myRobot.v_r  = (float) v_max;
		  		  else
		  			  myRobot.v_r  = (float)-1* v_max;
		  	  }
		  	  else if(abs(myRobot.v_r) <v_min ){
		  		  if(myRobot.v_r >= 0 )
		  			  myRobot.v_r  = (float)v_min;
		  		  else
		  			  myRobot.v_r = (float)-v_min ;
		  	  }
		  	  if(abs(myRobot.v_l) > v_max){
		  		  if(myRobot.v_l > 0 )
		  			  myRobot.v_l  = (float)v_max;
		  		  else
		  			  myRobot.v_l  = (float)-1* v_max;
		  	  }
		  	  else if(abs(myRobot.v_l) < v_min ){
		  		  if(myRobot.v_l > 0 )
		  			  myRobot.v_l  =(float) v_min;
		  		  else
		  			  myRobot.v_l = (float)-v_min ;
		  	  }
		  	  // limit max min PWM for motor
		  	  myRobot.v_l_PWM = mps_to_PWM(myRobot.v_l);
		  	  myRobot.v_r_PWM = mps_to_PWM(myRobot.v_r );
	  		  en_PID =0;
	  	  }


		  // PWM cho 2 motor
		  if(en_run == 1){
			  //left motor
			  // clockwise
			  if(myRobot.v_l_PWM > 0){
				  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,(int)myRobot.v_l_PWM);//
				  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
			  }
			  //counter Clokwise
			  else{
				  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//
				  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,(int)-myRobot.v_l_PWM);
			  }
			  //right motor
			  // clockwise
			  if(myRobot.v_r_PWM > 0){
				  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,(int)myRobot.v_r_PWM);//
				  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
			  }
			  //counter Clokwise
			  else{
				  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);//
				  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,(int)-1*myRobot.v_r_PWM);
			  }
			  //this make my PID come true , i dont know why we use the code above but it didnt work

			  //left motor
//			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,abs(myRobot.v_l_PWM));//
//			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
//			  //right motor
//			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, abs(myRobot.v_r_PWM));//
//			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);

		  }
		  else{
			  strcpy(myRobot.cmd,"STP");
//			  myRobot.theta = 0.0;
//			  init_Robot(&myRobot);
			  //left motor
			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//
			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
			  //right motor
			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);//
			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
			  //enable driver
//			  HAL_GPIO_WritePin(PE10_EN_DRIVER_GPIO_Port,PE10_EN_DRIVER_Pin,RESET);
		  }
		  // end condition


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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 159;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PE10_EN_DRIVER_GPIO_Port, PE10_EN_DRIVER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE10_EN_DRIVER_Pin */
  GPIO_InitStruct.Pin = PE10_EN_DRIVER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PE10_EN_DRIVER_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
