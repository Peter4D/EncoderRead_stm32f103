/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_it.h"
#include "utilities.h"
#include "scheduler_2.h"
#include "sw_timer.h"

#include <string.h>
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

/* USER CODE BEGIN PV */

//static uint32_t encoder_cnt = 0;

static HAL_StatusTypeDef hUart1_status;

typedef struct _encoder_t
{
    uint32_t    dir;           // encoder direction information
    uint32_t    puls_encoder;  // encoder puls count in one revolution -> REDUNDANT ? 
    int64_t     rev_cnt;       // total revolutions of encoder 
    int64_t     puls_cnt;      // total puls count 
    int64_t     puls_old_cnt;  // total puls count used for speed calculation
    int32_t     speed;         // calculated speed pulses per time unit
    int32_t     speed_old;     // for use in calculation of acceleration 
    int32_t     accel;         // calculated acceleration 
}encoder_t;

encoder_t hEncoder = {0};

/* user tasks */
void TASK_ecoder_data_debugOut(void);
/**
 * @brief program main state machine 
 */
void TASK_stateMachine(void);
/**
 * @brief main state machine function pointer 
 */
void (*pAtive_state)(void);

void SM_init(void);

void SM_idle(void);
void SM_waitDeceleration(void);
void SM_break(void);

void SM_transition_2_idle(void);
void SM_transition_2_waitDeceleration(void);
void SM_transition_2_break(void);


uint32_t Filter_digital_state(uint32_t const *const input);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t Filter_digital_state(uint32_t const *const input) {
    #define FILTER_LEVEL   (4)

    static uint32_t filter = 0;
    uint32_t return_value = 0;

    filter << 1;
    filter |= (*input & 0x0001);
    
    if( (filter & (FILTER_LEVEL - 1)) == (FILTER_LEVEL - 1) ) {
        return_value = 1;
    }else if( filter & (FILTER_LEVEL - 1) == 0 ){
        return_value = 0;
    }
    return return_value;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    #define CALC_PRSC_val       (5)
    static uint32_t calc_prsc = CALC_PRSC_val;
    //static uint32_t dir_filtered = 0;
    
    hEncoder.dir = (htim1.Instance->CR1 >> TIM_CR1_DIR_Pos) & 0x0001;

    /* HW timer2 calculation timer  */
    if(htim->Instance == htim2.Instance){
        //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
        //dir_filtered = Filter_digital_state(&hEncoder.dir);

        hEncoder.puls_encoder = htim1.Instance->CNT; 
        /* read count value of encoder timer: */ 
        hEncoder.puls_cnt   = hEncoder.puls_encoder + hEncoder.rev_cnt * ECODER_PULS_PER_REV;

        /*calculation Prescaler */
        calc_prsc--;
        if(calc_prsc == 0) {
            calc_prsc = CALC_PRSC_val;
            /* calculate speed */
            hEncoder.speed = hEncoder.puls_cnt - hEncoder.puls_old_cnt;
            hEncoder.puls_old_cnt = hEncoder.puls_cnt;

            /* calculate acceleration */
            //hEncoder.accel      = hEncoder.speed - hEncoder.speed_old;
            if(hEncoder.dir == 0) {
            //if(dir_filtered == 0) {
                hEncoder.accel = hEncoder.speed - hEncoder.speed_old;
            }else {
                hEncoder.accel = hEncoder.speed_old - hEncoder.speed;
            }
            hEncoder.speed_old  = hEncoder.speed;
        }
    }

    /* HW timer1 (ENCODER) update (overflow) */ 
    if(htim->Instance == htim1.Instance){
        /* #debug */
        //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);

        if(hEncoder.dir == ROT_OF_INTEREST) {
            /* clockwise direction */
            hEncoder.rev_cnt++;
        }else if(hEncoder.dir == COUNTER_ROT) {
            /* counter clockwise direction */
            hEncoder.rev_cnt--;
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    /* this callback function could run ring buffer to handle multiple messages */ 
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
    
    USER_TIM1_Init();
    USER_TIM2_Init();

    SM_init();
    Scheduler.add_task(&TASK_ecoder_data_debugOut, 200);
    Scheduler.add_task(&TASK_stateMachine, 200);
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) //#b
    {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        Scheduler.task_exe();
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

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void systick_timer_swHandler(void) {
    _sw_timers_tick();
    Scheduler.run();
}

/* user TASKs */

/* state machine global variables: */
sw_timer_t xTmr_break = {0};

void TASK_stateMachine(void) {
    pAtive_state();
}

void SM_init(void){
    swTimer_init(&xTmr_break);
    SM_transition_2_idle();
}

static int64_t revolution_cnt = 0;
void SM_idle(void){
    
    if( ((hEncoder.rev_cnt - revolution_cnt) > START_SPIN_CNT_TH ) &&
    (hEncoder.dir == ROT_OF_INTEREST) ) 
    {
        SM_transition_2_waitDeceleration();
    }else if (hEncoder.dir == COUNTER_ROT){
        revolution_cnt = hEncoder.rev_cnt;
    }

}


static uint32_t filter_cnt = 0;
static uint32_t speed_filter = 0;
void SM_waitDeceleration(void){
    

    if(hEncoder.accel < 0) {
        filter_cnt++;
    }else{
        filter_cnt = 0;
    }

    if(hEncoder.speed == 0) {
        speed_filter++;
    }else{
        speed_filter = 0;
    }

    if( (filter_cnt > DECELERATION_FILTER_TH) || speed_filter > SPEED_FILTER_TH) 
    {
        /*activate IO */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        SM_transition_2_break();
    }
}


void SM_break(void){
    /* wait to stop */
    if(hEncoder.speed == 0) {
        /* wait another 2 seconds */
        if(swTimer.getTmrStatus(&xTmr_break) == SWTM_STOP){
            swTimer.set(&xTmr_break, 2000);
        }
        if(swTimer.isElapsed(&xTmr_break) == 1){
            /*de-activate IO */
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
            SM_transition_2_idle();
        }
    }
}

void SM_transition_2_idle(void){
    swTimer.clear(&xTmr_break);
    revolution_cnt = hEncoder.rev_cnt;
    pAtive_state = &SM_idle;
}

void SM_transition_2_waitDeceleration(void){
    pAtive_state = &SM_waitDeceleration;
}

void SM_transition_2_break(void){
    filter_cnt = 0;
    speed_filter = 0;
    pAtive_state = &SM_break;
}


void TASK_ecoder_data_debugOut(void) {
    //systickOld = systickCnt;

    //static int32_t systickOld = 0;
    //static uint8_t hello_msg[] = "hello word\n";

    #define ENCODER_OUT_MSNG_len    (100u)
    static uint8_t encoder_uart_msng_str[ENCODER_OUT_MSNG_len] = {0};
    static uint8_t dir_value_str[] = "0";
    static uint8_t encoder_val_str[10] = {0};
    static uint32_t encoder_uart_msng_str_len;

    /* #debug */
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    // convert direction into string
    if(hEncoder.dir == ROT_OF_INTEREST){
        dir_value_str[0] = '1';
    }else {
        dir_value_str[0] = '0';
    }
    /* convert encoder counts to string  */
    
    /* construct uart out message */
    strcpy(encoder_uart_msng_str, "============# \n");
    strcat(encoder_uart_msng_str, "dir     : ");
    strcat(encoder_uart_msng_str, dir_value_str);
    strcat(encoder_uart_msng_str, "\n");

    num2str(hEncoder.puls_cnt, encoder_val_str);
    strcat(encoder_uart_msng_str, "step_cnt: ");
    strcat(encoder_uart_msng_str, encoder_val_str);
    strcat(encoder_uart_msng_str, "\n");

    num2str(hEncoder.rev_cnt, encoder_val_str);
    strcat(encoder_uart_msng_str, "rev_cnt : ");
    strcat(encoder_uart_msng_str, encoder_val_str);
    strcat(encoder_uart_msng_str, "\n");

    num2str(hEncoder.speed, encoder_val_str);
    strcat(encoder_uart_msng_str, "speed   : ");
    strcat(encoder_uart_msng_str, encoder_val_str);
    strcat(encoder_uart_msng_str, "\n");

    num2str(hEncoder.accel, encoder_val_str);
    strcat(encoder_uart_msng_str, "accel   : ");
    strcat(encoder_uart_msng_str, encoder_val_str);
    strcat(encoder_uart_msng_str, "\n");

    /* uint16_t #test */
    // num2str(hEncoder.puls_encoder, encoder_val_str);
    // strcat(encoder_uart_msng_str, "puls_encod: ");
    // strcat(encoder_uart_msng_str, encoder_val_str);
    // strcat(encoder_uart_msng_str, "\n");

    encoder_uart_msng_str_len = strlen(encoder_uart_msng_str);
    assert_param(encoder_uart_msng_str_len < ENCODER_OUT_MSNG_len);
    
    hUart1_status = HAL_UART_Transmit_IT(&huart1, encoder_uart_msng_str, encoder_uart_msng_str_len );
    //hUart1_status = HAL_UART_Transmit_IT(&huart1, hello_msg, (sizeof(hello_msg)-1) );
    if (hUart1_status != HAL_OK)
    {
        /* there is no buffer so this situation is not useful when two concurrent call are made */
        assert_param(0);
    }
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
    while(1);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    (void)file;
    (void)line;
    while (1)
        ;

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
