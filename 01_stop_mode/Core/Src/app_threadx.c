/* USER CODE BEGIN Header */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "fm_debug.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define THREAD_STACK_SIZE 1024

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern LPTIM_HandleTypeDef hlptim1;
extern RTC_HandleTypeDef hrtc;
uint8_t thread_stack[THREAD_STACK_SIZE];
TX_THREAD thread_ptr;
uint16_t g_lptim1_start;
uint16_t g_lptim1_end;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
VOID refresh_thread_entry(ULONG initial_input);


/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  /* USER CODE BEGIN App_ThreadX_Init */
  (void)byte_pool;
  tx_thread_create(&thread_ptr, "refresh_thread", refresh_thread_entry, 0x1234, thread_stack, THREAD_STACK_SIZE, 15, 15, 1, TX_AUTO_START);

  /* USER CODE END App_ThreadX_Init */

  return ret;
}

/**
  * @brief  MX_ThreadX_Init
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN  Before_Kernel_Start */

  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/**
  * @brief  App_ThreadX_LowPower_Timer_Setup
  * @param  count : TX timer count
  * @retval None
  */
void App_ThreadX_LowPower_Timer_Setup(ULONG count)
{
  /* USER CODE BEGIN  App_ThreadX_LowPower_Timer_Setup */
	uint32_t ticks_to_sleep;

#ifdef FM_DEBUG_UART_TX_TIME_ON_IDLE
	fm_debug_uint32_uart(count);
#endif
	g_lptim1_start = LPTIM1->CNT;
	ticks_to_sleep = (32768 / 16) * count; // clock_freq /  RTC_WAKEUPCLOCK_RTCCLK_DIV16
	ticks_to_sleep /= TX_TIMER_TICKS_PER_SECOND;
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, ticks_to_sleep, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
  /* USER CODE END  App_ThreadX_LowPower_Timer_Setup */
}

/**
  * @brief  App_ThreadX_LowPower_Enter
  * @param  None
  * @retval None
  */
void App_ThreadX_LowPower_Enter(void)
{
  /* USER CODE BEGIN  App_ThreadX_LowPower_Enter */
	// CPU goes to stop mode,

	HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);

  /* USER CODE END  App_ThreadX_LowPower_Enter */
}

/**
  * @brief  App_ThreadX_LowPower_Exit
  * @param  None
  * @retval None
  */
void App_ThreadX_LowPower_Exit(void)
{
  /* USER CODE BEGIN  App_ThreadX_LowPower_Exit */

	/*
	 * If CPU wakes up other reason but timer flag we must wait until
	 * expected time was elapsed. Debugger issues makes wake up CPU earlier
	 */
	HAL_ResumeTick();
	SystemClock_Config();
  /* USER CODE END  App_ThreadX_LowPower_Exit */
}

/**
  * @brief  App_ThreadX_LowPower_Timer_Adjust
  * @param  None
  * @retval Amount of time (in ticks)
  */
ULONG App_ThreadX_LowPower_Timer_Adjust(void)
{
  /* USER CODE BEGIN  App_ThreadX_LowPower_Timer_Adjust */
	static uint16_t cnt_drift = 0;
	ULONG cnt_ret;
	g_lptim1_end = LPTIM1->CNT;
	cnt_ret = (g_lptim1_end - g_lptim1_start);
	cnt_ret *= TX_TIMER_TICKS_PER_SECOND;
	cnt_ret += cnt_drift;
	cnt_drift = cnt_ret % 2048; // 2048 = lptim_clok_frq /  clock_prescaler
	cnt_ret /= 2048;
	return cnt_ret;
  /* USER CODE END  App_ThreadX_LowPower_Timer_Adjust */
}

/* USER CODE BEGIN 1 */
VOID refresh_thread_entry(ULONG initial_input)
{
	while(1)
	{
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
		tx_thread_sleep(20);

		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
		tx_thread_sleep(200);
	}
}

/* USER CODE END 1 */
