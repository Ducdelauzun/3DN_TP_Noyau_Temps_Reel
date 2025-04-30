/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "drv_uart1.h"
#include "shell.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STACK_SIZE 256
#define TASK1_PRIORITY 1
#define TASK2_PRIORITY 2
#define TASK1_DELAY 1
#define TASK2_DELAY 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
SemaphoreHandle_t xSemaphore;
uint32_t delay_ms = 100;
extern TaskHandle_t xTaskTakeHandle;
TaskHandle_t xTaskTakeHandle = NULL;
QueueHandle_t xQueue;

BaseType_t ret;
SemaphoreHandle_t mutexPrintf;
static h_shell_t mon_shell;

TaskHandle_t ledTaskHandle = NULL;
volatile uint32_t led_period_ms = 0;

volatile uint32_t spam_count = 0;
char spam_msg[BUFFER_SIZE] = "spam";
TaskHandle_t spamTaskHandle = NULL;

SemaphoreHandle_t mySemaphore;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
int __io_putchar(int chr)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&chr, 1, HAL_MAX_DELAY);
	return chr;
}

void LedTask(void *pvParameters)
{
	while (1)
	{
		if (led_period_ms == 0)
		{
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			vTaskSuspend(NULL);
		}
		else
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			vTaskDelay(pdMS_TO_TICKS(led_period_ms / 2));
		}
	}
}

/*
void taskGive(void *pvParameters)
{
	for (;;)
	{
		printf("taskGive: avant xSemaphoreGive()\r\n");
		xSemaphoreGive(xSemaphore);
		printf("taskGive: après xSemaphoreGive()\r\n");

		vTaskDelay(pdMS_TO_TICKS(delay_ms));

		// Incrémente le délai de 100ms à chaque tour
		delay_ms += 100;
	}
}

void taskTake(void *pvParameters)
{
	for (;;)
	{
		printf("taskTake: en attente de sémaphore...\r\n");
		if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE)
		{
			printf("taskTake: sémaphore acquis !\r\n");
		}
		else
		{
			printf("taskTake: Timeout! Sémaphore non acquis.\r\n");
			// Reset software STM32
			NVIC_SystemReset();
		}
	}
}*/
/*
void taskGive(void *pvParameters)
{
	for (;;)
	{
		printf("taskGive: avant xTaskNotifyGive()\r\n");
		xTaskNotifyGive(xTaskTakeHandle);
		printf("taskGive: après xTaskNotifyGive()\r\n");

		vTaskDelay(pdMS_TO_TICKS(delay_ms));
		delay_ms += 100;
	}
}

void taskTake(void *pvParameters)
{
	xTaskTakeHandle = xTaskGetCurrentTaskHandle();

	for (;;)
	{
		printf("taskTake: en attente de notification...\r\n");

		uint32_t notif = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

		if (notif > 0)
		{
			printf("taskTake: notification reçue !\r\n");
		}
		else
		{
			printf("taskTake: Timeout! Aucune notification reçue.\r\n");
			NVIC_SystemReset();
		}
	}
}
 */
/*
void taskGive(void *pvParameters)
{
	TickType_t timerValue;

	for (;;)
	{
		timerValue = HAL_GetTick();
		printf("taskGive: envoi de %lu dans la queue\r\n", timerValue);

		if (xQueueSend(xQueue, &timerValue, portMAX_DELAY) != pdPASS)
		{
			printf("taskGive: ERREUR d'envoi dans la queue !\r\n");
		}

		vTaskDelay(pdMS_TO_TICKS(delay_ms));
		delay_ms += 100;
	}
}

void taskTake(void *pvParameters)
{
	TickType_t receivedValue;

	for (;;)
	{
		printf("taskTake: attente d'une valeur dans la queue...\r\n");

		if (xQueueReceive(xQueue, &receivedValue, pdMS_TO_TICKS(1000)) == pdTRUE)
		{
			printf("taskTake: reçu %lu depuis la queue !\r\n", receivedValue);
		}
		else
		{
			printf("taskTake: Timeout! Aucune donnée reçue.\r\n");
			NVIC_SystemReset();
		}
	}
}*/
/*
void task_bug(void * pvParameters)
{
	int delay = (int) pvParameters;
	for(;;)
	{
		xSemaphoreTake(mutexPrintf, portMAX_DELAY);
		printf("Je suis %s et je m'endors pour \%d ticks\r\n", pcTaskGetName(NULL), delay);
		xSemaphoreGive(mutexPrintf);
		vTaskDelay(delay);
	}
}*/

int ma_fonction_test(h_shell_t * h_shell, int argc, char ** argv)
{
	int size;
	size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "Fonction test appelee avec %d argument(s)\r\n", argc);
	h_shell->drv.transmit(h_shell->print_buffer, size);

	for(int i = 0; i < argc; i++) {
		size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "argv[%d] = %s\r\n", i, argv[i]);
		h_shell->drv.transmit(h_shell->print_buffer, size);
	}

	return 0;
}

void shellTask(void * pvParameters)
{
	shell_run(&mon_shell);
}

int led_shell_func(h_shell_t * h_shell, int argc, char ** argv)
{
	if (argc < 2)
	{
		int len = snprintf(h_shell->print_buffer, BUFFER_SIZE, "Usage: l <periode_ms>\r\n");
		h_shell->drv.transmit(h_shell->print_buffer, len);
		return -1;
	}

	led_period_ms = atoi(argv[1]);

	if (led_period_ms > 0)
	{
		if (eTaskGetState(ledTaskHandle) == eSuspended)
		{
			vTaskResume(ledTaskHandle);
		}
	}

	int len = snprintf(h_shell->print_buffer, BUFFER_SIZE,
			"Clignotement LED %s (periode = %lu ms)\r\n",
			led_period_ms == 0 ? "désactivé" : "activé", led_period_ms);
	h_shell->drv.transmit(h_shell->print_buffer, len);

	return 0;
}

void spamTask(void *pvParameters)
{
	while (1)
	{
		if (spam_count > 0)
		{
			printf("%s\r\n", spam_msg);
			spam_count--;
		}
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}

int spam_shell_func(h_shell_t * h_shell, int argc, char ** argv)
{
	if (argc < 3)
	{
		int len = snprintf(h_shell->print_buffer, BUFFER_SIZE,
				"Usage: s <message> <nombre>\r\n");
		h_shell->drv.transmit(h_shell->print_buffer, len);
		return -1;
	}

	strncpy(spam_msg, argv[1], BUFFER_SIZE - 1);
	spam_msg[BUFFER_SIZE - 1] = '\0';
	spam_count = atoi(argv[2]);

	int len = snprintf(h_shell->print_buffer, BUFFER_SIZE,
			"Spam lancé : %s x %lu\r\n", spam_msg, spam_count);
	h_shell->drv.transmit(h_shell->print_buffer, len);

	return 0;
}
/*
void dummyTask(void *pvParameters)
{
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}*/

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    char msg[100];
    int size = snprintf(msg, sizeof(msg),
        "\r\n[ERROR] Stack overflow detecté dans la tâche : %s\r\n", pcTaskName);
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, size, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

    __disable_irq();

    while (1)
    {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        for (volatile uint32_t i = 0; i < 1000000; i++);
    }
}

void recursive_overflow(void)
{
    char buffer[10000];
    (void)buffer;
    recursive_overflow();
}

void dummyTask(void *pvParameters)
{
    volatile uint32_t dummy[1000];

    // Forcer l'utilisation réelle de la pile
    for (int i = 0; i < 1000; i++) {
        ((volatile uint32_t*)dummy)[i] = i;
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int overflow_shell_func(h_shell_t * h_shell, int argc, char ** argv)
{
    int len = snprintf(h_shell->print_buffer, BUFFER_SIZE,
                       "Déclenchement du dépassement de pile...\r\n");
    h_shell->drv.transmit(h_shell->print_buffer, len);

    if (xTaskCreate(dummyTask, "OverflowTask", 64, NULL, 2, NULL) != pdPASS) {
        int len = snprintf(h_shell->print_buffer, BUFFER_SIZE, "Erreur création OverflowTask\r\n");
        h_shell->drv.transmit(h_shell->print_buffer, len);
    }

    return 0; // Jamais atteint
}

void configureTimerForRunTimeStats(void)
{
    // Utiliser SysTick comme timer de stats
    // Pas très précis mais OK pour test
}

unsigned long getRunTimeCounterValue(void)
{
    return HAL_GetTick();  // Renvoie le nombre de ms écoulées
}

void producerTask(void *pvParameters)
{
    for (;;)
    {
        xSemaphoreGive(mySemaphore);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void consumerTask(void *pvParameters)
{
    for (;;)
    {
        if (xSemaphoreTake(mySemaphore, portMAX_DELAY)) {
            printf("Sémaphore pris !\r\n");
        }
    }
}
/* USER CODE END PFP */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	/*if (xTaskCreate(LedTask, "LED Task", 128, NULL, 1, &ledTaskHandle) != pdPASS) {
		printf("Erreur création tâche LED\r\n");
		Error_Handler();
	}*/

	/*
	// Création du sémaphore binaire
	xSemaphore = xSemaphoreCreateBinary();
	if (xSemaphore == NULL) {
		Error_Handler();
	}

	if (xTaskCreate(taskGive, "Give", 128, NULL, 1, NULL) != pdPASS) {
    	printf("Erreur création tâche Give\r\n");
    	Error_Handler();
	}
	if (xTaskCreate(taskTake, "Take", 128, NULL, 2, NULL) != pdPASS) {
    	printf("Erreur création tâche Take\r\n");
    	Error_Handler();
	}
	 */

	/*
	if (xTaskCreate(taskGive, "Give", 128, NULL, 1, NULL) != pdPASS) {
    	printf("Erreur création tâche Give\r\n");
    	Error_Handler();
	}
	if (xTaskCreate(taskTake, "Take", 128, NULL, 2, &xTaskTakeHandle) != pdPASS) {
    	printf("Erreur création tâche Take\r\n");
    	Error_Handler();
	} */
	/*
	xQueue = xQueueCreate(5, sizeof(TickType_t));
	if (xQueue == NULL) {
		Error_Handler(); // Erreur de création
	}

	if (xTaskCreate(taskGive, "Give", 128, NULL, 1, NULL) != pdPASS) {
    	printf("Erreur création tâche Give\r\n");
    	Error_Handler();
	}
	if (xTaskCreate(taskTake, "Take", 128, NULL, 2, NULL) != pdPASS) {
    	printf("Erreur création tâche Take\r\n");
    	Error_Handler();
	}*/
	/*
	mutexPrintf = xSemaphoreCreateMutex();
	if (mutexPrintf == NULL) {
		Error_Handler();
	}

	ret = xTaskCreate(task_bug, "Tache 1", STACK_SIZE, \
			(void *) TASK1_DELAY, TASK1_PRIORITY, NULL);
	configASSERT(pdPASS == ret);
	ret = xTaskCreate(task_bug, "Tache 2", STACK_SIZE, \
			(void *) TASK2_DELAY, TASK2_PRIORITY, NULL);
	configASSERT(pdPASS == ret);*/

	mon_shell.drv.receive = drv_uart1_receive;
	mon_shell.drv.transmit = drv_uart1_transmit;

	shell_init(&mon_shell);
	if (shell_add(&mon_shell, 't', ma_fonction_test, "Fonction test utilisateur") != 0) {
	    printf("Erreur ajout commande shell t\r\n");
	    Error_Handler();
	}
	if (xTaskCreate(shellTask, "Shell", 512, NULL, 1, NULL) != pdPASS) {
	    	printf("Erreur création tâche Shell\r\n");
	    	Error_Handler();
		}
	if (xTaskCreate(LedTask, "LED Task", 128, NULL, 1, &ledTaskHandle) != pdPASS) {
	    	printf("Erreur création tâche LED Task\r\n");
	    	Error_Handler();
		}
	if (shell_add(&mon_shell, 'l', led_shell_func, "Contrôle de la LED") != 0) {
	    printf("Erreur ajout commande shell t\r\n");
	    Error_Handler();
	}

	if (xTaskCreate(spamTask, "Spam", 256, NULL, 1, &spamTaskHandle) != pdPASS) {
	    	printf("Erreur création tâche Spam\r\n");
	    	Error_Handler();
		}
	if (shell_add(&mon_shell, 's', spam_shell_func, "Affiche un message plusieurs fois") != 0) {
	    printf("Erreur ajout commande shell t\r\n");
	    Error_Handler();
	}
	/*
	for (int i = 0; i < 100; i++) {
	    if (xTaskCreate(dummyTask, "Dummy", 128, NULL, 1, NULL) != pdPASS) {
	        printf("Erreur creation tache dummy %d\r\n", i);
	        Error_Handler();
	    }
	}*/

	if (shell_add(&mon_shell, 'o', overflow_shell_func, "Déclenche un overflow") != 0) {
	    printf("Erreur ajout commande shell o\r\n");
	    Error_Handler();
	}

	mySemaphore = xSemaphoreCreateBinary();
	xTaskCreate(producerTask, "Producer", 128, NULL, 1, NULL);
	xTaskCreate(consumerTask, "Consumer", 128, NULL, 1, NULL);

	vQueueAddToRegistry(mySemaphore, "MySemaphore");

	vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		//HAL_Delay(100);
		if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_SET) {
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);  // Allume la LED
		} else {
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);  // Éteint la LED
		}
		printf("Test printf bien\r\n");
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
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
