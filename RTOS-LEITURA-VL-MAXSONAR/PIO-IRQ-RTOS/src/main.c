#include "conf_board.h"
#include <asf.h>

#include "PIO_OLED.h"
#include "PIO_FUNCTIONS.h"
#include "TC-RTT-RTC.h"

/************************************************************************/
/* BOARD CONFIG                                                         */
/************************************************************************/

//PINO ECHO
#define ECHO_PI				PIOD
#define	ECHO_PI_ID			ID_PIOD
#define ECHO_PI_IDX			30
#define ECHO_PI_IDX_MASK	(1 << ECHO_PI_IDX)

//PINO TRIGGER
#define TRIG_PI				PIOA
#define	TRIG_PI_ID			ID_PIOA
#define TRIG_PI_IDX			6
#define TRIG_PI_IDX_MASK	(1 << TRIG_PI_IDX)


/** RTOS  */
#define TASK_PROCESSAMENTO_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_PROCESSAMENTO_STACK_PRIORITY            (tskIDLE_PRIORITY)

SemaphoreHandle_t xSemaphoreEchoHandler;

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/************************************************************************/
/* recursos RTOS                                                        */
/************************************************************************/


//Variaveis
volatile double tempo = 0;
double freq = (float) 1/(2*0.000058);
volatile char aguardo = 0;
volatile double echo_flag = 0;

/************************************************************************/
/* prototypes local                                                     */
/************************************************************************/

void grafico_distancias(int dist);
void echo_callback(void);
void TC6_Handler(void);
void init(void);
static void BUT_init(void);
static void BUT1_OLED_init(void);
void pin_toggle(Pio *pio, uint32_t mask);
static void USART1_init(void);
void LED_init(int estado);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

/**
* \brief Called if stack overflow during execution
*/
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	* identify which task has overflowed its stack.
	*/
	for (;;) {
	}
}

/**
* \brief This function is called by FreeRTOS idle task
*/
extern void vApplicationIdleHook(void) { pmc_sleep(SAM_PM_SMODE_SLEEP_WFI); }

/**
* \brief This function is called by FreeRTOS each tick
*/
extern void vApplicationTickHook(void) {}

extern void vApplicationMallocFailedHook(void) {
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT((volatile void *)NULL);
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

	
void TC1_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC0, 1);

	/** Muda o estado do LED (pisca) **/
	flag_but1 = 1; 
}

void echo_callback(void){
	//if (!pio_get(ECHO_PI, PIO_INPUT, ECHO_PI_IDX_MASK)) {
	
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreEchoHandler, &xHigherPriorityTaskWoken);
}

void TC6_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC2, 0);

	/** Muda o estado do LED (pisca) **/
	
	if (aguardo) {
		
		tempo = 0;
		//Se ultrapassa o tempo, reinicia o tempo, a flag de aguardo e do echo
		aguardo = 0;
		echo_flag = 0;
		
		tc_stop(TC2, 0);
		//printa erro
	}
	
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_processamento(void *pvParameters) {
	
	

	for (;;)  {
		if (flag_but1) {
				
				//ao apertar o botão, dá um sinal pro trigger 
				pio_set(TRIG_PI,TRIG_PI_IDX_MASK);
				delay_us(10);
				pio_clear(TRIG_PI,TRIG_PI_IDX_MASK);
				
				pin_toggle(LED_PI1, LED_PI1_IDX_MASK);
				vTaskDelay(300);
				pin_toggle(LED_PI1, LED_PI1_IDX_MASK);
				flag_but1  = 0;
				
			}
		
		//semaforo ativado pelo echo
		if ((xSemaphoreTake(xSemaphoreEchoHandler, 10 / portTICK_PERIOD_MS))){
			if (!echo_flag) {
				
				RTT_init(freq, 0, 0);
				echo_flag = 1;
				aguardo = 1;
				
				// 42.5 é 1/2*TMAX
				int TC_t = 42.5 + 0.2*(tempo);
				TC_init(TC2, ID_TC6, 0, TC_t);
				tc_start(TC2, 0);
				
				} else if (echo_flag && aguardo) {
				
				//reinicia a flag do echo
				echo_flag = 0;
				
				tempo = rtt_read_timer_value(RTT);
				aguardo = 0;
				tc_stop(TC2, 0);
				
				if (tempo != 0) {
					
					char str[300];
					double  tempo_real= (float) tempo/freq;
					double distancia_cm = (340*tempo_real*100.0)/2.0;
					printf("A distância foi de %d\n", (int) distancia_cm);
					
				}
			}
		} 
		
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

/**
* \brief Configure the console UART.
*/
static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}


void init(void) {
	//Initialize the board clock
	sysclk_init();
	oled_init();
	
	// Desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	//Definindo o ECHO PIO como input
	pmc_enable_periph_clk(ECHO_PI_ID);
	pio_set_input(ECHO_PI,ECHO_PI_IDX_MASK,PIO_DEFAULT);
	
	//Definindo o ECHO PIO como output
	pmc_enable_periph_clk(TRIG_PI_ID);
	pio_configure(TRIG_PI, PIO_OUTPUT_0,TRIG_PI_IDX_MASK, PIO_DEFAULT);
	
	pio_handler_set(ECHO_PI,
	ECHO_PI_ID,
	ECHO_PI_IDX_MASK,
	PIO_IT_EDGE,
	echo_callback);
	
	pio_enable_interrupt(ECHO_PI, ECHO_PI_IDX_MASK);
	pio_get_interrupt_status(ECHO_PI);
	
	NVIC_EnableIRQ(ECHO_PI_ID);
	NVIC_SetPriority(ECHO_PI_ID, 4); // Prioridade 4

}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

/**
*  \brief FreeRTOS Real Time Kernel example entry point.
*
*  \return Unused (ANSI-C compatibility).
*/
int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	init();
	configure_console();

	/* Attempt to create a semaphore. */
	xSemaphoreEchoHandler = xSemaphoreCreateBinary();
	if (xSemaphoreEchoHandler == NULL)
	printf("falha em criar o semaforo echo handler led\n");

	/* Create task to make led blink */
	if (xTaskCreate(task_processamento, "Led", TASK_PROCESSAMENTO_STACK_SIZE, NULL,
	TASK_PROCESSAMENTO_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* RTOS não deve chegar aqui !! */
	while (1) {
	}

	/* Will only get here if there was insufficient memory to create the idle
	* task. */
	return 0;
}
