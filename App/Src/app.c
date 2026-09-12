#include "app.h"
#include "app_adc.h"
#include "app_can.h"
#include "app_contactor.h"
#include "app_jk.h"
#include "app_therm.h"
#include "app_thermal.h"
#include "app_timing.h"
#include "bms_calib.h"
#include "bms_errors.h"
#include "CAN_DB.h"
#include "error_handler.h"
#include "led_driver.h"
#include "main.h"

extern CAN_HandleTypeDef  hcan1, hcan2;
extern ADC_HandleTypeDef  hadc1;
extern TIM_HandleTypeDef  htim3;
extern UART_HandleTypeDef huart1;

#define THERM_PERIOD_MS      (1000u)

static volatile uint16_t adcBuf[3];
static EH_HandleTypeDef  eh;
static struct LED ledGreen = { LED_BLINK, NULL, 0u };
static struct LED ledRed   = { LED_OFF,   NULL, 0u };
static uint32_t thermTick;

/* Overrides the __weak HAL implementation so the LED driver's shared blink
   phase advances. Without this neither LED ever blinks. */
void HAL_IncTick(void)
{
    uwTick += uwTickFreq;
    LED_IncSyncTick();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1) { ADC_OnConvComplete(); }
}

/* Shared by both buses: dispatch on the instance or CAN1 traffic would land in
   the thermistor path. */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1)      { CAN_App_OnRx1(hcan); }
    else if (hcan == &hcan2) { CAN_App_OnRx2(hcan); }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) { JK_OnTxComplete(); }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart == &huart1) { JK_OnRxEvent(size); }
}

/* Three Error_Handler() sites run before MX_GPIO_Init, where a write to the
   gated GPIOB is silently discarded on the F1. RCC is always clocked, so open
   the gate here. Idempotent; spec 10. */
static void forceRedLedUsable(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef cfg;
    cfg.Pin   = RED_LD_Pin;
    cfg.Mode  = GPIO_MODE_OUTPUT_PP;
    cfg.Pull  = GPIO_NOPULL;
    cfg.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RED_LD_GPIO_Port, &cfg);
}

/* A helper so App_OnFatalError can bind these too: 16 of the 18
   Error_Handler() sites run before initAll(). */
static void bindLeds(void)
{
    ledGreen.GPIO_Port = GREEN_LD_GPIO_Port; ledGreen.GPIO_Pin = GREEN_LD_Pin;
    ledRed.GPIO_Port   = RED_LD_GPIO_Port;   ledRed.GPIO_Pin   = RED_LD_Pin;
}

static void initAll(void)
{
    bindLeds();

    /*
     * Priorities from spec 3.4, asserted here because CubeMX has silently
     * changed generated behaviour on this project before.
     */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 2, 0);
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 2, 0);
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
    HAL_NVIC_SetPriority(USART1_IRQn, 4, 0);
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 4, 0);
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 4, 0);

    /* CAN first: EH_init needs the scheduler, and faults are reportable from
       the next line onward. */
    /* Before EH_init, so a failure here cannot be reported as a fault:
       escalate instead, or the board runs with a dead bus and says nothing. */
    if (!CAN_App_Init(&hcan1, &hcan2, &eh)) { Error_Handler(); }
    EH_init(&eh, &hcan1, BMSMASTER_NODE_FRAME_ID, CAN_App_Scheduler());

    /* A hand edit at bring-up that breaks the NTC table's strict ordering
       would divide by zero in the temperature interpolation; code 10. */
    if (!CALIB_NtcCountIsMonotonic()) { Error_Handler(); }

    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) { Error_Handler(); }
    ADC_Init(adcBuf, &eh);
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuf, 3u) != HAL_OK) { Error_Handler(); }

    CONTACTOR_Init(&htim3);
    JK_Init(&huart1, &eh);
    THERM_Init(&eh);
    THERMAL_Init(&eh);
    LED_ChangeState(&ledGreen, LED_BLINK);
}

static void step(uint32_t now)
{
    ADC_Task(now);
    if (Timing_Due(now, &thermTick, THERM_PERIOD_MS)) { THERM_Task(); }
    JK_Task(now);
    CONTACTOR_Task(now);
    THERMAL_Evaluate(ADC_Ready(), ADC_TempCenti(),
                     THERM_MaxRaw(), THERM_MaxModule(), THERM_MaxTherm());

    const LED_STATE_e want = (eh.activeErrorCount > 0u) ? LED_ON : LED_OFF;
    if (ledRed.state != want) { LED_ChangeState(&ledRed, want); }
    LED_Handle(&ledGreen);
    LED_Handle(&ledRed);

    CAN_App_Task();
}

void app_main(void)
{
    initAll();
    for (;;) {
        step(HAL_GetTick());
    }
}

void App_OnFatalError(void)
{
    /*
     * Reachable from Error_Handler() before any init has run, so nothing here
     * may assume a handle exists: every hardware access below is guarded.
     */

    /* No-op before CONTACTOR_Init, and correct: PWM_Out_Init is the only
       caller of HAL_TIM_PWM_Start on TIM3, so until it runs the output is
       never enabled and CCR3 stays 0 - the contactor cannot be energised. */
    CONTACTOR_ForceOpen();

    if (ledRed.GPIO_Port == NULL) { bindLeds(); }
    if (ledRed.GPIO_Port != NULL) { forceRedLedUsable(); LED_ChangeState(&ledRed, LED_ON); }

    if (!EH_isInitialized(&eh)) {
        return;                 /* CAN never came up: LED and trap is all we have */
    }
    EH_stop(&eh, BMS_ERR_FATAL_INIT, ERROR_SEVERITY_ERROR);
    /* Assumes interrupts are still serviced: true for every current
       Error_Handler() call site (init and superloop context only). */
    for (;;) {
        CAN_App_Task();         /* keep the node frame alive so the vehicle learns why */
    }
}
