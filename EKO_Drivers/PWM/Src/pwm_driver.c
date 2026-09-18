/**
 * @file pwm_driver.c
 * @brief PWM signal driver for PERLA project
 *
 * This module provides initialisation and update routines
 * for measuring PWM duty cycle using STM32 timer input capture
 * in PWM Input mode and generating said signal.
 *
 * The driver supports PWM input configured either on:
 *  - TIM Channel 1 (period on CH1, duty on CH2)
 *  - TIM Channel 2 (period on CH2, duty on CH1)
 *
 * To achieve a sensible reading the timer tick clock MUST be much higher than
 * the measured PWM signal frequency (many counts per period). Whenever possible
 * use a large ARR so the captured period does not overflow. Calculated duty will
 * always be slightly lower than the actual due to finite rise/fall times.
 *
 * @author AGH EKO-ENERGIA
 * @author Andrzej Gondek
 */
#include "pwm_driver.h"
#include "main.h"
#include <math.h>
#include <stdbool.h>
/**
 * @brief Set PWM pulse duty as a percentage of the timer period.
 *
 * This function sets the PWM duty cycle for a specified timer channel by
 * converting a duty value (0.0–100.0) into a timer compare value
 * based on the current auto-reload register (ARR). The calculated compare
 * value determines how long the PWM output stays high during one period.
 *
 * @param PWM Pointer to PWM_signal structure containing timer handle, channel, and duty cycle.
 * @param duty PWM duty value from [0.0, 100.0] to be set.
 */
void PWM_Out_setDuty(struct PWM_Out_signal *PWM,float duty)
{
	uint32_t pulseValue = (uint32_t)roundf((float)__HAL_TIM_GET_AUTORELOAD(PWM->htim) * (duty / 100.0f));
	PWM->duty = duty;
	__HAL_TIM_SET_COMPARE(PWM->htim,PWM->Channel,pulseValue);
}
/**
 * @brief Monitors PWM input signal and updates duty cycle in case of signal timeout.
 *
 * This function checks whether a PWM input signal has timed out based on the
 * expected signal frequency and a configurable monitoring period count.
 * If no signal edge is detected within the calculated timeout period,
 * the function reads the current GPIO pin state and sets the duty cycle
 * to either 0% or 100%, assuming a constant LOW or HIGH signal.
 *
 * @param signal Pointer to PWM_IC_signal structure containing signal parameters
 *               such as frequency, last capture clock, and duty cycle.
 * @param GPIOx  Pointer to GPIO port where the PWM signal pin is connected.
 * @param GPIO_Pin GPIO pin number used for PWM input monitoring.
 *
 * @note The timeout is calculated as:
 *       TIMEOUT_MS = (1000.0f / frequency) * PWM_monitorPeriodCount
 *       Minimum timeout is clamped to 1 ms.
 *
 * @warning If frequency is set incorrectly or to 0, timeout calculation
 *          may produce invalid results.
 *
 * @retval None
 */
void PWM_IC_Monitor(struct PWM_IC_signal* signal, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    /* 1) If IC callback posted new captures, compute duty = (high_time / period) * 100 */
    if (signal->dataReady) {
        if (signal->icVal != 0) {
            signal->duty = ((float)signal->icPulseVal * 100.0f) / (float)signal->icVal;
        }
        signal->dataReady = false;
        signal->clock = HAL_GetTick();
    }

    /*
     * 2) Signal-loss timeout:
     *    TIMEOUT_MS = (1000 / frequency) * PWM_monitorPeriodCount  (min 1 ms)
     *    frequency==0 → keep minimum timeout to avoid divide-by-zero
     */
    uint32_t now = HAL_GetTick();
    uint32_t TIMEOUT_MS = 1;

    if (signal->frequency > 0) {
        TIMEOUT_MS = (uint32_t)((1000.0f / (float)signal->frequency) * (float)PWM_monitorPeriodCount);
        if (TIMEOUT_MS < 1) {
            TIMEOUT_MS = 1;
        }
    }

    /* No edges for TIMEOUT_MS → treat pin as stuck HIGH (100%) or LOW (0%) */
    if ((now - signal->clock) > TIMEOUT_MS) {
        if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET) {
            signal->duty = 100.0f;
        } else {
            signal->duty = 0.0f;
        }
        signal->clock = now;
    }
}

/**
 * @brief Initialize PWM input signal structure and timer channel
 *
 * This function initializes the PWM_signal structure fields and
 * configures the selected timer input capture channel for PWM
 * input measurement.
 *
 * The timer must already be configured in PWM Input mode
 *
 * @param signal Pointer to PWM_signal structure to initialize
 * @param frequency Expected PWM signal frequency
 * @param isChannel1
 *        - true  : PWM input configured on TIM Channel 1
 *        - false : PWM input configured on TIM Channel 2
 * @param htim Pointer to the timer handle used for input capture
 */
void PWM_IC_Init(struct PWM_IC_signal* signal,
		            TIM_HandleTypeDef *htim,
                    int frequency,
                    bool isChannel1)
{
    signal->duty = 0.0f;
    signal->frequency = frequency;
    signal->icVal = 0;
	signal->icPulseVal = 0;   // Initialize new field
	signal->dataReady = false; // Initialize new field
    signal->ch1 = isChannel1;
    signal->clock = HAL_GetTick();
    /* Configure input capture parameters */
    signal->sConfigIC.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
    signal->sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    signal->sConfigIC.ICFilter    = 0;
    if (isChannel1)
    {
        HAL_TIM_IC_ConfigChannel(htim, &signal->sConfigIC, TIM_CHANNEL_1);
		HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_1);
        HAL_TIM_IC_Start(htim, TIM_CHANNEL_2);
    }
    else
    {
        HAL_TIM_IC_ConfigChannel(htim, &signal->sConfigIC, TIM_CHANNEL_2);
		HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_2);
        HAL_TIM_IC_Start(htim, TIM_CHANNEL_1);
    }
}
/**
 * @brief  Starts PWM generation on a specified timer channel and initializes PWM parameters.
 * @param  htim Pointer to the TIM HAL handle used for PWM generation.
 * @param  Channel Specifies the TIM channel to start PWM on.
 *                 This parameter can be one of the following values:
 *                 @arg TIM_CHANNEL_1
 *                 @arg TIM_CHANNEL_2
 *                 @arg TIM_CHANNEL_3
 *                 @arg TIM_CHANNEL_4
 * @param  duty Duty cycle in the range [0.0, 100.0].
 * @param  frequency PWM signal frequency.
 * @param  PWM Pointer to PWM_signal structure to initialize.
 * @note   While generally calling this function on an already running PWM signal
 *         is safe, it may cause a short glitch in the signal.
 */
HAL_StatusTypeDef PWM_Out_Init(struct PWM_Out_signal *PWM, TIM_HandleTypeDef *htim, uint32_t Channel, float duty,int frequency)
{
	if (PWM == NULL || htim == NULL) {
		return HAL_ERROR;
	}

	/*
	 * Order matters: publish handle/channel BEFORE starting the timer so that
	 * PWM_Out_setDuty (called below) uses the correct fields, and so the CCR
	 * preload register holds the requested duty before UEV latches it.
	 */
	PWM->htim      = htim;
	PWM->Channel   = Channel;
	PWM->frequency = frequency;
	PWM->duty      = duty;

	/* Write CCR preload with target duty */
	PWM_Out_setDuty(PWM, duty);

	/*
	 * Start CC output + counter. Idempotent: if MX_TIM3_Init already started
	 * the channel (defensive early start), HAL_TIM_PWM_Start returns HAL_ERROR
	 * because channel_state == BUSY. That is NOT a real failure here — treat
	 * "already running" as success.
	 */
	HAL_TIM_ChannelStateTypeDef ch_state = TIM_CHANNEL_STATE_GET(htim, Channel);
	if (ch_state == HAL_TIM_CHANNEL_STATE_READY) {
		if (HAL_TIM_PWM_Start(htim, Channel) != HAL_OK) {
			return HAL_ERROR;
		}
	} else if (ch_state != HAL_TIM_CHANNEL_STATE_BUSY) {
		return HAL_ERROR;
	}
	/* Ensure timer is enabled regardless of who started the channel */
	__HAL_TIM_ENABLE(htim);

	/*
	 * HAL_TIM_PWM_ConfigChannel enables CCRx preload (OCxPE=1), so the value
	 * we just wrote to CCRx sits in the preload and only loads to the shadow
	 * on the next Update Event. Trigger a UEV manually so the first PWM cycle
	 * already reflects the requested duty instead of the reset value (0 = LOW).
	 */
	__HAL_TIM_SET_COUNTER(htim, 0);
	htim->Instance->EGR = TIM_EGR_UG;

	return HAL_OK;
}
/**
 * @brief Update PWM duty cycle measurement
 *
 * This function reads captured timer values and computes
 * the PWM duty cycle in percent.
 *
 * Behavior depends on the PWM input configuration:
 * - If PWM input is on Channel 1:
 *     - CH1 captures period
 *     - CH2 captures high time
 * - If PWM input is on Channel 2:
 *     - CH2 captures period
 *     - CH1 captures high time
 * Function is to be used inside the HAL_TIM_IC_CaptureCallback
 * after verifying whether the correct timer caused the interrupt.
 * The result is stored in PWM_signal::duty as a percentage
 * value in the range 0–100.
 *
 * @param htim Pointer to the timer handle used for input capture
 * @param PWM Pointer to initialized PWM_signal structure
 */
void PWM_IC_update(struct PWM_IC_signal *PWM, TIM_HandleTypeDef *htim)
{
    if (PWM->ch1)
    {
        // Read Period
        PWM->icVal = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        // Read High Time
        PWM->icPulseVal = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    }
    else
    {
        // Read Period
        PWM->icVal = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        // Read High Time
        PWM->icPulseVal = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    }

    // Raise flag for the monitor function
    PWM->dataReady = true;
}
