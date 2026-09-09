# PWM Generation

## Copied project configuration

The copied project generates relay PWM on:

- Timer: `TIM3`
- Channel: `TIM_CHANNEL_3`
- Pin: `PB0` (`RELAY_CTRL`)
- Prescaler: `71`
- Auto-reload period: `999`
- Counter mode: up-counting
- Output mode: PWM1

With a timer clock of approximately 72 MHz:

$$f_{PWM} = \frac{72\,MHz}{(71 + 1)(999 + 1)} = 1\,kHz$$

`Core/Src/tim.c` contains an important user-code override. CubeMX may initially configure channel 3 as output-compare timing, but the code changes it to `TIM_OCMODE_PWM1` with `HAL_TIM_PWM_ConfigChannel()`. This is what makes PB0 a real PWM output.

## Duty-cycle generation

The EKO PWM driver converts duty percentage into a compare value:

$$CCR = round\left(ARR \times \frac{duty}{100}\right)$$

For `ARR = 999`, 0% gives 0, 50% gives approximately 500, and 100% gives 999.

```c
PWM_Out_signal relay;

PWM_Out_Init(&relay, &htim3, TIM_CHANNEL_3, 100.0f, 1000);
PWM_Out_setDuty(&relay, 50.0f);
```

`PWM_Out_Init()` stores the timer handle and channel, writes the initial compare value, starts the channel when needed, enables the timer, resets the counter, and generates an update event so the first active cycle uses the requested duty.

## BMS relay use case

The original BMS wrapper in `BMS_PWM.c` uses:

1. 100% duty at 1 kHz during startup.
2. A 2 second startup period.
3. 50% duty at 1 kHz during operation.

```c
BMS_PWM_Init(&bms, &htim3);

for (;;) {
    BMS_PWM_NormalMode(&bms);
}
```

The clean copied project has the correct low-level timer configuration and EKO PWM driver, but it does not contain the original BMS wrapper or a call from `main()`. Add that application layer when relay startup and operational states are required.

## Correctness checks

- Keep PB0 configured for the timer alternate-function output.
- Keep channel 3 configured as PWM1 after CubeMX regeneration.
- Match the configured timer frequency to the duty-state constants.
- Validate duty input at the application boundary.
- Ensure `HAL_TIM_MspPostInit()` runs so the GPIO is configured.
- Verify approximately 1 kHz and the expected duty transition with an oscilloscope or logic analyzer.

## PWM input support

The same EKO driver supports PWM input capture. `PWM_IC_update()` reads period and high-time captures, while `PWM_IC_Monitor()` calculates duty or reports a stuck-high/stuck-low state after a timeout. This is separate from the relay output on TIM3 channel 3 and requires a timer configured for PWM input capture.
