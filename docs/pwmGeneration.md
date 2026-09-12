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

`Core/Src/tim.c` has **no** user-code override. Every `USER CODE` section in the file is empty.
`BMS-Master.ioc` configures `PWM Generation CH3` directly on TIM3 (`SH.S_TIM3_CH3.0=TIM3_CH3,PWM
Generation3 CH3`), so CubeMX itself generates `HAL_TIM_PWM_Init()` and
`HAL_TIM_PWM_ConfigChannel()` with `sConfigOC.OCMode = TIM_OCMODE_PWM1` - what makes PB0 a real
PWM output is the `.ioc` setting, not an application-layer patch.

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

The EKO driver's `PWM_Out_setDuty()` uses `roundf()`, not `round()` (a `double` function), to
keep the conversion single-precision - the toolchain target has no FPU, so any `double` arithmetic
pulls in soft-float library calls.

## Contactor relay use case (`App/Src/app_contactor.c`)

The application-layer contactor state machine does **not** call `PWM_Out_setDuty()` on its
state-change path. It writes the timer compare register directly with
`__HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_3, ccr)`, using pre-computed compare-value constants
rather than a duty percentage:

| State | `CCR` | Duty |
| --- | ---: | ---: |
| `CCR_PULL_IN` | 999 | 100 % |
| `CCR_HOLD` | 500 | 50 % |
| `CCR_OPEN` | 0 | 0 % |

Setting compare values directly, rather than converting a percentage through
`PWM_Out_setDuty()`'s `roundf()` call on every transition, keeps floating-point arithmetic off
the safety-relevant open/close path. `PWM_Out_Init()` is still called once at startup - it writes
the CCR preload and forces the update event so the first active cycle is correct - but the
running state machine only ever writes `CCR` directly afterward.

Sequence: 100 % duty for 2 s on every transition to closed (coil pull-in), then 50 % to hold;
0 % whenever open. See [firmwareSpec.md](firmwareSpec.md) section 8 for the full contactor and
safe-state state machine.

## Correctness checks

- Keep PB0 configured for the timer alternate-function output.
- Keep channel 3 configured as PWM1 after CubeMX regeneration.
- Match the configured timer frequency to the duty-state constants.
- Validate duty input at the application boundary.
- Ensure `HAL_TIM_MspPostInit()` runs so the GPIO is configured.
- Verify approximately 1 kHz and the expected duty transition with an oscilloscope or logic analyzer.

## PWM input support

The same EKO driver supports PWM input capture. `PWM_IC_update()` reads period and high-time captures, while `PWM_IC_Monitor()` calculates duty or reports a stuck-high/stuck-low state after a timeout. This is separate from the relay output on TIM3 channel 3 and requires a timer configured for PWM input capture.
