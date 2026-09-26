/*
 * App_OnFatalError is reached from Error_Handler() at 18 call sites, 16 of
 * which run before initAll(). This test links app.c and calls it with nothing
 * initialised - the case that used to dereference a NULL GPIO port.
 */
#include "test_runner.h"
#include "app.h"
#include "app_contactor.h"
#include "main.h"

/* app.c's externs. None of them is touched by App_OnFatalError. */
CAN_InstanceTypeDef  can1Inst, can2Inst;
CAN_HandleTypeDef    hcan1 = { &can1Inst, { DISABLE } };
CAN_HandleTypeDef    hcan2 = { &can2Inst, { DISABLE } };
ADC_HandleTypeDef    hadc1;
TIM_InstanceTypeDef  tim3Inst;
TIM_HandleTypeDef    htim3 = { &tim3Inst };
UART_InstanceTypeDef uart1Inst;
UART_HandleTypeDef   huart1 = { &uart1Inst };

TEST(a_fatal_error_before_any_init_does_not_fault)
{
    Fake_Reset();
    /* Nothing initialised: no LED port, no contactor timer, no error handler. */
    App_OnFatalError();
    /* Returned at all, which it could not do while it dereferenced NULL. */
    CHECK(1);
}

TEST(a_fatal_error_before_any_init_still_lights_the_red_led)
{
    Fake_Reset();
    CHECK_EQ(Fake_PinState(RED_LD_GPIO_Port, RED_LD_Pin), GPIO_PIN_RESET);
    App_OnFatalError();
    CHECK_EQ(Fake_PinState(RED_LD_GPIO_Port, RED_LD_Pin), GPIO_PIN_SET);
}

TEST(a_fatal_error_before_any_init_writes_no_compare_register)
{
    Fake_Reset();
    tim3Inst.CCR3 = 0xDEADu;            /* would be clobbered by an unguarded write */
    App_OnFatalError();
    CHECK_EQ(tim3Inst.CCR3, 0xDEADu);   /* the contactor timer is not ours yet */
}

/* On the SystemClock_Config fault path MX_GPIO_Init has not run, so GPIOB's
   clock gate is shut and a write to it is discarded rather than faulting - the
   LED would stay dark with no symptom. Assert the gate is opened and the pin
   configured, not merely that a write was attempted. */
TEST(a_fatal_error_before_any_init_opens_the_led_port_clock_gate)
{
    Fake_Reset();
    CHECK(!Fake_GpioClockEnabled(RED_LD_GPIO_Port));

    App_OnFatalError();

    CHECK(Fake_GpioClockEnabled(RED_LD_GPIO_Port));
    CHECK((Fake_GpioConfiguredPins(RED_LD_GPIO_Port) & RED_LD_Pin) != 0u);
}

TEST(a_fatal_error_after_init_opens_a_closed_contactor_and_lights_the_led)
{
    Fake_Reset();
    tim3Inst.ARR = 999u;
    CONTACTOR_Init(&htim3);

    /* A NODE frame with no Activ inside the window closes the contactor. */
    CONTACTOR_OnSafeStateFrame(3u);
    CONTACTOR_Task(0u);
    CHECK(CONTACTOR_IsClosed());
    CHECK(Fake_LastCompare() > 0u);

    App_OnFatalError();
    CHECK(!CONTACTOR_IsClosed());
    CHECK_EQ(Fake_LastCompare(), 0u);
    CHECK_EQ(tim3Inst.CCR3, 0u);
    CHECK_EQ(Fake_PinState(RED_LD_GPIO_Port, RED_LD_Pin), GPIO_PIN_SET);
}

int main(void)
{
    RUN(a_fatal_error_before_any_init_does_not_fault);
    RUN(a_fatal_error_before_any_init_still_lights_the_red_led);
    RUN(a_fatal_error_before_any_init_writes_no_compare_register);
    RUN(a_fatal_error_before_any_init_opens_the_led_port_clock_gate);
    RUN(a_fatal_error_after_init_opens_a_closed_contactor_and_lights_the_led);
    return TEST_SUMMARY();
}
