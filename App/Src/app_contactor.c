#include "app_contactor.h"
#include "pwm_driver.h"

#define SAFESTATE_ACTIV_ID   (1u)    /* presence is the whole message: no signals */
#define SAFESTATE_NODE_ID    (3u)
#define SAFE_CLEAR_MS        (300u)  /* spec 8: the literal notionSpec reading */
#define PULL_IN_MS           (2000u)

/*
 * Compare values rather than a percentage: exact, and it keeps soft-float off a
 * path that runs on every state change. ARR is 999 at 1 kHz, so 999 is 100 %
 * and 500 is 50 %. PWM_Out_Init still does the setup, because it writes the
 * CCR preload and forces an update event so the very first cycle is correct.
 */
#define CCR_PULL_IN          (999u)
#define CCR_HOLD             (500u)
#define CCR_OPEN             (0u)

static TIM_HandleTypeDef *timer;
static struct PWM_Out_signal pwm;
static volatile bool     activPending;   /* set by the ISR, consumed by the Task */
static uint32_t lastActivMs;
static bool     activSeen;
static volatile bool nodeSeen;        /* ISR-written, Task-read: visibility, not atomicity */
static volatile bool closed;          /* written by ForceOpen from ISR/Error_Handler context */
static uint32_t closedAtMs;

static void setCompare(uint32_t ccr)
{
    if (timer != NULL) {
        __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_3, ccr);
    }
}

void CONTACTOR_Init(TIM_HandleTypeDef *htim)
{
    timer = htim;
    activPending = false;
    activSeen = false;
    lastActivMs = 0u;
    nodeSeen = false;
    closed = false;
    closedAtMs = 0u;
    (void)PWM_Out_Init(&pwm, htim, TIM_CHANNEL_3, 0.0f, 1000);
    setCompare(CCR_OPEN);
}

void CONTACTOR_OnSafeStateFrame(uint32_t stdId)
{
    /* Sets a flag only. Timestamping happens in CONTACTOR_Task, which is given
       the tick: no module reads the clock. Costs at most one loop iteration of
       precision against a 300 ms window. */
    if (stdId == SAFESTATE_ACTIV_ID) {
        activPending = true;
    } else if (stdId == SAFESTATE_NODE_ID) {
        nodeSeen = true;
    }
}

void CONTACTOR_Task(uint32_t nowMs)
{
    if (activPending) {
        activPending = false;
        lastActivMs = nowMs;
        activSeen = true;
    }

    /* Safe state is asserted while an Activ frame arrived within the window. */
    const bool safeAsserted = activSeen && ((uint32_t)(nowMs - lastActivMs) < SAFE_CLEAR_MS);

    if (safeAsserted || !nodeSeen) {
        if (closed) {
            closed = false;
            setCompare(CCR_OPEN);
        }
        return;
    }

    if (!closed) {
        closed = true;
        closedAtMs = nowMs;
        setCompare(CCR_PULL_IN);        /* the kick repeats on every close */
        return;
    }

    setCompare(((uint32_t)(nowMs - closedAtMs) < PULL_IN_MS) ? CCR_PULL_IN : CCR_HOLD);
}

void CONTACTOR_ForceOpen(void)
{
    closed = false;
    setCompare(CCR_OPEN);
}

bool CONTACTOR_IsClosed(void) { return closed; }
