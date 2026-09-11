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
static bool     safeAsserted;         /* an Activ frame is inside its 300 ms window */
static volatile bool nodeSeen;        /* ISR-written, Task-read: visibility, not atomicity */
static volatile bool closed;          /* written by ForceOpen from ISR/Error_Handler context */
static uint32_t closedAtMs;
static bool     pullInDone;           /* the 2 s kick of this close has elapsed */

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
    safeAsserted = false;
    lastActivMs = 0u;
    nodeSeen = false;
    closed = false;
    closedAtMs = 0u;
    pullInDone = false;
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
        safeAsserted = true;
    }

    /* Elapsed windows are latched, not recomputed. Nothing refreshes lastActivMs
       once Activ frames stop, so a bare (now - lastActivMs) re-enters the window
       at the 2^32 ms tick wrap and opens the contactor for 300 ms. */
    if (safeAsserted && (uint32_t)(nowMs - lastActivMs) >= SAFE_CLEAR_MS) {
        safeAsserted = false;
    }

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
        pullInDone = false;
        setCompare(CCR_PULL_IN);        /* the kick repeats on every close */
        return;
    }

    /* Same latch: closedAtMs is frozen while closed, so the wrap would otherwise
       re-run the 2 s 100 % kick on an already-closed contactor. */
    if (!pullInDone && (uint32_t)(nowMs - closedAtMs) >= PULL_IN_MS) {
        pullInDone = true;
    }
    setCompare(pullInDone ? CCR_HOLD : CCR_PULL_IN);
}

void CONTACTOR_ForceOpen(void)
{
    closed = false;
    setCompare(CCR_OPEN);
}

bool CONTACTOR_IsClosed(void) { return closed; }
