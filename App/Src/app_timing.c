#include "app_timing.h"

bool Timing_Due(uint32_t now, uint32_t *last, uint32_t periodMs)
{
    /* Unsigned difference stays correct across the HAL_GetTick() wrap. */
    if ((now - *last) < periodMs) {
        return false;
    }
    *last += periodMs;
    if ((now - *last) >= periodMs) {
        *last = now;                /* fell behind: resync, no catch-up burst */
    }
    return true;
}
