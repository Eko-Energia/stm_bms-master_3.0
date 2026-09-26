#ifndef APP_TIMING_H
#define APP_TIMING_H
#include <stdbool.h>
#include <stdint.h>

/* True once per periodMs. Advances *last by whole periods so cadence does not
   drift, and resynchronises if more than one period was missed. */
bool Timing_Due(uint32_t now, uint32_t *last, uint32_t periodMs);
#endif
