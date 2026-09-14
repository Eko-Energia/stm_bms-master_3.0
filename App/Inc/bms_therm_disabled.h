#ifndef BMS_THERM_DISABLED_H
#define BMS_THERM_DISABLED_H

/* Thermistors accepted as broken. Each reports 0 degC and is skipped by every
   check, so its cell has no thermal protection. Temporary: repair, then delete.
   X(module, therm) is BMSMaster_PCB<module>Therm<therm>Temp, 1-based. */
#define THERM_DISABLED_LIST(X)                                                 \
    /* PCBCells6_Therm9, id 261: shorted, pinned at the 100 degC ceiling.   */ \
    X(6u, 9u)                                                                  \
    /* PCBCells3_Therm6, id 236: reads offset from its pack, never saturates */\
    X(3u, 6u)

#endif /* BMS_THERM_DISABLED_H */
