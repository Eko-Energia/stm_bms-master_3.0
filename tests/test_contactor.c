#include "test_runner.h"
#include "app_contactor.h"

#define SAFESTATE_ACTIV_ID (1u)
#define SAFESTATE_NODE_ID  (3u)

static TIM_InstanceTypeDef tinst;
static TIM_HandleTypeDef htim = { &tinst };

static void setup(void)
{
    Fake_Reset();
    tinst.ARR = 999u;
    tinst.CCR3 = 0u;
    CONTACTOR_Init(&htim);
}

/* Must run before any other test calls CONTACTOR_Init: it is the only point
   in the process where "before CONTACTOR_Init has run" is genuinely true. */
TEST(power_on_and_pre_init_force_open_are_safe)
{
    Fake_Reset();
    CHECK(!CONTACTOR_IsClosed());
    CONTACTOR_ForceOpen();       /* must not crash before Init */
    CHECK(!CONTACTOR_IsClosed());
}

TEST(boot_state_is_open)
{
    setup();
    CHECK(!CONTACTOR_IsClosed());
    CHECK_EQ(Fake_LastCompare(), 0u);
}

TEST(nothing_closes_before_a_node_frame_is_seen)
{
    setup();
    CONTACTOR_Task(5000u);
    CHECK(!CONTACTOR_IsClosed());
    CHECK_EQ(Fake_LastCompare(), 0u);
}

TEST(a_node_frame_closes_with_the_pull_in_kick)
{
    setup();
    CONTACTOR_OnSafeStateFrame(SAFESTATE_NODE_ID);
    CONTACTOR_Task(0u);
    CHECK(CONTACTOR_IsClosed());
    CHECK_EQ(Fake_LastCompare(), 999u);           /* 100 % */

    CONTACTOR_Task(1999u);
    CHECK_EQ(Fake_LastCompare(), 999u);           /* still within the 2 s kick */

    CONTACTOR_Task(2000u);
    CHECK_EQ(Fake_LastCompare(), 500u);           /* 50 % hold */
}

TEST(safe_state_active_opens_immediately_even_mid_kick)
{
    setup();
    CONTACTOR_OnSafeStateFrame(SAFESTATE_NODE_ID);
    CONTACTOR_Task(0u);
    CHECK_EQ(Fake_LastCompare(), 999u);

    CONTACTOR_OnSafeStateFrame(SAFESTATE_ACTIV_ID);
    CONTACTOR_Task(500u);
    CHECK(!CONTACTOR_IsClosed());
    CHECK_EQ(Fake_LastCompare(), 0u);
}

TEST(it_recloses_three_hundred_milliseconds_after_the_last_activ)
{
    setup();
    CONTACTOR_OnSafeStateFrame(SAFESTATE_NODE_ID);
    CONTACTOR_Task(0u);
    CONTACTOR_OnSafeStateFrame(SAFESTATE_ACTIV_ID);
    CONTACTOR_Task(1000u);
    CHECK(!CONTACTOR_IsClosed());

    CONTACTOR_Task(1299u);
    CHECK(!CONTACTOR_IsClosed());
    CONTACTOR_Task(1300u);
    CHECK(CONTACTOR_IsClosed());
    CHECK_EQ(Fake_LastCompare(), 999u);           /* the kick repeats on every close */
}

TEST(repeated_activ_frames_hold_it_open)
{
    setup();
    CONTACTOR_OnSafeStateFrame(SAFESTATE_NODE_ID);
    CONTACTOR_Task(0u);
    for (uint32_t t = 100u; t <= 2000u; t += 200u) {
        CONTACTOR_OnSafeStateFrame(SAFESTATE_ACTIV_ID);
        CONTACTOR_Task(t);
        CHECK(!CONTACTOR_IsClosed());
    }
}

TEST(the_node_precondition_is_a_one_time_latch)
{
    setup();
    CONTACTOR_OnSafeStateFrame(SAFESTATE_NODE_ID);
    CONTACTOR_Task(0u);
    /* No further NODE frames for a minute; it must stay closed. There is no
       heartbeat supervision, by decision - spec 8. */
    CONTACTOR_Task(60000u);
    CHECK(CONTACTOR_IsClosed());
}

TEST(force_open_drops_the_duty_for_error_handler)
{
    setup();
    CONTACTOR_OnSafeStateFrame(SAFESTATE_NODE_ID);
    CONTACTOR_Task(0u);
    CONTACTOR_ForceOpen();
    CHECK(!CONTACTOR_IsClosed());
    CHECK_EQ(Fake_LastCompare(), 0u);
}

TEST(unrelated_frame_ids_are_ignored)
{
    setup();
    CONTACTOR_OnSafeStateFrame(2u);               /* SafeState_AccVal */
    CONTACTOR_OnSafeStateFrame(130u);
    CONTACTOR_Task(0u);
    CHECK(!CONTACTOR_IsClosed());
}

TEST(safe_state_timeout_boundary_is_pinned_both_sides)
{
    setup();
    CONTACTOR_OnSafeStateFrame(SAFESTATE_NODE_ID);
    CONTACTOR_Task(0u);
    CONTACTOR_OnSafeStateFrame(SAFESTATE_ACTIV_ID);
    CONTACTOR_Task(1000u);
    CHECK(!CONTACTOR_IsClosed());

    /* one tick before the 300 ms window elapses: still open */
    CONTACTOR_Task(1000u + 299u);
    CHECK(!CONTACTOR_IsClosed());

    /* exactly at the window: closes again */
    CONTACTOR_Task(1000u + 300u);
    CHECK(CONTACTOR_IsClosed());
}

TEST(tick_wrap_does_not_stall_the_safe_state_timeout)
{
    setup();
    CONTACTOR_OnSafeStateFrame(SAFESTATE_NODE_ID);
    CONTACTOR_Task(0xFFFFFFF0u);

    /* Activ frame arrives right before the tick wraps */
    CONTACTOR_OnSafeStateFrame(SAFESTATE_ACTIV_ID);
    CONTACTOR_Task(0xFFFFFFFEu);
    CHECK(!CONTACTOR_IsClosed());

    /* now < lastActivMs numerically, after the wrap; the window has not
       elapsed yet, so it must stay open */
    CONTACTOR_Task(0x00000002u);
    CHECK(!CONTACTOR_IsClosed());

    /* 300 ms after 0xFFFFFFFE wraps to 0x12A; the timeout must still fire */
    CONTACTOR_Task(0x0000012Au);
    CHECK(CONTACTOR_IsClosed());
}

int main(void)
{
    RUN(power_on_and_pre_init_force_open_are_safe);
    RUN(boot_state_is_open);
    RUN(nothing_closes_before_a_node_frame_is_seen);
    RUN(a_node_frame_closes_with_the_pull_in_kick);
    RUN(safe_state_active_opens_immediately_even_mid_kick);
    RUN(it_recloses_three_hundred_milliseconds_after_the_last_activ);
    RUN(repeated_activ_frames_hold_it_open);
    RUN(the_node_precondition_is_a_one_time_latch);
    RUN(force_open_drops_the_duty_for_error_handler);
    RUN(unrelated_frame_ids_are_ignored);
    RUN(safe_state_timeout_boundary_is_pinned_both_sides);
    RUN(tick_wrap_does_not_stall_the_safe_state_timeout);
    return TEST_SUMMARY();
}
