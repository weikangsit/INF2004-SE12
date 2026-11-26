/** @file    test_safety.c
 *  @brief   Unit tests UT-26 (R-7: trip and clear) and UT-27 (R-7: task
 *           heartbeats) for safety.c.
 *
 *  events_handle() and heartbeats_check() are static, so the file under
 *  test is included here. The motors, Motion Control, telemetry and the
 *  radio are fakes that record what Safety asked of them.
 */
#include <string.h>

#include "unity.h"
#include "stub_kernel.h"
#include "stub_modules.h"

#include "safety.c"

#define NOW_MS     (10000U)
#define BOOT_MS    (0U)          /* well past SAFETY_BOOT_GRACE_MS = 2000 */

void
setUp (void)
{
    stub_kernel_reset();
    stub_modules_reset();
    safety_init();
    fault_init();
    gb_latched = false;
}

void
tearDown (void)
{
}

typedef struct
{
    uint32_t     event;
    fault_code_t code;
    uint32_t     detail;
} trip_case_t;

/* UT-26: every fault event inhibits the motors, latches, stops Motion
 * Control and reports one EV_FAULT with its code (and detail). */
void
test_UT26_each_event_trips_once (void)
{
    static trip_case_t const cases[] =
    {
        { SAFETY_EVT_ESTOP,     FC_ESTOP,     0U },
        { SAFETY_EVT_COLLISION, FC_COLLISION, 0U },
        { SAFETY_EVT_OVERTILT,  FC_OVERTILT,  0U },
        { SAFETY_EVT_LINE_LOST, FC_LINE_LOST, 0U },
        { SAFETY_EVT_LIFTED,    FC_LINE_LOST, SAFETY_DETAIL_LIFTED }
    };
    size_t k = 0U;

    for (k = 0U; k < (sizeof(cases) / sizeof(cases[0])); k++)
    {
        stub_modules_reset();
        gb_latched = false;

        events_handle(cases[k].event);

        TEST_ASSERT_TRUE_MESSAGE(stub_inhibit_state, "motors inhibited");
        TEST_ASSERT_TRUE_MESSAGE(safety_is_latched(), "latched");
        TEST_ASSERT_EQUAL_INT_MESSAGE(1, stub_motion_stop_calls, "Motion stopped");
        TEST_ASSERT_EQUAL_INT_MESSAGE(1, stub_events_of_kind(EV_FAULT), "one EV_FAULT");
        TEST_ASSERT_EQUAL_UINT8(cases[k].code, stub_events[0].code);
        TEST_ASSERT_EQUAL_INT16((int16_t) cases[k].detail, stub_events[0].value1);
    }
}

/* UT-26: codes are the documented bit numbers. */
void
test_UT26_fault_code_numbers (void)
{
    TEST_ASSERT_EQUAL_INT(8, FC_ESTOP);
    TEST_ASSERT_EQUAL_INT(5, FC_COLLISION);
    TEST_ASSERT_EQUAL_INT(6, FC_OVERTILT);
    TEST_ASSERT_EQUAL_INT(7, FC_LINE_LOST);
    TEST_ASSERT_EQUAL_INT(9, FC_HEARTBEAT);
}

/* UT-26: a second event while latched brakes again but sends no second
 * EV_FAULT and does not stop Motion Control twice. */
void
test_UT26_second_event_sends_no_second_fault (void)
{
    events_handle(SAFETY_EVT_ESTOP | SAFETY_EVT_COLLISION);

    TEST_ASSERT_EQUAL_INT(2, stub_inhibit_calls);           /* brake every time */
    TEST_ASSERT_EQUAL_INT(1, stub_motion_stop_calls);
    TEST_ASSERT_EQUAL_INT(1, stub_events_of_kind(EV_FAULT));
    TEST_ASSERT_EQUAL_UINT8(FC_ESTOP, stub_events[0].code); /* first one wins */
}

/* UT-26: safety_report() raises the flag that wakes the task. */
void
test_UT26_report_sets_the_wake_flag (void)
{
    UINT events = 0U;

    safety_report(SAFETY_EVT_ESTOP);
    TEST_ASSERT_EQUAL_INT(E_OK, tk_wai_flg(g_evt_flg, ALL_EVENTS,
                                           TWF_ORW | TWF_BITCLR, &events, TMO_POL));
    TEST_ASSERT_EQUAL_UINT(SAFETY_EVT_ESTOP, events);
    TEST_ASSERT_EQUAL_INT(E_TMOUT, tk_wai_flg(g_evt_flg, ALL_EVENTS,
                                              TWF_ORW | TWF_BITCLR, &events, TMO_POL));
}

/* UT-26: safety_clear() clears the latch, the inhibit and the fault mask. */
void
test_UT26_clear_resets_latch_inhibit_and_mask (void)
{
    events_handle(SAFETY_EVT_ESTOP);
    fault_set(FC_ESTOP);
    TEST_ASSERT_NOT_EQUAL_UINT16(0U, fault_get_mask());

    safety_clear();

    TEST_ASSERT_FALSE(safety_is_latched());
    TEST_ASSERT_FALSE(stub_inhibit_state);
    TEST_ASSERT_EQUAL_UINT16(0U, fault_get_mask());

    /* and a new fault trips again */
    events_handle(SAFETY_EVT_OVERTILT);
    TEST_ASSERT_TRUE(safety_is_latched());
    TEST_ASSERT_EQUAL_INT(2, stub_events_of_kind(EV_FAULT));
}

/* Task deadlines in task order (detail number): Line, Motion, IMU,
 * Ultrasonic, Nav, radio, Telemetry. */
static uint32_t const g_deadline_ms[7] = { 100U, 100U, 200U, 900U, 100U, 500U, 300U };

static void
check_one_task (uint32_t idx, uint32_t age_ms, uint32_t boot_ms)
{
    uint32_t last_seen[7];
    uint32_t k = 0U;

    for (k = 0U; k < 7U; k++)
    {
        last_seen[k] = NOW_MS;
    }
    last_seen[idx]   = NOW_MS - age_ms;
    stub_radio_alive = (5U != idx);           /* idx 5 is the radio itself */

    heartbeats_check(NOW_MS, boot_ms, last_seen);
}

/* UT-27: at its deadline a task is still fine; 1 ms later Safety trips
 * FC_HEARTBEAT with the task number as detail. Telemetry only warns. */
void
test_UT27_deadlines_and_detail_codes (void)
{
    uint32_t idx = 0U;

    for (idx = 0U; idx < 6U; idx++)
    {
        setUp();
        check_one_task(idx, g_deadline_ms[idx], BOOT_MS);
        TEST_ASSERT_FALSE_MESSAGE(safety_is_latched(), "at the deadline: no trip");

        setUp();
        check_one_task(idx, g_deadline_ms[idx] + 1U, BOOT_MS);
        TEST_ASSERT_TRUE_MESSAGE(safety_is_latched(), "past the deadline: trip");
        TEST_ASSERT_EQUAL_INT(1, stub_events_of_kind(EV_FAULT));
        TEST_ASSERT_EQUAL_UINT8(FC_HEARTBEAT, stub_events[0].code);
        TEST_ASSERT_EQUAL_INT16((int16_t) idx, stub_events[0].value1);
    }
}

/* UT-27: Telemetry late is a warning only (FC_TELEM_LATE), never a trip. */
void
test_UT27_telemetry_only_warns (void)
{
    check_one_task(6U, g_deadline_ms[6] + 1U, BOOT_MS);

    TEST_ASSERT_FALSE(safety_is_latched());
    TEST_ASSERT_EQUAL_INT(0, stub_events_of_kind(EV_FAULT));
    TEST_ASSERT_EQUAL_INT(0, stub_inhibit_calls);
    TEST_ASSERT_NOT_EQUAL_UINT16(0U, fault_get_mask() & (1U << FC_TELEM_LATE));
}

/* UT-27: nothing trips inside the 2 s boot grace period. */
void
test_UT27_boot_grace (void)
{
    uint32_t idx = 0U;

    for (idx = 0U; idx < 6U; idx++)
    {
        setUp();
        check_one_task(idx, 5000U, NOW_MS - 1999U);   /* 1.999 s after boot */
        TEST_ASSERT_FALSE(safety_is_latched());
    }

    setUp();
    check_one_task(0U, 5000U, NOW_MS - 2001U);        /* just past the grace */
    TEST_ASSERT_TRUE(safety_is_latched());
}

/* UT-27: a heartbeat resets the task's age. */
void
test_UT27_heartbeat_resets_age (void)
{
    uint32_t last_seen[7];
    uint32_t k = 0U;

    for (k = 0U; k < 7U; k++)
    {
        last_seen[k] = NOW_MS;
    }
    last_seen[4] = NOW_MS - 5000U;            /* Nav silent for 5 s ... */
    safety_heartbeat(SAFETY_HB_NAV);          /* ... but it just beat   */

    heartbeats_check(NOW_MS, BOOT_MS, last_seen);

    TEST_ASSERT_FALSE(safety_is_latched());
    TEST_ASSERT_EQUAL_UINT32(NOW_MS, last_seen[4]);
}

int
main (void)
{
    UNITY_BEGIN();
    RUN_TEST(test_UT26_each_event_trips_once);
    RUN_TEST(test_UT26_fault_code_numbers);
    RUN_TEST(test_UT26_second_event_sends_no_second_fault);
    RUN_TEST(test_UT26_report_sets_the_wake_flag);
    RUN_TEST(test_UT26_clear_resets_latch_inhibit_and_mask);
    RUN_TEST(test_UT27_deadlines_and_detail_codes);
    RUN_TEST(test_UT27_telemetry_only_warns);
    RUN_TEST(test_UT27_boot_grace);
    RUN_TEST(test_UT27_heartbeat_resets_age);

    return (UNITY_END());
}
