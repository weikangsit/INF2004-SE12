/** @file    test_sensor_line.c
 *  @brief   Unit tests UT-5 and UT-6 (R-1): black_hold() and
 *           junction_seen() in sensor_line.c.
 *
 *  Both functions are static, so the file under test is included here.
 *  The line task runs every PERIOD_LINE_MS = 5 ms, so the tests step the
 *  functions in 5 ms steps.
 */
#include <string.h>

#include "unity.h"
#include "stub_kernel.h"
#include "stub_modules.h"

#include "sensor_line.c"

#define STEP_MS  (5U)

void
setUp (void)
{
    stub_kernel_reset();
    stub_modules_reset();
    (void) memset(g_hold, 0, sizeof(g_hold));
    g_jn_left_ms  = 0U;
    g_jn_right_ms = 0U;
    gb_jn_left    = false;
    gb_jn_right   = false;
    gb_jn_latched = false;
}

void
tearDown (void)
{
}

/* UT-5: a black reading is held for LINE_BLACK_HOLD_MS = 30 ms; the raw
 * junction check is not changed by the hold. */
void
test_UT5_black_hold_30ms_and_no_false_junction (void)
{
    uint32_t t           = 0U;
    uint32_t junctions   = 0U;

    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IR_LEFT, black_hold(VEHICLE_IR_LEFT, 0U));
    junctions += junction_seen(VEHICLE_IR_LEFT, 0U) ? 1U : 0U;

    for (t = STEP_MS; t <= 25U; t += STEP_MS)         /* raw white */
    {
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(VEHICLE_IR_LEFT, black_hold(0U, t),
                                        "still held at 25 ms or less");
        junctions += junction_seen(0U, t) ? 1U : 0U;
    }

    for (t = 30U; t <= 40U; t += STEP_MS)
    {
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(0U, black_hold(0U, t),
                                        "hold over from 30 ms");
        junctions += junction_seen(0U, t) ? 1U : 0U;
    }

    TEST_ASSERT_EQUAL_UINT32(0U, junctions);          /* held L is no junction */
}

/* The hold ends at once when the other sensor sees black (the line moved
 * across): the held reading must not look like a junction. */
void
test_UT5_hold_ends_when_other_sensor_sees_black (void)
{
    (void) black_hold(VEHICLE_IR_LEFT, 0U);
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_IR_RIGHT,
                            black_hold(VEHICLE_IR_RIGHT, 10U));
}

/* UT-6 A: left black at t = 0, right black at t = 200 ms (inside the
 * 250 ms window): one junction; no repeat while both stay black. */
void
test_UT6A_junction_inside_window (void)
{
    uint32_t t = 0U;

    TEST_ASSERT_FALSE(junction_seen(VEHICLE_IR_LEFT, 0U));
    for (t = STEP_MS; t < 200U; t += STEP_MS)
    {
        TEST_ASSERT_FALSE(junction_seen(0U, t));
    }

    TEST_ASSERT_TRUE(junction_seen(VEHICLE_IR_RIGHT, 200U));
    for (t = 205U; t <= 300U; t += STEP_MS)           /* both stay black */
    {
        TEST_ASSERT_FALSE_MESSAGE(
            junction_seen(VEHICLE_IR_LEFT | VEHICLE_IR_RIGHT, t),
            "no repeat while both stay black");
    }
}

/* UT-6 B: left black at t = 0, right black at t = 300 ms (outside the
 * window): no junction. */
void
test_UT6B_no_junction_outside_window (void)
{
    uint32_t t = 0U;

    TEST_ASSERT_FALSE(junction_seen(VEHICLE_IR_LEFT, 0U));
    for (t = STEP_MS; t < 300U; t += STEP_MS)
    {
        TEST_ASSERT_FALSE(junction_seen(0U, t));
    }

    TEST_ASSERT_FALSE(junction_seen(VEHICLE_IR_RIGHT, 300U));
}

/* UT-6: the next junction needs the sensors white for the window first. */
void
test_UT6_rearms_after_white_window (void)
{
    uint32_t t = 0U;

    TEST_ASSERT_FALSE(junction_seen(VEHICLE_IR_LEFT, 0U));
    TEST_ASSERT_TRUE(junction_seen(VEHICLE_IR_RIGHT, 10U));   /* first */

    /* Both black again after only 100 ms of white: still latched. */
    for (t = 15U; t < 110U; t += STEP_MS)
    {
        TEST_ASSERT_FALSE(junction_seen(0U, t));
    }
    TEST_ASSERT_FALSE(junction_seen(VEHICLE_IR_LEFT | VEHICLE_IR_RIGHT, 110U));

    /* White for more than JUNCTION_WINDOW_MS, then both black: second. */
    for (t = 115U; t <= 370U; t += STEP_MS)
    {
        TEST_ASSERT_FALSE(junction_seen(0U, t));
    }
    TEST_ASSERT_TRUE(junction_seen(VEHICLE_IR_LEFT | VEHICLE_IR_RIGHT, 375U));
}

int
main (void)
{
    UNITY_BEGIN();
    RUN_TEST(test_UT5_black_hold_30ms_and_no_false_junction);
    RUN_TEST(test_UT5_hold_ends_when_other_sensor_sees_black);
    RUN_TEST(test_UT6A_junction_inside_window);
    RUN_TEST(test_UT6B_no_junction_outside_window);
    RUN_TEST(test_UT6_rearms_after_white_window);

    return (UNITY_END());
}
