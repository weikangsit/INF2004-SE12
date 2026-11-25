/** @file    test_line_steer.c
 *  @brief   Unit tests UT-1 to UT-4 (R-1): line_steer() in motion_control.c.
 *
 *  line_steer() is static, so the file under test is included here.
 *  Convention of the function: +1 turns right, -1 turns left; the result is
 *  the wheel-speed differential in m/s.
 */
#include <string.h>

#include "unity.h"
#include "stub_kernel.h"
#include "stub_modules.h"

#include "motion_control.c"

void
setUp (void)
{
    stub_kernel_reset();
    stub_modules_reset();
    (void) memset(&g_ctx, 0, sizeof(g_ctx));
}

void
tearDown (void)
{
}

static motion_in_t
reading (uint8_t bits, uint32_t now_ms)
{
    motion_in_t in;

    (void) memset(&in, 0, sizeof(in));
    in.now_ms       = now_ms;
    in.line_bits    = bits;
    in.b_line_fresh = true;

    return (in);
}

/* UT-1: left sensor black only: turn left, toward the black sensor. */
void
test_UT1_left_black_steers_left (void)
{
    motion_in_t const in = reading(VEHICLE_IR_LEFT, 1000U);

    TEST_ASSERT_EQUAL_FLOAT(-0.09f, line_steer(&in));
    TEST_ASSERT_EQUAL_INT32(-1, g_ctx.line_last);
    TEST_ASSERT_TRUE(g_ctx.b_steered);            /* steering-speed hold */
    TEST_ASSERT_EQUAL_UINT32(1000U, g_ctx.steer_ms);
}

/* UT-2: right sensor black only: turn right. */
void
test_UT2_right_black_steers_right (void)
{
    motion_in_t const in = reading(VEHICLE_IR_RIGHT, 2000U);

    TEST_ASSERT_EQUAL_FLOAT(0.09f, line_steer(&in));
    TEST_ASSERT_EQUAL_INT32(1, g_ctx.line_last);
    TEST_ASSERT_TRUE(g_ctx.b_steered);
}

/* UT-3: both white keeps turning gently toward the side that saw black. */
void
test_UT3_both_white_turns_gently_to_last_side (void)
{
    motion_in_t const left  = reading(VEHICLE_IR_LEFT, 1000U);
    motion_in_t const white = reading(0U, 1005U);

    (void) line_steer(&left);                     /* line_last = -1 */
    TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, -0.0225f, line_steer(&white));
    TEST_ASSERT_EQUAL_INT32(-1, g_ctx.line_last); /* unchanged */

    /* Never saw black: drives straight. */
    g_ctx.line_last = 0;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, line_steer(&white));
}

/* UT-4: both black is a junction: no steering, memory cleared. */
void
test_UT4_both_black_drives_straight (void)
{
    motion_in_t const junction = reading(VEHICLE_IR_LEFT | VEHICLE_IR_RIGHT,
                                         3000U);

    g_ctx.line_last = -1;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, line_steer(&junction));
    TEST_ASSERT_EQUAL_INT32(0, g_ctx.line_last);
}

int
main (void)
{
    UNITY_BEGIN();
    RUN_TEST(test_UT1_left_black_steers_left);
    RUN_TEST(test_UT2_right_black_steers_right);
    RUN_TEST(test_UT3_both_white_turns_gently_to_last_side);
    RUN_TEST(test_UT4_both_black_drives_straight);

    return (UNITY_END());
}
