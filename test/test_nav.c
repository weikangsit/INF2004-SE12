/** @file    test_nav.c
 *  @brief   Unit tests UT-19 and UT-20 (R-5): the bypass width check
 *           (bypass_peek_result) and the return leg (bypass_return) in
 *           nav_fsm.c.
 *
 *  The functions are static, so the file under test is included here.
 *  Motion Control, the ultrasonic scan and the other modules are fakes that
 *  record the commands the FSM gives them.
 */
#include <string.h>

#include "unity.h"
#include "stub_kernel.h"
#include "stub_modules.h"

#include "nav_fsm.c"

void
setUp (void)
{
    stub_kernel_reset();
    stub_modules_reset();
    nav_fsm_init();                           /* clears g_nav, state INIT */
    g_nav.b_bypass_left  = true;
    g_nav.bypass         = BYPASS_PEEK_WIDTH;
    g_nav.width_steps    = 0U;
    g_nav.b_width_margin = false;
}

void
tearDown (void)
{
}

/* The check for the next step: the car is stopped, facing forward. */
static void
peek (bool b_blocked)
{
    g_nav.bypass = BYPASS_PEEK_WIDTH;
    bypass_peek_result(b_blocked);
}

/* UT-19 (a): clear at once: one margin step, check again, then along. */
void
test_UT19a_clear_at_once_takes_margin_step_then_goes_along (void)
{
    peek(false);
    TEST_ASSERT_EQUAL_INT(BYPASS_TURN_OUT, g_nav.bypass);
    TEST_ASSERT_TRUE(g_nav.b_width_margin);

    /* the sideways step that follows the turn is the margin step: 30 cm */
    bypass_go(BYPASS_STEP_OUT);
    TEST_ASSERT_EQUAL_UINT32(BYPASS_WIDTH_MARGIN_CM, stub_forward_dist_cm);
    TEST_ASSERT_EQUAL_UINT32(30U, stub_forward_dist_cm);

    peek(false);                              /* clear again after the margin */
    TEST_ASSERT_EQUAL_INT(BYPASS_ALONG, g_nav.bypass);
    TEST_ASSERT_EQUAL_UINT32(BYPASS_ALONG_FIRST_CM, stub_forward_dist_cm);
    TEST_ASSERT_EQUAL_INT(NAV_INIT, g_nav.state);   /* no fault */
}

/* UT-19 (b): blocked twice, then clear: two more 20 cm steps, then (a). */
void
test_UT19b_blocked_twice_steps_20cm_each_time (void)
{
    peek(true);
    TEST_ASSERT_EQUAL_INT(BYPASS_TURN_OUT, g_nav.bypass);
    TEST_ASSERT_FALSE(g_nav.b_width_margin);
    bypass_go(BYPASS_STEP_OUT);
    TEST_ASSERT_EQUAL_UINT32(BYPASS_STEP_CM, stub_forward_dist_cm);

    peek(true);
    TEST_ASSERT_EQUAL_INT(BYPASS_TURN_OUT, g_nav.bypass);
    bypass_go(BYPASS_STEP_OUT);
    TEST_ASSERT_EQUAL_UINT32(20U, stub_forward_dist_cm);

    peek(false);                              /* first clear: margin step */
    TEST_ASSERT_TRUE(g_nav.b_width_margin);
    bypass_go(BYPASS_STEP_OUT);
    TEST_ASSERT_EQUAL_UINT32(30U, stub_forward_dist_cm);

    peek(false);
    TEST_ASSERT_EQUAL_INT(BYPASS_ALONG, g_nav.bypass);
    TEST_ASSERT_EQUAL_UINT32(4U, g_nav.width_steps);   /* 4 checks so far */
}

/* UT-19 (c): blocked every time: 9 more steps, then on the 10th check
 * (BYPASS_MAX_STEPS) FC_NO_ROUTE detail 3 and FAULT. */
void
test_UT19c_blocked_every_time_gives_up_at_10_steps (void)
{
    uint32_t n = 0U;

    TEST_ASSERT_EQUAL_UINT32_MESSAGE(10U, BYPASS_MAX_STEPS,
                                     "the plan says 10 steps (200 cm)");

    for (n = 1U; n < 10U; n++)
    {
        peek(true);
        TEST_ASSERT_EQUAL_INT_MESSAGE(BYPASS_TURN_OUT, g_nav.bypass, "steps again");
        TEST_ASSERT_EQUAL_INT(0, stub_events_of_kind(EV_FAULT));
    }

    peek(true);                               /* the 10th */
    TEST_ASSERT_EQUAL_INT(NAV_FAULT, g_nav.state);
    TEST_ASSERT_EQUAL_INT(BYPASS_NONE, g_nav.bypass);
    TEST_ASSERT_EQUAL_INT(1, stub_events_of_kind(EV_FAULT));
    TEST_ASSERT_EQUAL_UINT8(FC_NO_ROUTE, stub_events[stub_event_count - 1].code);
    TEST_ASSERT_EQUAL_INT16(3, stub_events[stub_event_count - 1].value1);
    TEST_ASSERT_TRUE(stub_motion_stop_calls >= 1);
}

/* UT-20: the sideways steps are measured by the encoders (20 cm + 30 cm
 * margin = 50 cm); the return leg is that plus BYPASS_RETURN_EXTRA_CM. */
void
test_UT20_return_leg_is_offset_plus_15cm (void)
{
    g_nav.bypass       = BYPASS_STEP_OUT;     /* first step: 20 cm */
    g_nav.leg_start_mm = 0;
    stub_vstate.dist_mm = 200;
    bypass_step_done();
    TEST_ASSERT_EQUAL_INT32(200, g_nav.offset_mm);

    g_nav.bypass       = BYPASS_STEP_OUT;     /* margin step: 30 cm */
    g_nav.leg_start_mm = 200;
    stub_vstate.dist_mm = 500;
    bypass_step_done();
    TEST_ASSERT_EQUAL_INT32(500, g_nav.offset_mm);

    bypass_return();
    TEST_ASSERT_EQUAL_UINT32(65U, stub_forward_dist_cm);          /* 50 + 15 */
    TEST_ASSERT_EQUAL_UINT32(SPEED_SEARCH_CMS, stub_forward_speed_cms);
    TEST_ASSERT_EQUAL_INT(NAV_LINE_REACQUIRE, g_nav.state);
    TEST_ASSERT_EQUAL_INT(BYPASS_NONE, g_nav.bypass);
    TEST_ASSERT_NOT_EQUAL_UINT8(NO_SEQ, g_nav.pending_seq);
}

/* UT-20: when the return drive ends and no sensor saw the line:
 * FC_REACQUIRE. */
void
test_UT20_no_line_at_end_of_return_is_fc_reacquire (void)
{
    nav_in_t in;

    g_nav.offset_mm = 500;
    bypass_return();

    (void) memset(&in, 0, sizeof(in));
    in.now = g_nav.state_ms + 1000U;
    g_nav.pending_seq = NO_SEQ;               /* the return drive finished */

    reacquire_step(&in);

    TEST_ASSERT_EQUAL_INT(NAV_FAULT, g_nav.state);
    TEST_ASSERT_EQUAL_UINT8(FC_REACQUIRE, stub_events[stub_event_count - 1].code);
}

/* UT-20: when the line is seen: drive LINE_SENSOR_AHEAD_CM (8 cm) more so
 * the axle is on it, with no fault. */
void
test_UT20_line_seen_drives_8cm_more (void)
{
    nav_in_t in;

    g_nav.offset_mm = 500;
    bypass_return();
    stub_forward_calls = 0;

    (void) memset(&in, 0, sizeof(in));
    in.now         = g_nav.state_ms + 1000U;
    in.line_events = SENSOR_LINE_EVT_REGAINED;

    reacquire_step(&in);

    TEST_ASSERT_EQUAL_INT(1, stub_forward_calls);
    TEST_ASSERT_EQUAL_UINT32(8U, stub_forward_dist_cm);
    TEST_ASSERT_EQUAL_UINT32(LINE_SENSOR_AHEAD_CM, stub_forward_dist_cm);
    TEST_ASSERT_TRUE(g_nav.b_on_line_step);
    TEST_ASSERT_EQUAL_INT(NAV_LINE_REACQUIRE, g_nav.state);
}

int
main (void)
{
    UNITY_BEGIN();
    RUN_TEST(test_UT19a_clear_at_once_takes_margin_step_then_goes_along);
    RUN_TEST(test_UT19b_blocked_twice_steps_20cm_each_time);
    RUN_TEST(test_UT19c_blocked_every_time_gives_up_at_10_steps);
    RUN_TEST(test_UT20_return_leg_is_offset_plus_15cm);
    RUN_TEST(test_UT20_no_line_at_end_of_return_is_fc_reacquire);
    RUN_TEST(test_UT20_line_seen_drives_8cm_more);

    return (UNITY_END());
}
