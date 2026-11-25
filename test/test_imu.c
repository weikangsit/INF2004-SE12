/** @file    test_imu.c
 *  @brief   Unit tests UT-13, UT-14 (R-3, R-12: hump detection and the
 *           highest hump), UT-28 (R-7: impact check) and UT-29 (R-7:
 *           over-tilt) for imu_terrain.c.
 *
 *  The functions are static, so the file under test is included here.
 *
 *  A hump is driven the way the pitch detector sees it: the car moves
 *  STEP_MM per sample at a pitch whose sine is 0.1, so each sample adds
 *  exactly 1 mm of climb (sin(pitch) x distance step).
 */
#include <math.h>
#include <string.h>

#include "unity.h"
#include "stub_kernel.h"
#include "stub_modules.h"

#include "imu_terrain.c"

#define STEP_MM  (10)

static float32_t g_pitch_rad;
static float32_t g_pitch_deg;
static int32_t   g_dist_mm;

void
setUp (void)
{
    stub_kernel_reset();
    stub_modules_reset();
    imu_terrain_init();                       /* clears g_ctx, makes the queue */
    g_pitch_rad = asinf(0.1f);
    g_pitch_deg = g_pitch_rad * RAD2DEG;
    g_dist_mm   = 0;
    stub_vstate.nav_state = (uint8_t) NAV_READY;
}

void
tearDown (void)
{
}

/* One sample climbing (up) or descending 1 mm. */
static void
step (bool b_up)
{
    g_dist_mm += STEP_MM;
    hump_track(b_up ? g_pitch_rad : -g_pitch_rad,
               b_up ? g_pitch_deg : -g_pitch_deg, g_dist_mm);
}

/* One level sample: pitch 0, no climb. */
static void
level (void)
{
    hump_track(0.0f, 0.0f, g_dist_mm);
}

/* Climb climb_mm, then roll back down until the hump can end, then level. */
static void
drive_hump (int climb_mm)
{
    int n = 0;

    for (n = 0; n < climb_mm; n++)
    {
        step(true);
    }

    for (n = 0; (n < 100) && (g_ctx.b_in_hump || g_ctx.b_candidate) &&
                (g_ctx.hump_h >= (HUMP_END_FRACTION * g_ctx.hump_max)); n++)
    {
        step(false);
    }

    level();
}

/* Highest hump as the IMU task publishes it, in 0.5 mm: one level sample
 * through sample_process() and read what it handed to the shared record. */
static uint16_t
published_peak_half_mm (void)
{
    vec3_t const acc = { { 0.0f, 0.0f, 1.0f } };
    vec3_t const mag = { { 0.2f, 0.0f, 0.3f } };

    stub_vstate.dist_mm = g_dist_mm;
    sample_process(&acc, &mag);

    return (stub_imu_out.hump_peak_half_mm);
}

static int
count_events (imu_terrain_event_t * p_events, int max)
{
    int n = 0;

    while ((n < max) && imu_terrain_event_take(&p_events[n]))
    {
        n++;
    }

    return (n);
}

/* UT-13 (a): 6 mm of climb is only a candidate: no hump, no event. */
void
test_UT13a_6mm_is_not_a_hump (void)
{
    imu_terrain_event_t ev[4];
    int                 n = 0;

    for (n = 0; n < 6; n++)
    {
        step(true);
    }
    TEST_ASSERT_TRUE(g_ctx.b_candidate);
    TEST_ASSERT_FALSE(g_ctx.b_in_hump);

    level();                                  /* pitch under HUMP_OFF_DEG */
    TEST_ASSERT_FALSE(g_ctx.b_candidate);
    TEST_ASSERT_EQUAL_INT(0, count_events(ev, 4));
    TEST_ASSERT_EQUAL_INT(0, stub_events_of_kind(EV_HUMP));
    TEST_ASSERT_EQUAL_UINT16(0U, published_peak_half_mm());
}

/* UT-13 (b): 12 mm of climb (at least HUMP_MIN_MM = 10 mm) is a hump:
 * HUMP_START, then HUMP_END when level; the peak is 12 mm = 24 half mm. */
void
test_UT13b_12mm_is_a_hump (void)
{
    imu_terrain_event_t ev[4];
    int                 n = 0;

    for (n = 0; n < 12; n++)
    {
        step(true);
    }
    TEST_ASSERT_TRUE(g_ctx.b_in_hump);

    n = count_events(ev, 4);
    TEST_ASSERT_EQUAL_INT(1, n);
    TEST_ASSERT_EQUAL_INT(IMU_EVT_HUMP_START, ev[0]);

    for (n = 0; (n < 100) && (g_ctx.hump_h >= (HUMP_END_FRACTION * g_ctx.hump_max)); n++)
    {
        step(false);
    }
    TEST_ASSERT_TRUE(g_ctx.b_in_hump);        /* still in it until level */
    level();
    TEST_ASSERT_FALSE(g_ctx.b_in_hump);

    n = count_events(ev, 4);
    TEST_ASSERT_EQUAL_INT(1, n);
    TEST_ASSERT_EQUAL_INT(IMU_EVT_HUMP_END, ev[0]);

    TEST_ASSERT_EQUAL_INT(1, stub_events_of_kind(EV_HUMP));
    TEST_ASSERT_EQUAL_INT16(120, stub_events[stub_event_count - 1].value1);  /* x0.1 mm */
    TEST_ASSERT_EQUAL_UINT16(24U, published_peak_half_mm());
}

/* UT-13 (c): the same 12 mm climb, but just under the threshold at 9 mm
 * and then level: still no hump. */
void
test_UT13c_9mm_is_not_a_hump (void)
{
    imu_terrain_event_t ev[4];
    int                 n = 0;

    for (n = 0; n < 9; n++)
    {
        step(true);
    }
    level();
    TEST_ASSERT_EQUAL_INT(0, count_events(ev, 4));
    TEST_ASSERT_FALSE(g_ctx.b_in_hump);
}

/* UT-14: humps of 15, 25 and 12 mm in one run: the published peak is 30,
 * then 50, and stays 50. A reset fault (FAULT to another state) clears it. */
void
test_UT14_highest_hump_kept_until_reset (void)
{
    drive_hump(15);
    TEST_ASSERT_EQUAL_UINT16(30U, published_peak_half_mm());

    drive_hump(25);
    TEST_ASSERT_EQUAL_UINT16(50U, published_peak_half_mm());

    drive_hump(12);
    TEST_ASSERT_EQUAL_UINT16(50U, published_peak_half_mm());

    stub_vstate.nav_state = (uint8_t) NAV_FAULT;
    TEST_ASSERT_EQUAL_UINT16(50U, published_peak_half_mm());   /* fault alone keeps it */

    stub_vstate.nav_state = (uint8_t) NAV_READY;               /* reset fault */
    TEST_ASSERT_EQUAL_UINT16(0U, published_peak_half_mm());
}

/* UT-28 (a): a one-sample jump of 1.6 g that comes straight back is
 * rejected. */
void
test_UT28a_one_sample_spike_rejected (void)
{
    vec3_t const rest  = { { 0.0f, 0.0f, 1.0f } };
    vec3_t const spike = { { 1.6f, 0.0f, 1.0f } };

    impact_check(&rest);
    impact_check(&spike);
    TEST_ASSERT_TRUE(g_ctx.b_impact_pending);
    impact_check(&rest);
    TEST_ASSERT_FALSE(g_ctx.b_impact_pending);
    TEST_ASSERT_EQUAL_INT(0, stub_safety_report_count);
}

/* UT-28 (b): a jump of 1.6 g that stays 0.8 g away on the next sample is a
 * collision. */
void
test_UT28b_confirmed_jump_is_a_collision (void)
{
    vec3_t const rest  = { { 0.0f, 0.0f, 1.0f } };
    vec3_t const spike = { { 1.6f, 0.0f, 1.0f } };
    vec3_t const stays = { { 0.8f, 0.0f, 1.0f } };

    impact_check(&rest);
    impact_check(&spike);
    TEST_ASSERT_EQUAL_INT(0, stub_safety_report_count);   /* not yet */
    impact_check(&stays);
    TEST_ASSERT_EQUAL_INT(1, stub_safety_report_count);
    TEST_ASSERT_EQUAL_UINT32(SAFETY_EVT_COLLISION, stub_safety_reports[0]);
}

/* UT-28 (c): a sample at 1.97 g is saturated and is ignored. */
void
test_UT28c_saturated_sample_ignored (void)
{
    vec3_t const rest      = { { 0.0f, 0.0f, 1.0f } };
    vec3_t const saturated = { { 1.97f, 0.0f, 1.0f } };

    impact_check(&rest);
    impact_check(&saturated);
    TEST_ASSERT_FALSE(g_ctx.b_impact_pending);
    impact_check(&rest);
    TEST_ASSERT_EQUAL_INT(0, stub_safety_report_count);
}

/* UT-29: over-tilt must last OVERTILT_HOLD_MS = 200 ms (10 samples of
 * 20 ms). It is reported on every sample from then on; Safety latches once. */
void
test_UT29_overtilt_needs_200ms (void)
{
    int n = 0;

    /* 150 ms (7 samples) at 36 deg, then level: nothing. */
    for (n = 0; n < 7; n++)
    {
        tilt_check(36.0f, 0.0f);
    }
    tilt_check(0.0f, 0.0f);
    TEST_ASSERT_EQUAL_INT(0, stub_safety_report_count);
    TEST_ASSERT_EQUAL_UINT32(0U, g_ctx.tilt_ms);

    /* 250 ms (13 samples): first report on the 10th sample (200 ms). */
    for (n = 1; n <= 13; n++)
    {
        tilt_check(36.0f, 0.0f);

        if (n < 10)
        {
            TEST_ASSERT_EQUAL_INT_MESSAGE(0, stub_safety_report_count,
                                          "no report before 200 ms");
        }
        else if (10 == n)
        {
            TEST_ASSERT_EQUAL_INT(1, stub_safety_report_count);
            TEST_ASSERT_EQUAL_UINT32(SAFETY_EVT_OVERTILT, stub_safety_reports[0]);
        }
        else
        {
            /* keeps reporting while the tilt lasts */
        }
    }
    TEST_ASSERT_EQUAL_INT(4, stub_safety_report_count);
}

/* UT-29: 34 deg is below the 35 deg limit, even for a second. Roll counts
 * the same as pitch. */
void
test_UT29_below_limit_is_ignored_and_roll_counts (void)
{
    int n = 0;

    for (n = 0; n < 50; n++)
    {
        tilt_check(34.0f, -34.0f);
    }
    TEST_ASSERT_EQUAL_INT(0, stub_safety_report_count);

    for (n = 0; n < 10; n++)
    {
        tilt_check(0.0f, -36.0f);
    }
    TEST_ASSERT_EQUAL_INT(1, stub_safety_report_count);
}

int
main (void)
{
    UNITY_BEGIN();
    RUN_TEST(test_UT13a_6mm_is_not_a_hump);
    RUN_TEST(test_UT13b_12mm_is_a_hump);
    RUN_TEST(test_UT13c_9mm_is_not_a_hump);
    RUN_TEST(test_UT14_highest_hump_kept_until_reset);
    RUN_TEST(test_UT28a_one_sample_spike_rejected);
    RUN_TEST(test_UT28b_confirmed_jump_is_a_collision);
    RUN_TEST(test_UT28c_saturated_sample_ignored);
    RUN_TEST(test_UT29_overtilt_needs_200ms);
    RUN_TEST(test_UT29_below_limit_is_ignored_and_roll_counts);

    return (UNITY_END());
}
