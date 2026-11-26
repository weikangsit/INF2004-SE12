/** @file    test_comms.c
 *  @brief   Unit tests UT-30 (R-10: reconnect back-off) and UT-31 (R-10,
 *           R-8: the publish queue never blocks) for comms.c.
 *
 *  The functions are static, so the file under test is included here. lwIP,
 *  the MQTT client and the radio task are fakes. The message buffer fake
 *  follows the T-Kernel size rule (4-byte header + size rounded up to 4),
 *  which decides how many frames fit in the 360-byte queue.
 */
#include <string.h>

#include "unity.h"
#include "stub_kernel.h"
#include "stub_modules.h"

#include "comms.c"

#define START_MS  (1000U)

void
setUp (void)
{
    stub_kernel_reset();
    stub_modules_reset();
    stub_time_set_ms(START_MS);
    (void) memset((void *) &g_link, 0, sizeof(g_link));
    g_drop_count    = 0U;
    g_drop_reported = 0U;
    g_state_ms      = 0U;
    gp_mqtt         = NULL;
    comms_init();
}

void
tearDown (void)
{
}

/* Start a back-off and count the milliseconds until the link leaves it. */
static uint32_t
backoff_delay_ms (void)
{
    uint32_t waited = 0U;

    link_start_backoff();
    TEST_ASSERT_EQUAL_INT(LINK_BACKOFF, g_link.state);

    while ((LINK_BACKOFF == g_link.state) && (waited < 20000U))
    {
        stub_time_advance_ms(1U);
        waited++;
        link_backoff();
    }

    return (waited);
}

/* UT-30: five failed attempts in a row wait 0.5, 1, 2, 4, 4 s; a success
 * starts again at 0.5 s. */
void
test_UT30_backoff_doubles_to_4s_and_resets_on_success (void)
{
    TEST_ASSERT_EQUAL_UINT32(500U, backoff_delay_ms());
    TEST_ASSERT_EQUAL_UINT32(1000U, backoff_delay_ms());
    TEST_ASSERT_EQUAL_UINT32(2000U, backoff_delay_ms());
    TEST_ASSERT_EQUAL_UINT32(4000U, backoff_delay_ms());
    TEST_ASSERT_EQUAL_UINT32(4000U, backoff_delay_ms());     /* capped */
    TEST_ASSERT_EQUAL_UINT32(3U, g_link.backoff_step);

    link_go_online();                                        /* a success */
    TEST_ASSERT_EQUAL_INT(LINK_ONLINE, g_link.state);
    TEST_ASSERT_EQUAL_UINT32(0U, g_link.backoff_step);

    g_link.state = LINK_IDLE;
    g_link.b_connected = false;
    TEST_ASSERT_EQUAL_UINT32(500U, backoff_delay_ms());
}

/* UT-30: no retry before the delay is over. */
void
test_UT30_no_retry_early (void)
{
    link_start_backoff();
    stub_time_advance_ms(499U);
    link_backoff();
    TEST_ASSERT_EQUAL_INT(LINK_BACKOFF, g_link.state);
    stub_time_advance_ms(1U);
    link_backoff();
    TEST_ASSERT_EQUAL_INT(LINK_IDLE, g_link.state);
}

static uint32_t
publish_frame (uint8_t marker)
{
    uint8_t frame[PROTO_TELEM_LEN];

    (void) memset(frame, 0, sizeof(frame));
    frame[0] = marker;

    return (comms_publish(COMMS_TOPIC_TELEMETRY, frame, PROTO_TELEM_LEN) ? 1U : 0U);
}

/* UT-31: the 360-byte queue holds 15 telemetry frames (24 bytes each).
 * Ten more are accepted, each one dropping the oldest. No call waits. */
void
test_UT31_full_queue_drops_oldest_and_never_waits (void)
{
    uint8_t  buf[PUB_MSG_MAX_LEN];
    uint8_t  marker = 0U;
    INT      len    = 0;
    uint32_t count  = 0U;
    uint8_t  first  = 0xFFU;
    uint8_t  last   = 0U;

    TEST_ASSERT_EQUAL_UINT32(360U, PUB_Q_BYTES);

    for (marker = 0U; marker < 15U; marker++)
    {
        TEST_ASSERT_EQUAL_UINT32(1U, publish_frame(marker));
    }
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0U, g_drop_count, "15 frames fit");

    TEST_ASSERT_EQUAL_UINT32(1U, publish_frame(15U));
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(1U, g_drop_count, "the 16th drops one");

    for (marker = 16U; marker < 25U; marker++)
    {
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(1U, publish_frame(marker),
                                         "accepted although full");
    }
    TEST_ASSERT_EQUAL_UINT8(10U, g_drop_count);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, stub_nonpoll_calls, "never waited");

    /* What is left: the newest 15, in order (markers 10 to 24). */
    while ((len = tk_rcv_mbf(g_pub_q, buf, TMO_POL)) > 0)
    {
        TEST_ASSERT_EQUAL_UINT8(COMMS_TOPIC_TELEMETRY, buf[0]);
        if (0U == count)
        {
            first = buf[PUB_HEADER_LEN];
        }
        last = buf[PUB_HEADER_LEN];
        count++;
    }
    TEST_ASSERT_EQUAL_UINT32(15U, count);
    TEST_ASSERT_EQUAL_UINT8(10U, first);
    TEST_ASSERT_EQUAL_UINT8(24U, last);
}

/* UT-31: a frame that is empty or too long is refused without touching
 * the queue. */
void
test_UT31_bad_length_refused (void)
{
    uint8_t frame[PROTO_TELEM_LEN + 1U] = { 0U };

    TEST_ASSERT_FALSE(comms_publish(COMMS_TOPIC_TELEMETRY, frame, 0U));
    TEST_ASSERT_FALSE(comms_publish(COMMS_TOPIC_TELEMETRY, frame, PROTO_MSG_MAX_LEN + 1U));
    TEST_ASSERT_FALSE(comms_publish(COMMS_TOPIC_COUNT, frame, 1U));
    TEST_ASSERT_EQUAL_UINT8(0U, g_drop_count);
}

int
main (void)
{
    UNITY_BEGIN();
    RUN_TEST(test_UT30_backoff_doubles_to_4s_and_resets_on_success);
    RUN_TEST(test_UT30_no_retry_early);
    RUN_TEST(test_UT31_full_queue_drops_oldest_and_never_waits);
    RUN_TEST(test_UT31_bad_length_refused);

    return (UNITY_END());
}
