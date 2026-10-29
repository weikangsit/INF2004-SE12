/**
 * @file    telemetry.c
 * @brief   Telemetry Publisher Task (Buddy 1) - lowest priority.
 *
 *  Every 100 ms: forward queued EVENT frames and publish one 32-byte
 *  TELEM frame built from the motion state record (FR6, proto.h).
 */
#include <math.h>
#include <string.h>

#include "port.h"

#include "app.h"
#include "barcode_map.h"
#include "imu.h"
#include "ultrasonic.h"

static int16_t to_i16(float v)
{
    if (v > 32767.0f)  { return 32767;  }
    if (v < -32768.0f) { return -32768; }
    return (int16_t)lroundf(v);
}

void telem_event(proto_event_t kind, uint8_t code, int16_t v1, int16_t v2)
{
    telem_msg_t m;

    m.frame[0] = PROTO_EVENT;
    m.frame[1] = (uint8_t)kind;
    m.frame[2] = code;
    m.frame[3] = 0U;
    m.frame[4] = (uint8_t)((uint16_t)v1 & 0xFFU);
    m.frame[5] = (uint8_t)((uint16_t)v1 >> 8);
    m.frame[6] = (uint8_t)((uint16_t)v2 & 0xFFU);
    m.frame[7] = (uint8_t)((uint16_t)v2 >> 8);
    (void)tk_snd_mbf(g_telemetry_q, &m, (INT)sizeof(m), TMO_POL);
}

void telemetry_task(INT stacd, void *exinf)
{
    uint64_t next_us = time_us_64();

    (void)stacd;
    (void)exinf;

    for (;;)
    {
        telem_msg_t    m;
        motion_state_t st;
        proto_telem_t  f;
        float          obst_cm;

        period_wait(&next_us, PERIOD_TELEM_MS);
        heartbeat(HB_TELEM);

        while (tk_rcv_mbf(g_telemetry_q, &m, TMO_POL) > 0)
        {
            (void)comms_publish("event", m.frame, PROTO_EVENT_LEN);
        }

        state_read(&st);
        obst_cm = us_front_cm();

        (void)memset(&f, 0, sizeof(f));
        f.type         = PROTO_TELEM;
        f.state        = st.sys_state;
        f.flags        = (uint8_t)((st.line_seen ? TF_LINE_SEEN : 0U) |
                                   (safety_is_latched() ? TF_SAFETY_LATCHED : 0U) |
                                   (imu_is_ok() ? TF_IMU_OK : 0U) |
                                   (barcode_map_is_loaded() ? TF_CFG_LOADED : 0U) |
                                   (st.ir_l_black ? TF_IR_L_BLACK : 0U) |
                                   (st.ir_r_black ? TF_IR_R_BLACK : 0U));
        f.motion_class = st.motion_class;
        f.speed_l      = to_i16(st.speed_l_mps * 1000.0f);
        f.speed_r      = to_i16(st.speed_r_mps * 1000.0f);
        f.pitch        = to_i16(st.pitch_deg * 10.0f);
        f.roll         = to_i16(st.roll_deg * 10.0f);
        f.yaw          = (uint16_t)to_i16(st.heading_deg * 10.0f);
        f.turn_rate    = to_i16(st.turn_rate_dps * 10.0f);
        f.obstacle     = (obst_cm > 600.0f) ? 0xFFFFU : (uint16_t)lroundf(obst_cm * 10.0f);
        f.duty_l       = (int8_t)lroundf(st.duty_l * 100.0f);
        f.duty_r       = (int8_t)lroundf(st.duty_r * 100.0f);
        f.distance     = (int32_t)lroundf(st.dist_m * 1000.0f);
        f.line_pos     = st.line_pos;
        f.barcode      = (uint8_t)st.last_barcode;
        f.hump_peak    = (uint16_t)to_i16(st.hump_peak_mm * 10.0f);
        f.uptime_ms    = now_ms();
        /* RP2040 is little-endian, so the packed struct is the wire format */
        (void)comms_publish("telemetry", (const uint8_t *)&f, (uint32_t)sizeof(f));
    }
}
