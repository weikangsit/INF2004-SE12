/**
 * @file    app.h
 * @brief   Shared kernel objects, message types and the motion state
 *          record used by every task (Section 4, Task Interaction).
 */
#ifndef APP_H
#define APP_H

#include <stdbool.h>
#include <stddef.h>          /* size_t (PROHIBIT_DEF_SIZE_T, see CMake) */
#include <stdint.h>
#include <tk/tkernel.h>

#include "car_config.h"
#include "proto.h"

/* ------------------------------------------------------------------ */
/* Heartbeat bits in hb_flg - one per supervised task                  */
/* ------------------------------------------------------------------ */
#define HB_LINE      (1U << 0)
#define HB_MOTION    (1U << 1)
#define HB_IMU       (1U << 2)
#define HB_US        (1U << 3)
#define HB_NAV       (1U << 4)
#define HB_COMMS     (1U << 5)   /* unused: radio task is template-owned */
#define HB_TELEM     (1U << 6)
/* Supervised by the Safety Monitor. Comms is not: WiFi loss must never
 * stop the car (NFR7), and its task belongs to the template.          */
#define HB_ALL       (0x7FU & ~HB_COMMS)

/* Bits in evt_flg (ISR -> task signals and safety events)              */
#define EVT_BARCODE_EDGE   (1U << 0)  /* ISR: barcode edge stored        */
#define EVT_ECHO_DONE      (1U << 1)  /* ISR: echo falling edge          */
#define EVT_COLLISION      (1U << 2)  /* IMU -> Safety                   */
#define EVT_LINE_LOST      (1U << 3)  /* Line -> Safety / Nav            */
#define EVT_ESTOP          (1U << 4)  /* Nav (remote) -> Safety          */
#define EVT_OVERTILT       (1U << 5)  /* IMU -> Safety                   */

/* Bits in line_evt_flg (Line -> Nav)                                   */
#define LEVT_JUNCTION      (1U << 0)
#define LEVT_LOST          (1U << 1)
#define LEVT_REGAINED      (1U << 2)
#define LEVT_SENSOR_FAIL   (1U << 3)

/* ------------------------------------------------------------------ */
/* Message types                                                       */
/* ------------------------------------------------------------------ */
typedef struct
{
    int8_t   pos;          /* -1 line under LEFT sensor (steer left),
                              0 centred, +1 line under RIGHT sensor   */
    bool     line_seen;    /* at least one sensor on black             */
    bool     junction;     /* both on black                            */
    uint32_t state_ms;     /* how long pos has been unchanged          */
} line_pos_t;

/* Decoded barcode letter 'A'..'Z'. What it means is looked up in the
 * MQTT-configured table (barcode_map.h) by the Navigation FSM.        */
typedef char bar_cmd_t;

typedef enum
{
    MCMD_STOP = 0,
    MCMD_FWD,          /* arg_speed m/s, arg_dist m (0 = until next cmd) */
    MCMD_BWD,          /* arg_dist m                                    */
    MCMD_TURN_L,       /* arg_angle deg                                 */
    MCMD_TURN_R,
    MCMD_UTURN,
    MCMD_ENC_CAL,      /* self-test: learn each encoder's count direction */
    MCMD_MOTOR_TEST    /* bench: left alone, then right alone, open loop */
} motion_cmd_id_t;

typedef struct
{
    motion_cmd_id_t id;
    float           speed;     /* m/s                                    */
    float           dist;      /* m                                      */
    float           angle;     /* deg                                    */
    bool            follow_line;
    bool            end_on_line;  /* turns: stop when new branch seen    */
} motion_cmd_t;

typedef enum
{
    MEVT_DONE = 0,
    MEVT_STALL,
    MEVT_HUMP_START,
    MEVT_HUMP_END,
    MEVT_IMPACT
} motion_evt_id_t;

typedef struct
{
    motion_evt_id_t id;
    float           value;     /* hump height mm, etc.                   */
} motion_evt_t;

typedef enum
{
    OBS_NEAR = 0,
    OBS_PROFILE,
    OBS_ROUTE_LEFT,
    OBS_ROUTE_RIGHT,
    OBS_ROUTE_CONTINUE,
    OBS_ROUTE_REVERSE,
    OBS_NO_ROUTE
} obstacle_kind_t;

typedef struct
{
    obstacle_kind_t kind;
    float           range_cm;      /* closest point                      */
    int16_t         angle_deg;     /* bearing of closest point           */
    float           width_cm;
    float           clear_left_cm;
    float           clear_right_cm;
} obstacle_msg_t;

typedef enum
{
    SCAN_REQ_PROFILE = 0,
    SCAN_REQ_ABORT
} scan_req_t;

typedef enum
{
    RCMD_START = 0,
    RCMD_STOP,
    RCMD_ESTOP,
    RCMD_FINISH,
    RCMD_RESET,
    RCMD_MOTOR_TEST    /* bench test: each wheel alone, open loop */
} remote_cmd_t;

/* Binary event frame (proto.h EVENT), queued on telemetry_q */
typedef struct
{
    uint8_t frame[PROTO_EVENT_LEN];
} telem_msg_t;

/* ------------------------------------------------------------------ */
/* Motion state record (state_mtx). One writer per field.              */
/* ------------------------------------------------------------------ */
typedef struct
{
    /* written by Motion Control */
    float duty_l;
    float duty_r;
    float speed_l_mps;
    float speed_r_mps;
    float dist_m;          /* mean of both wheels, signed, since boot   */
    /* written by IMU Terrain */
    float pitch_deg;
    float roll_deg;
    float heading_deg;     /* 0..360                                    */
    float turn_rate_dps;
    float hump_peak_mm;
    uint8_t motion_class;  /* 0 stationary, 1 moving, 2 turning, 3 hump */
    /* written by Line / Barcode */
    int8_t  line_pos;
    bool    line_seen;
    char    last_barcode;
    /* written by Navigation FSM */
    uint8_t sys_state;
} motion_state_t;

/* ------------------------------------------------------------------ */
/* Kernel object IDs (created in main.c)                               */
/* ------------------------------------------------------------------ */
extern ID g_line_pos_q;
extern ID g_bar_cmd_q;
extern ID g_motion_cmd_q;
extern ID g_motion_evt_q;
extern ID g_obstacle_q;
extern ID g_scan_req_q;
extern ID g_remote_cmd_q;
extern ID g_telemetry_q;
extern ID g_hb_flg;
extern ID g_evt_flg;
extern ID g_line_evt_flg;
extern ID g_state_mtx;

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */
void     state_read(motion_state_t *p_out);
void     state_lock(void);
void     state_unlock(void);
motion_state_t *state_raw(void);          /* only between lock/unlock */

uint32_t now_ms(void);
void     period_wait(uint64_t *p_next_us, uint32_t period_ms);
void     heartbeat(uint32_t bit);
void     telem_event(proto_event_t kind, uint8_t code, int16_t v1, int16_t v2);

/* Task entry points */
void line_barcode_task(INT stacd, void *exinf);
void motion_task(INT stacd, void *exinf);
void imu_task(INT stacd, void *exinf);
void ultrasonic_task(INT stacd, void *exinf);
void nav_fsm_task(INT stacd, void *exinf);
void telemetry_task(INT stacd, void *exinf);
void safety_task(INT stacd, void *exinf);

/* Motion API (Section 6.2) - posts to motion_cmd_q */
void moveForward(float speed_mps, float dist_m, bool follow_line);
void moveBackward(float dist_m);
void turnLeft(float angle_deg, bool end_on_line);
void turnRight(float angle_deg, bool end_on_line);
void stop(void);

/* Comms entry used by the telemetry publisher: binary payload, copied */
bool comms_publish(const char *p_subtopic, const uint8_t *p_data, uint32_t len);
bool comms_is_online(void);

/* Safety latch query, used by Motion Control and Nav */
bool safety_is_latched(void);
void safety_clear(void);

#endif /* APP_H */
