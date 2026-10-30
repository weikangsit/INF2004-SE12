/** @file    imu_terrain.c
 *  @brief   IMU Terrain task (Buddy 4), see imu_terrain.h.
 *
 *  - Pitch and roll from the low-pass-filtered accelerometer; the sensor
 *    must be mounted X forward, Y left, Z up.
 *  - Tilt-compensated magnetometer heading and turn rate (no gyroscope).
 *  - Hump: a nose-up pitch is a candidate; it becomes a hump only after
 *    the car has climbed HUMP_MIN_MM (sin(pitch) x encoder distance), so
 *    acceleration jerks are not reported as humps.
 *  - Impact (collision): sudden horizontal jump of one sample.
 *  - Over-tilt: beyond OVERTILT_DEG for OVERTILT_HOLD_MS.
 *  - Hard-iron magnetometer calibration on request (GP21 held in READY):
 *    the car spins once while this task records min / max; the task keeps
 *    its 20 ms period and heartbeat throughout.
 *
 *  NOTE: Floats are used only for the local signal processing (Tribunal 1:
 *  full precision inside a task, packed shared data).
 *  NOTE: (void) casts mark return values that are deliberately ignored.
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <tm/tmonitor.h>

#include "port.h"

#include "car_config.h"
#include "i2c_drv.h"
#include "imu_terrain.h"
#include "proto.h"
#include "safety.h"
#include "systime.h"
#include "telemetry.h"
#include "vehicle_state.h"

/* LSM303DLHC registers and settings */
#define ACC_CTRL_REG1        (0x20U)
#define ACC_CTRL_REG4        (0x23U)
#define ACC_OUT_X_L          (0x28U)
#define MAG_CRA_REG          (0x00U)
#define MAG_CRB_REG          (0x01U)
#define MAG_MR_REG           (0x02U)
#define MAG_OUT_X_H          (0x03U)
#define AUTO_INC             (0x80U)
#define ACC_REG1_100HZ_XYZ   (0x57U)
#define ACC_REG4_2G_HR       (0x08U)
#define MAG_CRA_75HZ         (0x18U)
#define MAG_CRB_1G3          (0x20U)
#define MAG_MR_CONTINUOUS    (0x00U)

#define ACC_LSB_PER_G        (16384.0f)   /* +/-2 g, 16-bit left-justified */
#define MAG_LSB_XY           (1100.0f)    /* +/-1.3 gauss                 */
#define MAG_LSB_Z            (980.0f)
#define SAMPLE_BYTES         (6U)
#define REG_WRITE_BYTES      (2U)
#define I2C_TIMEOUT_US       (2000U)
#define I2C_FAIL_LIMIT       (5U)

#define RAD2DEG              (57.2957795f)
#define DT_S                 ((float32_t) PERIOD_IMU_MS / 1000.0f)
#define RATE_ALPHA           (0.3f)
#define FULL_TURN_DEG        (360.0f)
#define HALF_TURN_DEG        (180.0f)
#define HALVES               (2.0f)       /* deg / mm -> 0.5 steps        */
#define MM_PER_M             (1000.0f)
#define HUMP_END_FRACTION    (0.2f)
#define TURNING_DPS          (30.0f)
#define MOVING_MMS           (20)         /* sum of wheel speeds          */
#define TENTHS               (10.0f)
#define AXES                 (3U)
#define AXIS_X               (0U)
#define AXIS_Y               (1U)
#define AXIS_Z               (2U)
#define BIG                  (1.0e6f)
#define EVT_Q_DEPTH          (4U)
#define MBF_SLOT_BYTES       (8U)         /* 4-byte header + 1 byte       */
#define MAG_CAL_FAILED       (1U)

#define CLASS_STILL          (0U)
#define CLASS_MOVING         (1U)
#define CLASS_TURNING        (2U)
#define CLASS_HUMP           (3U)

typedef struct
{
    float32_t axis[AXES];
} vec3_t;

/** Task-private processing state. */
typedef struct
{
    vec3_t    acc_f;           /* low-pass-filtered acceleration, g      */
    vec3_t    acc_prev;        /* previous raw sample (impact check)     */
    bool      b_prev_valid;    /* acc_prev holds a real sample           */
    float32_t prev_hdg;
    float32_t rate;            /* deg/s                                  */
    int32_t   prev_dist_mm;
    bool      b_candidate;     /* nose-up, height not yet confirmed      */
    bool      b_in_hump;
    float32_t hump_h;          /* mm climbed in this hump                */
    float32_t hump_max;
    float32_t run_peak;        /* highest hump of the run                */
    uint32_t  tilt_ms;         /* time beyond OVERTILT_DEG               */
    uint32_t  fails;           /* consecutive failed reads               */
    vec3_t    cal_min;         /* magnetometer calibration               */
    vec3_t    cal_max;
    bool      b_cal_active;
} imu_ctx_t;

static ID            g_evt_q            = 0;
static volatile bool g_b_ok             = false;
static volatile bool g_b_cal_start_req  = false;   /* Nav sets, task clears */
static volatile bool g_b_cal_finish_req = false;
static bool          g_b_mag_calibrated = false;   /* task only            */
static vec3_t        g_mag_off;                    /* task only, gauss     */
static imu_ctx_t     g_ctx;                        /* task only            */

static bool      sensor_init (void);
static bool      reg_write (uint8_t addr, uint8_t reg, uint8_t value);
static bool      regs_read (uint8_t addr, uint8_t reg, uint8_t * p_buf);
static bool      acc_read (vec3_t * p_acc);
static bool      mag_read (vec3_t * p_mag);
static int16_t   le16 (uint8_t const * p_bytes);
static int16_t   be16 (uint8_t const * p_bytes);
static void      sample_process (vec3_t const * p_acc, vec3_t const * p_mag);
static void      impact_check (vec3_t const * p_acc);
static void      tilt_check (float32_t pitch_deg, float32_t roll_deg);
static void      hump_track (float32_t pitch, float32_t pitch_deg,
                             int32_t dist_mm);
static float32_t heading_compute (vec3_t const * p_mag, float32_t pitch,
                                  float32_t roll);
static void      mag_cal_step (vec3_t const * p_mag);
static void      event_post (imu_terrain_event_t event);
static int32_t   round_i (float32_t value);

/*!
 * @brief  Create the terrain event queue. Call from usermain() before the
 *         tasks start.
 */
void
imu_terrain_init (void)
{
    T_CMBF create;

    (void) memset(&g_ctx, 0, sizeof(g_ctx));
    (void) memset(&g_mag_off, 0, sizeof(g_mag_off));
    g_ctx.acc_f.axis[AXIS_Z] = 1.0f;              /* start level          */

    (void) memset(&create, 0, sizeof(create));
    create.mbfatr = TA_TFIFO;
    create.bufsz  = (SZ) (EVT_Q_DEPTH * MBF_SLOT_BYTES);       /* 32 B    */
    create.maxmsz = (INT) sizeof(uint8_t);
    g_evt_q       = tk_cre_mbf(&create);
}

/*!
 * @brief  IMU Terrain task body (priority PRI_IMU, 20 ms).
 * @param  stacd    Start code (unused).
 * @param  p_exinf  Extended information (unused).
 */
void
imu_terrain_task (INT stacd, void * p_exinf)
{
    uint64_t next_us = time_us_64();

    (void) stacd;
    (void) p_exinf;

    g_b_ok = sensor_init();

    for (;;)
    {
        vec3_t acc;
        vec3_t mag;

        systime_period_wait(&next_us, PERIOD_IMU_MS);
        safety_heartbeat(SAFETY_HB_IMU);

        if (acc_read(&acc) && mag_read(&mag))
        {
            g_ctx.fails = 0U;
            g_b_ok      = true;
            mag_cal_step(&mag);
            sample_process(&acc, &mag);
        }
        else
        {
            g_ctx.fails++;
            if (g_ctx.fails >= I2C_FAIL_LIMIT)
            {
                g_b_ok = false;                   /* Nav: sensor failure  */
            }
        }
    }
}

/*!
 * @brief  Report whether the IMU is answering.
 * @return False after I2C_FAIL_LIMIT failed reads in a row.
 */
bool
imu_terrain_is_ok (void)
{
    return (g_b_ok);
}

/*!
 * @brief  Take one terrain event, if any (Navigation FSM).
 * @param  p_event  Output event.
 * @return True if an event was taken.
 */
bool
imu_terrain_event_take (imu_terrain_event_t * p_event)
{
    uint8_t    msg   = 0U;
    bool const b_got = (tk_rcv_mbf(g_evt_q, &msg, TMO_POL) > 0);

    if (b_got)
    {
        /* Only the two event values are ever queued. */
        *p_event = (imu_terrain_event_t) msg;
    }

    return (b_got);
}

/*!
 * @brief  Start recording the magnetometer for calibration (Nav, while
 *         the car starts its slow spin).
 */
void
imu_terrain_mag_cal_start (void)
{
    g_b_cal_start_req = true;
}

/*!
 * @brief  Finish the calibration (Nav, when the spin is done). The task
 *         computes the offsets and reports EV_MAG_CAL.
 */
void
imu_terrain_mag_cal_finish (void)
{
    g_b_cal_finish_req = true;
}

/*!
 * @brief  Configure the accelerometer and magnetometer.
 * @return True if every register write was acknowledged.
 */
static bool
sensor_init (void)
{
    i2c_drv_init(PIN_IMU_SDA, PIN_IMU_SCL, IMU_I2C_HZ);

    return (reg_write(IMU_ACC_ADDR, ACC_CTRL_REG1, ACC_REG1_100HZ_XYZ) &&
            reg_write(IMU_ACC_ADDR, ACC_CTRL_REG4, ACC_REG4_2G_HR) &&
            reg_write(IMU_MAG_ADDR, MAG_CRA_REG, MAG_CRA_75HZ) &&
            reg_write(IMU_MAG_ADDR, MAG_CRB_REG, MAG_CRB_1G3) &&
            reg_write(IMU_MAG_ADDR, MAG_MR_REG, MAG_MR_CONTINUOUS));
}

/*!
 * @brief  Write one register.
 * @return True on ACK.
 */
static bool
reg_write (uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t const bytes[REG_WRITE_BYTES] = { reg, value };

    return (i2c_drv_write(addr, bytes, REG_WRITE_BYTES, false,
                          I2C_TIMEOUT_US));
}

/*!
 * @brief  Read SAMPLE_BYTES registers starting at reg.
 * @return True on success.
 */
static bool
regs_read (uint8_t addr, uint8_t reg, uint8_t * p_buf)
{
    bool b_ok = i2c_drv_write(addr, &reg, 1U, true, I2C_TIMEOUT_US);

    if (b_ok)
    {
        b_ok = i2c_drv_read(addr, p_buf, SAMPLE_BYTES, I2C_TIMEOUT_US * 2U);
    }

    return (b_ok);
}

/*!
 * @brief  Read the accelerometer.
 * @param  p_acc  Output, g.
 * @return True on success.
 */
static bool
acc_read (vec3_t * p_acc)
{
    uint8_t    raw[SAMPLE_BYTES];
    bool const b_ok = regs_read(IMU_ACC_ADDR,
                                (uint8_t) (ACC_OUT_X_L | AUTO_INC), raw);

    if (b_ok)
    {
        p_acc->axis[AXIS_X] = (float32_t) le16(&raw[0]) / ACC_LSB_PER_G;
        p_acc->axis[AXIS_Y] = (float32_t) le16(&raw[2]) / ACC_LSB_PER_G;
        p_acc->axis[AXIS_Z] = (float32_t) le16(&raw[4]) / ACC_LSB_PER_G;
    }

    return (b_ok);
}

/*!
 * @brief  Read the magnetometer (register order X, Z, Y, high byte first)
 *         and remove the hard-iron offsets.
 * @param  p_mag  Output, gauss.
 * @return True on success.
 */
static bool
mag_read (vec3_t * p_mag)
{
    uint8_t    raw[SAMPLE_BYTES];
    bool const b_ok = regs_read(IMU_MAG_ADDR, MAG_OUT_X_H, raw);

    if (b_ok)
    {
        p_mag->axis[AXIS_X] = ((float32_t) be16(&raw[0]) / MAG_LSB_XY) -
                              g_mag_off.axis[AXIS_X];
        p_mag->axis[AXIS_Z] = ((float32_t) be16(&raw[2]) / MAG_LSB_Z) -
                              g_mag_off.axis[AXIS_Z];
        p_mag->axis[AXIS_Y] = ((float32_t) be16(&raw[4]) / MAG_LSB_XY) -
                              g_mag_off.axis[AXIS_Y];
    }

    return (b_ok);
}

/*!
 * @brief  Little-endian 16-bit two's complement value.
 * @return The value.
 */
static int16_t
le16 (uint8_t const * p_bytes)
{
    uint32_t const word = (uint32_t) p_bytes[0] |
                          ((uint32_t) p_bytes[1] << 8);

    /* The 16-bit pattern is a two's complement register value. */
    return ((int16_t) (uint16_t) word);
}

/*!
 * @brief  Big-endian 16-bit two's complement value.
 * @return The value.
 */
static int16_t
be16 (uint8_t const * p_bytes)
{
    uint32_t const word = ((uint32_t) p_bytes[0] << 8) |
                          (uint32_t) p_bytes[1];

    /* The 16-bit pattern is a two's complement register value. */
    return ((int16_t) (uint16_t) word);
}

/*!
 * @brief  Process one good sample and publish the results.
 * @param  p_acc  Acceleration, g.
 * @param  p_mag  Magnetic field, gauss (offsets removed).
 */
static void
sample_process (vec3_t const * p_acc, vec3_t const * p_mag)
{
    vehicle_state_t state;
    vehicle_state_t out;
    float32_t       pitch = 0.0f;
    float32_t       roll  = 0.0f;
    float32_t       hdg   = 0.0f;
    float32_t       hdg_step    = 0.0f;
    uint32_t        idx   = 0U;
    uint8_t         cls   = CLASS_STILL;

    impact_check(p_acc);

    for (idx = 0U; idx < AXES; idx++)
    {
        g_ctx.acc_f.axis[idx] += IMU_LPF_ALPHA *
                                 (p_acc->axis[idx] - g_ctx.acc_f.axis[idx]);
    }

    /* X forward, Y left, Z up: nose-up pitch is positive. */
    {
        float32_t const acc_x = g_ctx.acc_f.axis[AXIS_X];
        float32_t const acc_y = g_ctx.acc_f.axis[AXIS_Y];
        float32_t const acc_z = g_ctx.acc_f.axis[AXIS_Z];

        pitch = atan2f(acc_x, sqrtf((acc_y * acc_y) + (acc_z * acc_z)));
        roll  = atan2f(acc_y, acc_z);
    }

    hdg = heading_compute(p_mag, pitch, roll);

    hdg_step = hdg - g_ctx.prev_hdg;
    if (hdg_step > HALF_TURN_DEG)
    {
        hdg_step -= FULL_TURN_DEG;
    }
    else if (hdg_step < -HALF_TURN_DEG)
    {
        hdg_step += FULL_TURN_DEG;
    }
    else
    {
        /* No wrap. */
    }
    g_ctx.rate    += RATE_ALPHA * ((hdg_step / DT_S) - g_ctx.rate);
    g_ctx.prev_hdg = hdg;

    tilt_check(pitch * RAD2DEG, roll * RAD2DEG);

    vehicle_state_read(&state);
    hump_track(pitch, pitch * RAD2DEG, state.dist_mm);

    if (g_ctx.b_in_hump)
    {
        cls = CLASS_HUMP;
    }
    else if (fabsf(g_ctx.rate) > TURNING_DPS)
    {
        cls = CLASS_TURNING;
    }
    else if ((abs(state.speed_l_mms) + abs(state.speed_r_mms)) > MOVING_MMS)
    {
        cls = CLASS_MOVING;
    }
    else
    {
        cls = CLASS_STILL;
    }

    (void) memset(&out, 0, sizeof(out));
    /* Each value is clamped again to its packed width by vehicle_state. */
    out.pitch_half_deg    = (int16_t) round_i(pitch * RAD2DEG * HALVES);
    out.roll_deg          = (int16_t) round_i(roll * RAD2DEG);
    out.heading_half_deg  = (uint16_t) round_i(hdg * HALVES);
    out.turn_rate_dps     = (int16_t) round_i(g_ctx.rate);
    out.hump_peak_half_mm = (uint16_t) round_i(
        fmaxf(g_ctx.run_peak, g_ctx.b_in_hump ? g_ctx.hump_max : 0.0f) *
        HALVES);
    out.motion_class      = cls;
    out.b_imu_valid       = true;
    out.b_mag_calibrated  = g_b_mag_calibrated;
    out.imu_stamp         = systime_stamp();
    vehicle_state_set_imu(&out);
}

/*!
 * @brief  Collision: a sudden horizontal jump between two raw samples.
 *         The first sample after boot only sets the reference.
 * @param  p_acc  Raw acceleration, g.
 */
static void
impact_check (vec3_t const * p_acc)
{
    if (g_ctx.b_prev_valid &&
        ((fabsf(p_acc->axis[AXIS_X] - g_ctx.acc_prev.axis[AXIS_X]) >
          IMPACT_G) ||
         (fabsf(p_acc->axis[AXIS_Y] - g_ctx.acc_prev.axis[AXIS_Y]) >
          IMPACT_G)))
    {
        safety_report(SAFETY_EVT_COLLISION);
    }

    g_ctx.acc_prev     = *p_acc;
    g_ctx.b_prev_valid = true;
}

/*!
 * @brief  Over-tilt: only once it has lasted OVERTILT_HOLD_MS (motor jerks
 *         look like tilt to the accelerometer but are shorter).
 * @param  pitch_deg  Pitch, degrees.
 * @param  roll_deg   Roll, degrees.
 */
static void
tilt_check (float32_t pitch_deg, float32_t roll_deg)
{
    if ((fabsf(pitch_deg) > OVERTILT_DEG) || (fabsf(roll_deg) > OVERTILT_DEG))
    {
        g_ctx.tilt_ms += PERIOD_IMU_MS;

        if (g_ctx.tilt_ms >= OVERTILT_HOLD_MS)
        {
            if (g_ctx.tilt_ms < (OVERTILT_HOLD_MS + PERIOD_IMU_MS))
            {
                /* Once per tilt: the angles that tripped it. */
                tm_printf((UB *) "imu: over-tilt pitch %d roll %d (x0.1)\n",
                          (int) round_i(pitch_deg * TENTHS),
                          (int) round_i(roll_deg * TENTHS));
            }

            safety_report(SAFETY_EVT_OVERTILT);
        }
    }
    else
    {
        g_ctx.tilt_ms = 0U;
    }
}

/*!
 * @brief  Hump detection: candidate on nose-up pitch, confirmed after the
 *         car has climbed HUMP_MIN_MM, ended when level and back down.
 * @param  pitch      Pitch, radians.
 * @param  pitch_deg  Pitch, degrees.
 * @param  dist_mm    Distance travelled since boot (Motion Control).
 */
static void
hump_track (float32_t pitch, float32_t pitch_deg, int32_t dist_mm)
{
    /* Distance step per sample is a few mm, so the float is exact. */
    float32_t const climb = sinf(pitch) *
                            (float32_t) (dist_mm - g_ctx.prev_dist_mm);

    g_ctx.prev_dist_mm = dist_mm;

    if ((!g_ctx.b_in_hump) && (!g_ctx.b_candidate) &&
        (pitch_deg > HUMP_ON_DEG))
    {
        g_ctx.b_candidate = true;
        g_ctx.hump_h      = 0.0f;
        g_ctx.hump_max    = 0.0f;
    }

    if (g_ctx.b_candidate)
    {
        g_ctx.hump_h  += climb;
        g_ctx.hump_max = fmaxf(g_ctx.hump_max, g_ctx.hump_h);

        if (g_ctx.hump_h >= HUMP_MIN_MM)
        {
            g_ctx.b_candidate = false;
            g_ctx.b_in_hump   = true;
            event_post(IMU_EVT_HUMP_START);
        }
        else if (pitch_deg < HUMP_OFF_DEG)
        {
            g_ctx.b_candidate = false;            /* only a jerk          */
        }
        else
        {
            /* Still climbing. */
        }
    }
    else if (g_ctx.b_in_hump)
    {
        g_ctx.hump_h  += climb;
        g_ctx.hump_max = fmaxf(g_ctx.hump_max, g_ctx.hump_h);

        if ((fabsf(pitch_deg) < HUMP_OFF_DEG) &&
            (g_ctx.hump_h < (HUMP_END_FRACTION * g_ctx.hump_max)))
        {
            g_ctx.b_in_hump = false;
            g_ctx.run_peak  = fmaxf(g_ctx.run_peak, g_ctx.hump_max);
            event_post(IMU_EVT_HUMP_END);
            /* Height in 0.1 mm; humps are well under 3 m, so it fits. */
            telemetry_post_event(EV_HUMP, 0U,
                                 (int16_t) round_i(g_ctx.hump_max * TENTHS),
                                 0);
        }
    }
    else
    {
        /* Level ground. */
    }
}

/*!
 * @brief  Tilt-compensated magnetic heading.
 * @param  p_mag  Magnetic field, gauss.
 * @param  pitch  Pitch, radians.
 * @param  roll   Roll, radians.
 * @return Heading, 0..360 degrees.
 */
static float32_t
heading_compute (vec3_t const * p_mag, float32_t pitch, float32_t roll)
{
    float32_t const cos_p  = cosf(pitch);
    float32_t const sin_p  = sinf(pitch);
    float32_t const cos_r  = cosf(roll);
    float32_t const sin_r  = sinf(roll);
    float32_t const horiz_x  = (p_mag->axis[AXIS_X] * cos_p) +
                          (p_mag->axis[AXIS_Z] * sin_p);
    float32_t const horiz_y  = (p_mag->axis[AXIS_X] * sin_r * sin_p) +
                          (p_mag->axis[AXIS_Y] * cos_r) -
                          (p_mag->axis[AXIS_Z] * sin_r * cos_p);
    float32_t       hdg = atan2f(-horiz_y, horiz_x) * RAD2DEG;

    if (hdg < 0.0f)
    {
        hdg += FULL_TURN_DEG;
    }

    return (hdg);
}

/*!
 * @brief  Magnetometer calibration: record min / max while requested,
 *         then set the hard-iron offsets of X and Y (a flat spin does not
 *         turn the Z axis, so its offset is left unchanged).
 * @param  p_mag  Field with the current offsets removed, gauss.
 */
static void
mag_cal_step (vec3_t const * p_mag)
{
    uint32_t idx = 0U;

    if (g_b_cal_start_req)
    {
        g_b_cal_start_req  = false;
        g_ctx.b_cal_active = true;
        for (idx = 0U; idx < AXES; idx++)
        {
            g_ctx.cal_min.axis[idx] = BIG;
            g_ctx.cal_max.axis[idx] = -BIG;
        }
    }

    if (g_ctx.b_cal_active)
    {
        for (idx = 0U; idx < AXES; idx++)
        {
            /* Record the raw field: add back the current offset. */
            float32_t const raw = p_mag->axis[idx] + g_mag_off.axis[idx];

            g_ctx.cal_min.axis[idx] = fminf(g_ctx.cal_min.axis[idx], raw);
            g_ctx.cal_max.axis[idx] = fmaxf(g_ctx.cal_max.axis[idx], raw);
        }
    }

    if (g_b_cal_finish_req)
    {
        float32_t const span_x = g_ctx.cal_max.axis[AXIS_X] -
                                 g_ctx.cal_min.axis[AXIS_X];
        float32_t const span_y = g_ctx.cal_max.axis[AXIS_Y] -
                                 g_ctx.cal_min.axis[AXIS_Y];
        bool const      b_ok   = g_ctx.b_cal_active &&
                                 (span_x >= MAG_CAL_MIN_SPAN_GS) &&
                                 (span_y >= MAG_CAL_MIN_SPAN_GS);

        g_b_cal_finish_req = false;
        g_ctx.b_cal_active = false;

        if (b_ok)
        {
            g_mag_off.axis[AXIS_X] = (g_ctx.cal_min.axis[AXIS_X] +
                                      g_ctx.cal_max.axis[AXIS_X]) / HALVES;
            g_mag_off.axis[AXIS_Y] = (g_ctx.cal_min.axis[AXIS_Y] +
                                      g_ctx.cal_max.axis[AXIS_Y]) / HALVES;
            g_b_mag_calibrated     = true;
        }

        /* Result code 0 or 1 fits a uint8_t. */
        telemetry_post_event(EV_MAG_CAL,
                             (uint8_t) (b_ok ? 0U : MAG_CAL_FAILED), 0, 0);
    }
}

/*!
 * @brief  Queue a terrain event for the Navigation FSM (1 byte).
 * @param  event  Event.
 */
static void
event_post (imu_terrain_event_t event)
{
    /* The event values are 0 and 1. */
    uint8_t const msg = (uint8_t) event;

    (void) tk_snd_mbf(g_evt_q, &msg, (INT) sizeof(msg), TMO_POL);
}

/*!
 * @brief  Round to the nearest integer; 0 for NaN or infinity (Barr-C
 *         5.4.b.v), limited to +/-1e6.
 * @return Rounded value.
 */
static int32_t
round_i (float32_t value)
{
    int32_t result = 0;

    if (isfinite(value))
    {
        /* Clamped to +/-1e6 first, so lroundf() fits an int32_t. */
        result = (int32_t) lroundf(fmaxf(fminf(value, BIG), -BIG));
    }

    return (result);
}

/*** end of file ***/
