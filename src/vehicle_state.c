/** @file    vehicle_state.c
 *  @brief   Shared vehicle state record, stored packed (see vehicle_state.h).
 *
 *  NOTE: (void) casts mark return values that are deliberately ignored.
 */
#include <string.h>
#include <tk/tkernel.h>

#include "bitpack.h"
#include "vehicle_state.h"

/** Field numbers; the order matches g_layout. */
typedef enum
{
    VS_SPEED_L = 0,
    VS_SPEED_R,
    VS_DUTY_L,
    VS_DUTY_R,
    VS_DIST,
    VS_PITCH,
    VS_ROLL,
    VS_HEADING,
    VS_RATE,
    VS_HUMP,
    VS_CLASS,
    VS_IMU_VALID,
    VS_MAG_CAL,
    VS_IMU_STAMP,
    VS_IR,
    VS_BARCODE,
    VS_NAV,
    VS_FIELD_COUNT
} vs_field_t;

#define PACKED_BYTES         (17U)        /* 132 bits, rounded up        */

/** Packed layout: width and signedness of each field. */
static bitpack_field_t const g_layout[VS_FIELD_COUNT] =
{
    { 12U, true  },   /* speed_l_mm/s: +/-2047                           */
    { 12U, true  },   /* speed_r_mm/s                                    */
    { 8U,  true  },   /* duty_l_pct: -100..100                           */
    { 8U,  true  },   /* duty_r_pct                                      */
    { 18U, true  },   /* dist_mm: +/-131 m                               */
    { 9U,  true  },   /* pitch, 0.5 deg: +/-128 deg                      */
    { 9U,  true  },   /* roll, 1 deg: +/-256 deg (+/-180 used)           */
    { 10U, false },   /* heading, 0.5 deg: 0..719                        */
    { 9U,  true  },   /* turn rate deg/s: +/-255                         */
    { 8U,  false },   /* hump peak, 0.5 mm: 0..127.5 mm                  */
    { 2U,  false },   /* motion class 0..3                               */
    { 1U,  false },   /* IMU valid                                       */
    { 1U,  false },   /* magnetometer calibrated                         */
    { 16U, false },   /* IMU time stamp, ms modulo 65536                 */
    { 2U,  false },   /* raw line sensors                                */
    { 3U,  false },   /* last barcode: slot 0..3 + valid                 */
    { 4U,  false }    /* nav state                                       */
};

static ID      g_mtx = 0;
static uint8_t g_packed[PACKED_BYTES];

static void field_put (vs_field_t field, int32_t value);
static int32_t field_get (uint8_t const * p_buf, vs_field_t field);
static void lock (void);
static void unlock (void);

/*!
 * @brief  Create the mutex and clear the record. Call from usermain()
 *         before any task starts.
 */
void
vehicle_state_init (void)
{
    T_CMTX create;

    (void) memset(&create, 0, sizeof(create));
    (void) memset(g_packed, 0, sizeof(g_packed));
    create.mtxatr = TA_INHERIT;                    /* priority inheritance */
    g_mtx         = tk_cre_mtx(&create);
}

/*!
 * @brief  Read a consistent snapshot of the whole record.
 * @param  p_out  Unpacked values.
 */
void
vehicle_state_read (vehicle_state_t * p_out)
{
    uint8_t copy[PACKED_BYTES];

    /* Copy under the lock, unpack outside it: the lock stays short. */
    lock();
    (void) memcpy(copy, g_packed, sizeof(copy));
    unlock();

    /* Each value fits its target type because of its packed width. */
    p_out->speed_l_mms       = (int16_t) field_get(copy, VS_SPEED_L);
    p_out->speed_r_mms       = (int16_t) field_get(copy, VS_SPEED_R);
    p_out->duty_l_pct        = (int8_t) field_get(copy, VS_DUTY_L);
    p_out->duty_r_pct        = (int8_t) field_get(copy, VS_DUTY_R);
    p_out->dist_mm           = field_get(copy, VS_DIST);
    p_out->pitch_half_deg    = (int16_t) field_get(copy, VS_PITCH);
    p_out->roll_deg          = (int16_t) field_get(copy, VS_ROLL);
    p_out->heading_half_deg  = (uint16_t) field_get(copy, VS_HEADING);
    p_out->turn_rate_dps     = (int16_t) field_get(copy, VS_RATE);
    p_out->hump_peak_half_mm = (uint16_t) field_get(copy, VS_HUMP);
    p_out->motion_class      = (uint8_t) field_get(copy, VS_CLASS);
    p_out->b_imu_valid       = (0 != field_get(copy, VS_IMU_VALID));
    p_out->b_mag_calibrated  = (0 != field_get(copy, VS_MAG_CAL));
    p_out->imu_stamp         = (uint16_t) field_get(copy, VS_IMU_STAMP);
    p_out->ir_black          = (uint8_t) field_get(copy, VS_IR);
    p_out->last_barcode      = (uint8_t) field_get(copy, VS_BARCODE);
    p_out->nav_state         = (uint8_t) field_get(copy, VS_NAV);
}

/*!
 * @brief  Motion Control's fields. Out-of-range values are clamped.
 * @param  speed_l_mms  Left wheel speed, mm/s.
 * @param  speed_r_mms  Right wheel speed, mm/s.
 * @param  duty_l_pct   Left motor duty, percent.
 * @param  duty_r_pct   Right motor duty, percent.
 * @param  dist_mm      Mean distance travelled since boot, mm.
 */
void
vehicle_state_set_motion (int32_t speed_l_mms, int32_t speed_r_mms,
                          int32_t duty_l_pct, int32_t duty_r_pct,
                          int32_t dist_mm)
{
    lock();
    field_put(VS_SPEED_L, speed_l_mms);
    field_put(VS_SPEED_R, speed_r_mms);
    field_put(VS_DUTY_L, duty_l_pct);
    field_put(VS_DUTY_R, duty_r_pct);
    field_put(VS_DIST, dist_mm);
    unlock();
}

/*!
 * @brief  IMU Terrain's fields (taken from the matching members of p_imu;
 *         the other members are ignored).
 * @param  p_imu  New IMU values.
 */
void
vehicle_state_set_imu (vehicle_state_t const * p_imu)
{
    lock();
    field_put(VS_PITCH, p_imu->pitch_half_deg);
    field_put(VS_ROLL, p_imu->roll_deg);
    field_put(VS_HEADING, p_imu->heading_half_deg);
    field_put(VS_RATE, p_imu->turn_rate_dps);
    field_put(VS_HUMP, p_imu->hump_peak_half_mm);
    field_put(VS_CLASS, p_imu->motion_class);
    field_put(VS_IMU_VALID, p_imu->b_imu_valid ? 1 : 0);
    field_put(VS_MAG_CAL, p_imu->b_mag_calibrated ? 1 : 0);
    field_put(VS_IMU_STAMP, p_imu->imu_stamp);
    unlock();
}

/*!
 * @brief  Line task: raw line sensor bits.
 * @param  ir_black  VEHICLE_IR_* bits.
 */
void
vehicle_state_set_line (uint8_t ir_black)
{
    lock();
    field_put(VS_IR, ir_black);
    unlock();
}

/*!
 * @brief  Line task: last decoded barcode.
 * @param  last_barcode  VEHICLE_BAR_* bits.
 */
void
vehicle_state_set_barcode (uint8_t last_barcode)
{
    lock();
    field_put(VS_BARCODE, last_barcode);
    unlock();
}

/*!
 * @brief  Navigation FSM: system state.
 * @param  nav_state  nav_state_t value.
 */
void
vehicle_state_set_nav (uint8_t nav_state)
{
    lock();
    field_put(VS_NAV, nav_state);
    unlock();
}

/*!
 * @brief  Store one field in the packed record (caller holds the lock).
 * @param  field  Field number.
 * @param  value  Value, clamped to the field's range.
 */
static void
field_put (vs_field_t field, int32_t value)
{
    (void) bitpack_put(g_packed, bitpack_offset(g_layout, field), value,
                       &g_layout[field]);
}

/*!
 * @brief  Read one field from a packed copy.
 * @param  p_buf  Packed copy.
 * @param  field  Field number.
 * @return Field value.
 */
static int32_t
field_get (uint8_t const * p_buf, vs_field_t field)
{
    return (bitpack_get(p_buf, bitpack_offset(g_layout, field),
                        &g_layout[field]));
}

/*!
 * @brief  Take the record mutex (waits; the holders keep it briefly).
 */
static void
lock (void)
{
    (void) tk_loc_mtx(g_mtx, TMO_FEVR);
}

/*!
 * @brief  Release the record mutex.
 */
static void
unlock (void)
{
    (void) tk_unl_mtx(g_mtx);
}

/*** end of file ***/
