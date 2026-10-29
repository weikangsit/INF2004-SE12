/**
 * @file    imu.c
 * @brief   IMU Terrain Task (Buddy 4) - GY-511 / LSM303DLHC on I2C0.
 *
 *  - pitch / roll from the low-pass-filtered accelerometer
 *  - tilt-compensated magnetometer heading and turn rate (no gyro)
 *  - hump detection, height = sum(sin(pitch) * encoder distance)
 *  - impact (collision) and over-tilt detection
 */
#include <math.h>

#include "port.h"
#include "i2c_drv.h"

#include "app.h"
#include "imu.h"

#define RAD2DEG         (57.2957795f)
#define DEG2RAD         (0.01745329f)

/* LSM303DLHC registers */
#define ACC_CTRL_REG1   (0x20U)
#define ACC_CTRL_REG4   (0x23U)
#define ACC_OUT_X_L     (0x28U)
#define MAG_CRA_REG     (0x00U)
#define MAG_CRB_REG     (0x01U)
#define MAG_MR_REG      (0x02U)
#define MAG_OUT_X_H     (0x03U)
#define AUTO_INC        (0x80U)

#define ACC_LSB_PER_G   (16384.0f)   /* +/-2 g, 16-bit left-justified  */
#define MAG_LSB_XY      (1100.0f)    /* +/-1.3 gauss                   */
#define MAG_LSB_Z       (980.0f)

#define I2C_TIMEOUT_US  (2000U)
#define I2C_FAIL_LIMIT  (5U)

/* Hard-iron offsets - TUNE with imu_calibrate_mag() (Assumption A4) */
static float g_mag_off_x;
static float g_mag_off_y;
static float g_mag_off_z;
static volatile bool g_ok;

bool imu_is_ok(void) { return g_ok; }

static bool wr(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return i2c_drv_write(addr, b, 2U, false, I2C_TIMEOUT_US);
}

static bool rd(uint8_t addr, uint8_t reg, uint8_t *p_buf, size_t n)
{
    if (!i2c_drv_write(addr, &reg, 1U, true, I2C_TIMEOUT_US))
    {
        return false;
    }
    return i2c_drv_read(addr, p_buf, n, I2C_TIMEOUT_US * 2U);
}

static bool imu_init(void)
{
    i2c_drv_init(PIN_IMU_SDA, PIN_IMU_SCL, IMU_I2C_HZ);

    return wr(IMU_ACC_ADDR, ACC_CTRL_REG1, 0x57U) &&  /* 100 Hz, XYZ on  */
           wr(IMU_ACC_ADDR, ACC_CTRL_REG4, 0x08U) &&  /* +/-2 g, HR      */
           wr(IMU_MAG_ADDR, MAG_CRA_REG,   0x18U) &&  /* 75 Hz           */
           wr(IMU_MAG_ADDR, MAG_CRB_REG,   0x20U) &&  /* +/-1.3 gauss    */
           wr(IMU_MAG_ADDR, MAG_MR_REG,    0x00U);    /* continuous      */
}

static bool read_acc(float *p_ax, float *p_ay, float *p_az)
{
    uint8_t b[6];

    if (!rd(IMU_ACC_ADDR, (uint8_t)(ACC_OUT_X_L | AUTO_INC), b, 6U))
    {
        return false;
    }
    *p_ax = (float)(int16_t)((b[1] << 8) | b[0]) / ACC_LSB_PER_G;
    *p_ay = (float)(int16_t)((b[3] << 8) | b[2]) / ACC_LSB_PER_G;
    *p_az = (float)(int16_t)((b[5] << 8) | b[4]) / ACC_LSB_PER_G;
    return true;
}

static bool read_mag(float *p_mx, float *p_my, float *p_mz)
{
    uint8_t b[6];

    /* order is X, Z, Y, high byte first */
    if (!rd(IMU_MAG_ADDR, MAG_OUT_X_H, b, 6U))
    {
        return false;
    }
    *p_mx = ((float)(int16_t)((b[0] << 8) | b[1]) / MAG_LSB_XY) - g_mag_off_x;
    *p_mz = ((float)(int16_t)((b[2] << 8) | b[3]) / MAG_LSB_Z)  - g_mag_off_z;
    *p_my = ((float)(int16_t)((b[4] << 8) | b[5]) / MAG_LSB_XY) - g_mag_off_y;
    return true;
}

/* Spin the robot in place while this runs (called from Nav at INIT
 * when the operator holds GP21). Uses min/max hard-iron estimate.   */
void imu_calibrate_mag(uint32_t duration_ms)
{
    float mn[3] = {  1e6f,  1e6f,  1e6f };
    float mx[3] = { -1e6f, -1e6f, -1e6f };
    uint32_t t0 = now_ms();

    g_mag_off_x = 0.0f;
    g_mag_off_y = 0.0f;
    g_mag_off_z = 0.0f;
    while ((now_ms() - t0) < duration_ms)
    {
        float v[3];
        uint32_t i;

        if (read_mag(&v[0], &v[1], &v[2]))
        {
            for (i = 0U; i < 3U; i++)
            {
                mn[i] = fminf(mn[i], v[i]);
                mx[i] = fmaxf(mx[i], v[i]);
            }
        }
        (void)tk_dly_tsk(20);
    }
    g_mag_off_x = (mn[0] + mx[0]) * 0.5f;
    g_mag_off_y = (mn[1] + mx[1]) * 0.5f;
    g_mag_off_z = (mn[2] + mx[2]) * 0.5f;
}

static void post_evt(motion_evt_id_t id, float value)
{
    motion_evt_t e = { id, value };
    (void)tk_snd_mbf(g_motion_evt_q, &e, (INT)sizeof(e), TMO_POL);
}

void imu_task(INT stacd, void *exinf)
{
    uint64_t next_us = time_us_64();
    float    fax = 0.0f;
    float    fay = 0.0f;
    float    faz = 1.0f;
    float    prev_ax = 0.0f;
    float    prev_ay = 0.0f;
    float    prev_hdg = 0.0f;
    float    rate = 0.0f;
    float    prev_dist = 0.0f;
    bool     in_hump = false;
    float    hump_h = 0.0f;
    float    hump_max = 0.0f;
    float    run_peak = 0.0f;
    uint32_t fails = 0U;

    (void)stacd;
    (void)exinf;

    g_ok = imu_init();

    for (;;)
    {
        float ax;
        float ay;
        float az;
        float mx;
        float my;
        float mz;
        motion_state_t st;

        period_wait(&next_us, PERIOD_IMU_MS);
        heartbeat(HB_IMU);

        if (!read_acc(&ax, &ay, &az) || !read_mag(&mx, &my, &mz))
        {
            fails++;
            if (fails >= I2C_FAIL_LIMIT)
            {
                g_ok = false;            /* Nav treats this as sensor failure */
            }
            continue;
        }
        fails = 0U;
        g_ok = true;

        /* --- impact: sudden horizontal jerk (unfiltered) --- */
        if ((fabsf(ax - prev_ax) > IMPACT_G) || (fabsf(ay - prev_ay) > IMPACT_G))
        {
            (void)tk_set_flg(g_evt_flg, EVT_COLLISION);
            post_evt(MEVT_IMPACT, 0.0f);
        }
        prev_ax = ax;
        prev_ay = ay;

        /* --- low-pass and tilt --- */
        fax += IMU_LPF_ALPHA * (ax - fax);
        fay += IMU_LPF_ALPHA * (ay - fay);
        faz += IMU_LPF_ALPHA * (az - faz);
        {
            /* X forward, Y left, Z up (mount GY-511 to match) */
            float pitch = atan2f(fax, sqrtf((fay * fay) + (faz * faz)));  /* nose-up + */
            float roll  = atan2f(fay, faz);
            float cp = cosf(pitch);
            float sp = sinf(pitch);
            float cr = cosf(roll);
            float sr = sinf(roll);
            float xh = (mx * cp) + (mz * sp);
            float yh = (mx * sr * sp) + (my * cr) - (mz * sr * cp);
            float hdg = atan2f(-yh, xh) * RAD2DEG;
            float dh;
            float pitch_deg = pitch * RAD2DEG;
            float roll_deg  = roll * RAD2DEG;
            float ddist;
            uint8_t cls;

            if (hdg < 0.0f)
            {
                hdg += 360.0f;
            }
            dh = hdg - prev_hdg;
            if (dh > 180.0f)   { dh -= 360.0f; }
            if (dh < -180.0f)  { dh += 360.0f; }
            rate += 0.3f * ((dh / ((float)PERIOD_IMU_MS / 1000.0f)) - rate);
            prev_hdg = hdg;

            if ((fabsf(pitch_deg) > OVERTILT_DEG) || (fabsf(roll_deg) > OVERTILT_DEG))
            {
                (void)tk_set_flg(g_evt_flg, EVT_OVERTILT);
            }

            /* --- hump: integrate sin(pitch) over encoder distance --- */
            state_read(&st);
            ddist = st.dist_m - prev_dist;
            prev_dist = st.dist_m;

            if (!in_hump && (pitch_deg > HUMP_ON_DEG))
            {
                in_hump = true;
                hump_h = 0.0f;
                hump_max = 0.0f;
                post_evt(MEVT_HUMP_START, 0.0f);
            }
            if (in_hump)
            {
                hump_h += sinf(pitch) * ddist * 1000.0f;       /* mm */
                hump_max = fmaxf(hump_max, hump_h);
                if ((fabsf(pitch_deg) < HUMP_OFF_DEG) && (hump_h < (0.2f * hump_max)))
                {
                    in_hump = false;
                    run_peak = fmaxf(run_peak, hump_max);
                    post_evt(MEVT_HUMP_END, hump_max);
                    telem_event(EV_HUMP, 0U, (int16_t)(hump_max * 10.0f), 0);
                }
            }

            /* --- motion class --- */
            if (in_hump)
            {
                cls = 3U;
            }
            else if (fabsf(rate) > 30.0f)
            {
                cls = 2U;
            }
            else if ((fabsf(st.speed_l_mps) + fabsf(st.speed_r_mps)) > 0.02f)
            {
                cls = 1U;
            }
            else
            {
                cls = 0U;
            }

            state_lock();
            state_raw()->pitch_deg = pitch_deg;
            state_raw()->roll_deg = roll_deg;
            state_raw()->heading_deg = hdg;
            state_raw()->turn_rate_dps = rate;
            state_raw()->hump_peak_mm = fmaxf(run_peak, in_hump ? hump_max : 0.0f);
            state_raw()->motion_class = cls;
            state_unlock();
        }
    }
}
