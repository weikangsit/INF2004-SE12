/** @file    car_config.h
 *  @brief   Group 12 robot car: pin map, timing and tuning constants.
 *
 *  Every number traces to the Week 6 Design Review or a Tribunal 1
 *  decision. Values marked TUNE are placeholders that must be measured on
 *  the real robot (Assumptions A2-A5, A9).
 */
#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Pin map (Week 6 Section 2)                                           */
/* ------------------------------------------------------------------ */
#define PIN_ENC_R_A          (0U)   /* GROVE 1 - right encoder A       */
#define PIN_ENC_R_B          (1U)   /* GROVE 1 - right encoder B       */
#define PIN_IR_R             (3U)   /* GROVE 2 - right line sensor D0;
                                       GP2 read constant white, so D0
                                       is on the 2nd wire, GP3          */
#define PIN_US_TRIG          (4U)   /* GROVE 3 - HC-SR04 Trig          */
#define PIN_US_ECHO          (5U)   /* GROVE 3 - HC-SR04 Echo          */
#define PIN_IR_BC            (6U)   /* GROVE 5 - barcode sensor D0     */
#define PIN_ENC_L_A          (7U)   /* GROVE 7 - left encoder A        */
#define PIN_M1A              (8U)   /* left motor  (PWM4 A)            */
#define PIN_M1B              (9U)   /* left motor  (PWM4 B)            */
#define PIN_M2A              (10U)  /* right motor (PWM5 A)            */
#define PIN_M2B              (11U)  /* right motor (PWM5 B)            */
#define PIN_SERVO            (15U)  /* servo port 4 (PWM7 B)           */
#define PIN_IMU_SDA          (16U)  /* GROVE 4 - I2C0 SDA              */
#define PIN_IMU_SCL          (17U)  /* GROVE 4 - I2C0 SCL              */
#define PIN_NEOPIXEL         (18U)  /* on-board WS2812                 */
#define PIN_BTN_START        (20U)  /* on-board button, active low     */
#define PIN_BTN_CAL          (21U)  /* on-board button, active low     */
#define PIN_IR_L             (27U)  /* GROVE 6 - left line sensor D0   */
#define PIN_ENC_L_B          (28U)  /* GROVE 7 - left encoder B        */
/* GP2 and GP26 are deliberately left unconnected (shared Grove pins). */

/* LM393 D0 level that means "black" (TCRT5000 modules output HIGH when
 * no IR is reflected). TUNE: flip if your modules are inverted.        */
#define IR_BLACK_LEVEL       (1U)

/* ------------------------------------------------------------------ */
/* Task periods and priorities (Week 6 Section 3, 1 = highest)          */
/* ------------------------------------------------------------------ */
#define PERIOD_LINE_MS       (5U)
#define PERIOD_MOTION_MS     (10U)
#define PERIOD_IMU_MS        (20U)
#define PERIOD_US_MS         (100U)
#define PERIOD_NAV_POLL_MS   (10U)   /* bounded wait between input polls */
#define PERIOD_TELEM_MS      (100U)
#define PERIOD_SAFETY_MS     (50U)   /* longest wait for a safety event  */

#define PRI_SAFETY           (1)
#define PRI_LINE             (2)
#define PRI_MOTION           (3)
#define PRI_NAV              (4)
#define PRI_IMU              (5)
#define PRI_US               (6)
#define PRI_TELEM            (8)     /* Comms (7) is the template task   */

/* ------------------------------------------------------------------ */
/* Motors (NFR8)                                                        */
/* ------------------------------------------------------------------ */
#define MOTOR_PWM_HZ         (20000U)
/* Motor test (wheels in the air): left GP9 forward, right GP11
 * forward. FWD_ON_A = 0 means "forward = PWM on the B pin".            */
#define LEFT_FWD_ON_A        (0)
#define RIGHT_FWD_ON_A       (0)

/* ------------------------------------------------------------------ */
/* Encoders, geometry and control (Assumption A2) - TUNE                */
/* ------------------------------------------------------------------ */
#define ENC_COUNTS_PER_REV   (1440.0f) /* x4 counts per wheel rev       */
#define WHEEL_DIAMETER_M     (0.065f)
#define TRACK_WIDTH_M        (0.125f)  /* wheel centre-to-centre        */
/* Initial guesses; the power-on self-test measures the real count
 * direction of each encoder and overrides them.                        */
#define ENC_SIGN_LEFT        (-1)
#define ENC_SIGN_RIGHT       (1)
#define ENC_CAL_DUTY_PM      (400)     /* self-test push, per mille     */
#define ENC_CAL_COUNTS       (120)     /* counts per wheel (~5 cm)      */
#define ENC_CAL_TIMEOUT_MS   (1500U)
#define ENC_CAL_SPIN_DEG     (25.0f)   /* rotation = a motor reversed   */
#define MOTOR_TEST_DUTY_PM   (400)     /* bench motor test, per wheel   */
#define MOTOR_TEST_MS        (1300U)   /* settle + drive per phase      */
#define MOTOR_TEST_SETTLE_MS (300U)
/* Dead-band sweep (dashboard "Dead-band test", wheels in the air): per
 * wheel, raise the duty from SWEEP_START_PM in SWEEP_STEP_PM steps of
 * SWEEP_STEP_MS until the wheel moves SWEEP_MOVE_COUNTS in one step
 * (= MOTOR_DEADBAND), then hold SWEEP_HOLD_PM and measure the speed
 * after a spin-up (gives PID_KFF).                                     */
#define SWEEP_START_PM       (20)      /* 2 %                           */
#define SWEEP_STEP_PM        (10)      /* +1 % per step                 */
#define SWEEP_STEP_MS        (60U)
#define SWEEP_MAX_PM         (800)     /* give up above 80 %            */
#define SWEEP_MOVE_COUNTS    (8)       /* ~1 mm of wheel travel         */
#define SWEEP_HOLD_PM        (600)     /* 60 %                          */
#define SWEEP_SPINUP_MS      (400U)
#define SWEEP_MEASURE_MS     (600U)
#define STALL_TIMEOUT_MS     (300U)    /* no encoder edge while driving */
#define STALL_MIN_DUTY       (0.25f)   /* duty above which a stall counts */

/* Per-wheel speed PID (duty per m/s). Full precision float inside
 * Motion Control only (Tribunal 1, Buddy 2).
 * Feed-forward = MOTOR_DEADBAND + PID_KFF x speed. Motor test: 40 % duty
 * gave ~3957 counts/s = 0.56 m/s, so ~0.7 duty per m/s. The old 2.2 was
 * 3x too high: the PID had to cancel most of it and swung the duty below
 * the motor dead band, so the wheels stopped and lurched (measured).
 * Each wheel has its own dead band and slope: the motors differ, and a
 * weaker one the integrator cannot make up pulls the car to one side
 * (measured: right wheel slower on the floor, car curved right).
 * TUNE: dead band = lowest duty that turns that wheel (motor test);
 * on the floor, raise a wheel's KFF until its steady duty on the
 * dashboard matches the other wheel's.                                 */
#define PID_KP               (1.00f)
#define PID_KI               (1.50f)
#define PID_KD               (0.00f)
#define PID_KFF_L            (0.55f)   /* duty per m/s above dead band  */
#define PID_KFF_R            (0.55f)
#define MOTOR_DEADBAND_L     (0.07f)   /* duty below which it won't turn */
#define MOTOR_DEADBAND_R     (0.08f)
#define PID_I_LIMIT          (0.35f)   /* integral limit, m (x KI = 0.5) */
#define FF_MIN_SPEED_MPS     (0.005f)  /* below this the target is "stop" */
/* Encoder speed is averaged over this many control periods (40 ms):
 * one 10 ms period holds only ~6 counts at 9 cm/s.                     */
#define SPEED_AVG_PERIODS    (4U)
/* Speed set-point ramp: limits the jerk that the IMU would otherwise
 * read as tilt, hump or impact. TUNE.                                  */
#define ACCEL_LIMIT_MPS2     (0.50f)
/* Each wheel's target may change by at most this per second, so
 * straight -> turn, steering corrections and speed caps blend instead
 * of stepping (measured: jerky turns). Braking at the end stays
 * immediate. TUNE.                                                     */
#define WHEEL_ACCEL_MPS2     (1.00f)
/* Runs and turns with a known end slow down along a v^2 = 2 a d curve
 * to this speed before the end, instead of braking at full speed.      */
#define END_MIN_SPEED_MPS    (0.04f)

/* Speeds in cm/s, scaled by SPEED_SCALE_PCT (75 % since the IR sensor
 * commit; Week 6 NFR1 cruise is 30 cm/s, see Tribunal change log).     */
#define SPEED_SCALE_PCT      (75U)
#define SPEED_CRUISE_CMS     ((30U * SPEED_SCALE_PCT) / 100U)  /* 22    */
#define SPEED_HUMP_CMS       ((15U * SPEED_SCALE_PCT) / 100U)  /* 11    */
#define SPEED_SEARCH_CMS     ((12U * SPEED_SCALE_PCT) / 100U)  /* 9     */
#define SPEED_TURN_CMS       ((15U * SPEED_SCALE_PCT) / 100U)  /* 11    */
#define SPEED_STEER_CMS      ((18U * SPEED_SCALE_PCT) / 100U)  /* 13    */
#define MAG_CAL_SPEED_CMS    (8U)      /* slow spin while calibrating   */

/* Line steering: wheel speed differential while one sensor sees black.
 * LINE_STEER_DIR +1 steers toward the black sensor, -1 away from it.   */
#define K_LINE_STEER_MPS     (0.075f)
#define LINE_STEER_DIR       (1)
/* After the last steering correction the car stays at SPEED_STEER for
 * this long before ramping back to cruise, so the speed does not surge
 * 13 <-> 22 cm/s each time the sensor flickers on a curve (measured:
 * jerky curves). TUNE: longer = smoother, slower on S-bends.           */
#define LINE_SLOW_HOLD_MS    (400U)
/* Two straddling sensors read white both when centred and when the line
 * has run out past one of them on a curve. A sensor that saw black for
 * at least LINE_CURVE_MS (a curve, not a nudge) and then went white
 * keeps the car turning that way at LINE_RECOVER_GAIN x the steering
 * for up to LINE_RECOVER_MS, or until a sensor sees black again
 * (measured: line lost on curves, car ran straight). TUNE.             */
#define LINE_CURVE_MS        (100U)
#define LINE_RECOVER_MS      (300U)
#define LINE_RECOVER_GAIN    (0.5f)
#define K_DIST_EQ            (2.00f)   /* m/s per m of wheel mismatch   */
#define K_HEADING            (0.004f)  /* m/s per degree heading error  */
#define TURN_EARLY_FRACTION  (0.60f)   /* new branch accepted after 60 % */
#define TURN_XCHECK_DEG      (20.0f)   /* encoder vs IMU disagreement   */

/* Data older than this is not used (Tribunal 1 interface table).       */
#define LINE_MAX_AGE_MS      (20U)
#define IMU_MAX_AGE_MS       (60U)

/* ------------------------------------------------------------------ */
/* Line / barcode (Week 6 Section 6.1)                                  */
/* ------------------------------------------------------------------ */
#define BAR_DEBOUNCE_US      (500U)
#define BAR_EDGES_FULL       (30U)     /* one '*X*' symbol = 30 edges   */
#define BAR_IDLE_TIMEOUT_US  (150000U)
#define BAR_EDGE_BUF_LEN     (64U)
/* Narrowest wide element / widest narrow element, x10. Code 39 uses
 * 2:1 to 3:1; a ratio under 1.5 is not a clean read (Tribunal 1).      */
#define BAR_MIN_WIDE_RATIO_X10 (15U)
#define CAL_WINDOW_MS        (5000U)
/* Two straddling sensors cannot tell "centred" from "lost": the line
 * counts as lost when neither sensor has seen black for this long
 * while line following (Week 6: 3 s). TUNE on long straights.          */
#define LINE_LOST_MS         (3000U)
/* A sensor that saw black keeps reporting black for this long, so a
 * reading that flickers white at the line edge (measured: missed black
 * on curves, car ran straight off) still steers. Junctions and
 * calibration use the raw reading. 30 ms = 6.6 mm at cruise. TUNE.     */
#define LINE_BLACK_HOLD_MS   (30U)

/* ------------------------------------------------------------------ */
/* IMU (Week 6 Section 3, Assumption A4)                                */
/* ------------------------------------------------------------------ */
#define IMU_I2C_HZ           (400000U)
#define IMU_ACC_ADDR         (0x19U)
#define IMU_MAG_ADDR         (0x1EU)
#define IMU_LPF_ALPHA        (0.20f)
#define HUMP_ON_DEG          (4.0f)
#define HUMP_OFF_DEG         (1.5f)
/* A nose-up pitch only becomes a hump once the car has climbed this
 * high (sin(pitch) x encoder distance). Speeding up also reads as
 * nose-up pitch but adds only 2-4 mm (measured: hump start / cruise
 * feedback loop). TUNE.                                                */
#define HUMP_MIN_MM          (10.0f)
#define IMPACT_G             (1.50f)   /* horizontal jerk -> collision  */
/* A jump only counts if the next reading is still this far from the
 * reading before it (one-sample spikes from a loose mount or I2C noise
 * come straight back). TUNE.                                          */
#define IMPACT_CONFIRM_G     (0.75f)
/* Readings at or beyond this on any axis are saturated (range +/-2 g)
 * or corrupt and are ignored by the impact check.                    */
#define ACC_SAT_G            (1.95f)
#define OVERTILT_DEG         (35.0f)
/* The tilt must last this long: motor jerks are shorter (measured:
 * false over-tilt trips on motor jerks). TUNE.                         */
#define OVERTILT_HOLD_MS     (200U)
#define MAG_CAL_SPIN_DEG     (360U)    /* one slow turn on GP21 hold    */
#define MAG_CAL_MIN_SPAN_GS  (0.10f)   /* each axis must change by this */

/* ------------------------------------------------------------------ */
/* Ultrasonic, servo and obstacle handling (Week 6 Section 6.3)         */
/* ------------------------------------------------------------------ */
#define US_OBSTACLE_CM       (12U)
#define US_SCAN_RANGE_CM     (40U)     /* hits beyond this are clear    */
#define US_ECHO_TIMEOUT_US   (30000U)
#define US_MIN_GAP_MS        (60U)
#define SERVO_SETTLE_MS      (120U)
#define SERVO_CENTRE_DEG     (90U)
/* Sweep limits. Requirement: 15..165 deg. Interim (Tribunal 1, servo
 * travel limited until the mount is raised): 60..120 deg after a
 * reverse of REVERSE_BEFORE_SCAN_CM. Set 15 / 165 and 0 cm once fixed. */
#define SCAN_MIN_DEG         (60U)
#define SCAN_MAX_DEG         (120U)
#define SCAN_COARSE_RAYS     (5U)
#define SERVO_FINE_STEP_DEG  (5U)
#define SERVO_FINE_SPAN_DEG  (15U)
#define ROBOT_WIDTH_CM       (15U)     /* TUNE                          */
#define CLEAR_MARGIN_CM      (5U)
#define MIN_SIDE_CLEAR_CM    (30U)
/* Reverse before scanning, only along the car's own straight track
 * (Tribunal 1 Version C): needs this distance + margin driven straight
 * on the line since the last turn. 0 disables the reverse.             */
#define REVERSE_BEFORE_SCAN_CM (40U)
#define REVERSE_MARGIN_CM    (4U)      /* +/-4 cm reverse tolerance     */
#define REVERSE_RETRY_CM     (10U)
#define BYPASS_TURN_DEG      (45U)
#define BYPASS_OFFSET_CM     (25U)
#define BYPASS_LENGTH_CM     (35U)
#define REACQUIRE_TIMEOUT_MS (3000U)

/* ------------------------------------------------------------------ */
/* Navigation                                                           */
/* ------------------------------------------------------------------ */
#define INIT_TIMEOUT_MS      (15000U)
#define BARCODE_TURN_DEG     (90U)
#define BARCODE_STRAIGHT_CM  (10U)
#define UTURN_DEG            (180U)
#define BTN_LONG_PRESS_MS    (2000U)   /* GP21 hold in READY: mag cal   */

/* ------------------------------------------------------------------ */
/* Safety: per-task heartbeat deadlines (Week 6 feedback)               */
/* ------------------------------------------------------------------ */
#define SAFETY_BOOT_GRACE_MS (2000U)
#define HB_DEADLINE_LINE_MS  (100U)    /* period 5 ms                   */
#define HB_DEADLINE_MOTION_MS (100U)   /* period 10 ms                  */
#define HB_DEADLINE_IMU_MS   (200U)    /* period 20 ms + I2C timeouts   */
#define HB_DEADLINE_US_MS    (600U)    /* one scan step: settle + ping  */
#define HB_DEADLINE_NAV_MS   (100U)    /* period 10 ms                  */
#define HB_DEADLINE_TELEM_MS (300U)    /* warning only                  */
#define HB_DEADLINE_COMMS_MS (500U)    /* radio task poll counter       */

/* ------------------------------------------------------------------ */
/* WiFi / MQTT (Week 6 Section 1, A1)                                   */
/* ------------------------------------------------------------------ */
/* Set in wifi_secrets.h, which CMake generates from -DWIFI_SSID=...,
 * -DWIFI_PASSWORD=... and -DMQTT_BROKER_IP=... (the same file the
 * template's radio task reads). Defaults only as a fallback.           */
#include "wifi_secrets.h"
#ifndef WIFI_SSID
# define WIFI_SSID           "Kuro"
#endif
#ifndef WIFI_PASSWORD
# define WIFI_PASSWORD       "change-me"
#endif
#ifndef MQTT_BROKER_IP
# define MQTT_BROKER_IP      "172.20.10.6"   /* laptop on hotspot Kuro   */
#endif
#ifndef MQTT_BROKER_PORT
# define MQTT_BROKER_PORT    (1883U)
#endif
#ifndef MQTT_CLIENT_ID
# define MQTT_CLIENT_ID      "robocar-g12"
#endif
#define MQTT_TOPIC_ROOT      "robot/12"
/* MQTT_PASSWORD (in wifi_secrets.h) must match the broker's --password;
 * leave it undefined for an open broker.                               */
#ifndef MQTT_USER
# define MQTT_USER           "car"
#endif
#define MQTT_BACKOFF_MAX_MS  (4000U)
#define MQTT_OFFLINE_BUF     (10U)     /* 1 s of telemetry at 100 ms    */

#endif /* CAR_CONFIG_H */

/*** end of file ***/
