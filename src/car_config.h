/**
 * @file    car_config.h
 * @brief   Group 12 robot car - pin map, timing and tuning constants.
 *
 * Every number here traces to the Week 6 Design Review. Values marked
 * TUNE are placeholders that must be measured on the real robot
 * (Assumptions A2-A5, A9).
 */
#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Pin map (Section 2)                                                 */
/* ------------------------------------------------------------------ */
#define PIN_ENC_R_A          (0U)   /* GROVE 1 - right encoder A       */
#define PIN_ENC_R_B          (1U)   /* GROVE 1 - right encoder B       */
#define PIN_IR_R             (3U)   /* GROVE 2 - right line sensor D0
                                       (2nd signal wire; GP2 read
                                       constant white, so D0 is on GP3) */
#define PIN_US_TRIG          (4U)   /* GROVE 3 - HC-SR04 Trig          */
#define PIN_US_ECHO          (5U)   /* GROVE 3 - HC-SR04 Echo          */
#define PIN_IR_BC            (6U)   /* GROVE 5 - barcode sensor D0     */
#define PIN_ENC_L_A          (7U)   /* GROVE 7 - left encoder A        */
#define PIN_M1A              (8U)   /* Left motor  (PWM4 A)            */
#define PIN_M1B              (9U)   /* Left motor  (PWM4 B)            */
#define PIN_M2A              (10U)  /* Right motor (PWM5 A)            */
#define PIN_M2B              (11U)  /* Right motor (PWM5 B)            */
#define PIN_SERVO            (15U)  /* Servo port 4 (PWM7 B)           */
#define PIN_IMU_SDA          (16U)  /* GROVE 4 - I2C0 SDA              */
#define PIN_IMU_SCL          (17U)  /* GROVE 4 - I2C0 SCL              */
#define PIN_NEOPIXEL         (18U)  /* On-board WS2812                 */
#define PIN_BTN_START        (20U)  /* On-board button, active low     */
#define PIN_BTN_CAL          (21U)  /* On-board button, active low     */
#define PIN_IR_L             (27U)  /* GROVE 6 - left line sensor D0   */
#define PIN_ENC_L_B          (28U)  /* GROVE 7 - left encoder B        */
/* GP2 and GP26 are deliberately left unconnected (shared Grove pins). */

/* LM393 D0 level that means "black" (TCRT5000 modules output HIGH when
 * no IR is reflected). TUNE: flip if your modules are inverted.       */
#define IR_BLACK_LEVEL       (1U)

/* ------------------------------------------------------------------ */
/* Task periods (Section 3)                                            */
/* ------------------------------------------------------------------ */
#define PERIOD_LINE_MS       (5U)
#define PERIOD_MOTION_MS     (10U)
#define PERIOD_IMU_MS        (20U)
#define PERIOD_US_MS         (100U)
#define PERIOD_NAV_POLL_MS   (10U)
#define PERIOD_COMMS_MS      (10U)
#define PERIOD_TELEM_MS      (100U)
#define PERIOD_SAFETY_MS     (50U)

/* micro T-Kernel priorities, 1 = highest (Section 3) */
#define PRI_SAFETY           (1)
#define PRI_LINE             (2)
#define PRI_MOTION           (3)
#define PRI_NAV              (4)
#define PRI_IMU              (5)
#define PRI_US               (6)
#define PRI_COMMS            (7)
#define PRI_TELEM            (8)

/* ------------------------------------------------------------------ */
/* Motors (NFR8)                                                       */
/* ------------------------------------------------------------------ */
#define MOTOR_PWM_HZ         (20000U)
/* Motor test 19:52 (wheels in the air):
 *   left : GP9 -> forward (enc -3957), GP8 -> backward (enc +3905)
 *   right: GP11 -> forward (enc +3875), GP10 -> backward (enc -4026)
 * FWD_ON_A = 0 means "forward = PWM on the B pin (GP9 / GP11)".        */
#define LEFT_FWD_ON_A        (0)
#define RIGHT_FWD_ON_A       (0)

/* ------------------------------------------------------------------ */
/* Encoders and geometry - TUNE (Assumption A2)                        */
/* ------------------------------------------------------------------ */
#define ENC_COUNTS_PER_REV   (1440.0f) /* x4 counts per wheel rev       */
#define WHEEL_DIAMETER_M     (0.065f)
#define TRACK_WIDTH_M        (0.125f)  /* wheel centre-to-centre        */
/* Initial guesses only: the power-on self-test measures the real count
 * direction of each encoder (MCMD_ENC_CAL) and overrides these.
 * Motor test 19:52: going forward, left counts down, right counts up.  */
#define ENC_SIGN_LEFT        (-1)
#define ENC_SIGN_RIGHT       (1)
#define ENC_CAL_DUTY         (0.40f)   /* open-loop drive during the test */
#define ENC_CAL_COUNTS       (120)     /* counts per wheel (~5 cm) to decide */
#define ENC_CAL_TIMEOUT_MS   (1500U)
#define ENC_CAL_SPIN_DEG     (25.0f)   /* heading change = a motor is reversed */
#define MOTOR_TEST_DUTY      (0.40f)   /* bench motor test, per wheel     */
#define MOTOR_TEST_MS        (1300U)   /* 0.3 s settle + 1 s drive        */
#define STALL_TIMEOUT_MS     (300U)
#define STALL_MIN_DUTY       (0.25f)   /* duty above which a stall counts */

/* Speed PID, per wheel (duty units per m/s) - TUNE */
#define PID_KP               (1.20f)
#define PID_KI               (6.00f)
#define PID_KD               (0.00f)
#define PID_KFF              (2.20f)   /* feed-forward duty per m/s     */

/* Overall speed scale: every driving speed below (and the steering
 * differential) is multiplied by this. 1.0 = original speeds.      */
#define SPEED_SCALE          (0.75f)

#define SPEED_CRUISE_MPS     (0.30f * SPEED_SCALE)   /* 22.5 cm/s     */
#define SPEED_HUMP_MPS       (0.15f * SPEED_SCALE)
#define SPEED_SEARCH_MPS     (0.12f * SPEED_SCALE)
#define SPEED_TURN_MPS       (0.15f * SPEED_SCALE)

#define K_LINE_STEER         (0.10f * SPEED_SCALE)   /* m/s differential per line step */
/* +1: steer TOWARD the sensor that sees black (left black -> steer
 *     left, right black -> steer right).
 * -1: steer AWAY from it.                                            */
#define LINE_STEER_DIR       (1)
/* Forward speed while a line sensor sees black (correcting). Inner
 * wheel runs at this minus K_LINE_STEER, outer wheel plus it.        */
#define SPEED_STEER_MPS      (0.18f * SPEED_SCALE)
#define K_DIST_EQ            (2.00f)   /* m/s per m of wheel mismatch    */
#define K_HEADING            (0.004f)  /* m/s per degree heading error   */
#define TURN_EARLY_FRACTION  (0.60f)   /* new branch accepted after 60 % */
#define TURN_XCHECK_DEG      (20.0f)   /* encoder vs IMU disagreement    */

/* ------------------------------------------------------------------ */
/* Line / barcode (Section 6.1)                                        */
/* ------------------------------------------------------------------ */
#define BAR_DEBOUNCE_US      (500U)
#define BAR_EDGES_FULL       (30U)
#define BAR_IDLE_TIMEOUT_US  (150000U)
#define BAR_EDGE_BUF_LEN     (64U)
#define CAL_WINDOW_MS        (5000U)

/* ------------------------------------------------------------------ */
/* IMU (Section 3, A4)                                                 */
/* ------------------------------------------------------------------ */
#define IMU_I2C_HZ           (400000U)
#define IMU_ACC_ADDR         (0x19U)
#define IMU_MAG_ADDR         (0x1EU)
#define IMU_LPF_ALPHA        (0.20f)
#define HUMP_ON_DEG          (4.0f)
#define HUMP_OFF_DEG         (1.5f)
#define IMPACT_G             (1.50f)   /* horizontal jerk -> collision  */
#define OVERTILT_DEG         (35.0f)

/* ------------------------------------------------------------------ */
/* Ultrasonic + servo (Section 6.3)                                    */
/* ------------------------------------------------------------------ */
#define US_OBSTACLE_CM       (12.0f)
#define US_SCAN_RANGE_CM     (40.0f)
#define US_ECHO_TIMEOUT_US   (30000U)
#define US_MIN_GAP_MS        (60U)
#define SERVO_SETTLE_MS      (120U)
#define SERVO_CENTRE_DEG     (90)
#define SERVO_FINE_STEP_DEG  (5)
#define SERVO_FINE_SPAN_DEG  (15)
#define ROBOT_WIDTH_CM       (15.0f)   /* TUNE                           */
#define CLEAR_MARGIN_CM      (5.0f)
#define BYPASS_OFFSET_M      (0.25f)
#define BYPASS_LENGTH_M      (0.35f)
#define REACQUIRE_TIMEOUT_MS (3000U)

/* ------------------------------------------------------------------ */
/* Safety (Section 3)                                                  */
/* ------------------------------------------------------------------ */
#define SAFETY_HB_TIMEOUT_MS (300U)

/* ------------------------------------------------------------------ */
/* WiFi / MQTT (Section 1, A1) - edit before building                  */
/* ------------------------------------------------------------------ */
/* WiFi / MQTT - set in wifi_secrets.h, which CMake generates from     */
/* -DWIFI_SSID=... -DWIFI_PASSWORD=... -DMQTT_BROKER_IP=... (the same  */
/* file the template's radio task reads). Defaults only as a fallback. */
/* ------------------------------------------------------------------ */
#include "wifi_secrets.h"
#ifndef WIFI_SSID
#define WIFI_SSID            "Kuro"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD        "change-me"
#endif
#ifndef MQTT_BROKER_IP
#define MQTT_BROKER_IP       "172.20.10.6"    /* laptop on hotspot Kuro (re-check with ipconfig) */
#endif
#ifndef MQTT_BROKER_PORT
#define MQTT_BROKER_PORT     (1883U)
#endif
#ifndef MQTT_CLIENT_ID
#define MQTT_CLIENT_ID       "robocar-g12"
#endif
#define MQTT_TOPIC_ROOT      "robot/12"
/* MQTT_PASSWORD (in wifi_secrets.h) must match the broker's --password;
 * leave it undefined for an open broker.                              */
#ifndef MQTT_USER
#define MQTT_USER            "car"
#endif
#define MQTT_BACKOFF_MAX_MS  (4000U)
#define MQTT_OFFLINE_BUF     (10U)     /* 1 s of telemetry at 100 ms    */

#endif /* CAR_CONFIG_H */
