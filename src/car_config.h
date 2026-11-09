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

/* Units shared by the modules (US_PER_MS is in systime.h).            */
#define PI_F                 (3.14159265f)
#define MM_PER_M             (1000.0f)
#define CM_PER_M             (100.0f)
#define MM_PER_CM            (10)

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
#define PIN_IR_BC            (26U)  /* barcode sensor D0 (wired to GP26;
                                       GP6 stayed constant white)       */
#define PIN_IR_L             (27U)  /* GROVE 6 - left line sensor D0   */
#define PIN_ENC_L_B          (28U)  /* GROVE 7 - left encoder B        */
/* GP2 and GP6 are deliberately left unconnected (shared Grove pins). */

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
#define TRACK_WIDTH_M        (0.110f)  /* wheel centre-to-centre, measured */
/* Track width for spot turns only: the tyres scrub, so the encoders need
 * more travel per degree than TRACK_WIDTH_M gives. TUNE on the track:
 * dashboard "Calibrate compass" in READY ends with one checked 90 deg
 * turn ("turn check: encoder 90 vs IMU x"); scale this by 90 / x.      */
#define TURN_TRACK_WIDTH_M   (0.179f)  /* 0.165 x 90 / 83 (turn check) */
/* Initial guesses; the power-on self-test measures the real count
 * direction of each encoder and overrides them.                        */
#define ENC_SIGN_LEFT        (-1)
#define ENC_SIGN_RIGHT       (1)
#define ENC_CAL_DUTY_PM      (400)     /* self-test push, per mille     */
#define ENC_CAL_COUNTS       (120)     /* counts per wheel (~1.7 cm)    */
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

/* Per-wheel speed PID (duty per m/s), float inside Motion Control only
 * (Tribunal 1, Buddy 2). Feed-forward = MOTOR_DEADBAND + PID_KFF x speed
 * (motor test: 40 % duty = 0.56 m/s). Each wheel has its own dead band
 * and slope: the motors differ. TUNE with the dashboard dead-band test;
 * on the floor, raise a wheel's KFF until both steady duties match.    */
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
/* Slower while steering: the same differential turns tighter. Simulated
 * on circles of 8-30 cm radius: 13 cm/s lost circles under ~12 cm,
 * 8 cm/s held all of them and wandered less on straights.              */
#define SPEED_STEER_CMS      ((11U * SPEED_SCALE_PCT) / 100U)  /* 8     */

/* Line steering: wheel speed differential while one sensor sees black.
 * LINE_STEER_DIR +1 steers toward the black sensor, -1 away from it.   */
#define K_LINE_STEER_MPS     (0.09f)   /* inner wheel ~0 at 8 cm/s */
#define LINE_STEER_DIR       (1)
/* Both white is centred, or the line slipped past the inner sensor on a
 * curve: keep turning toward the side that last saw black at this
 * fraction of K_LINE_STEER_MPS (simulated: 0 loses circles, 1.0 loses
 * gentle curves). TUNE.                                                */
#define LINE_LAST_GAIN       (0.25f)
/* After the last steering correction the car stays at SPEED_STEER for
 * this long before ramping back to cruise, so the speed does not surge
 * 13 <-> 22 cm/s each time the sensor flickers on a curve (measured:
 * jerky curves). TUNE: longer = smoother, slower on S-bends.           */
#define LINE_SLOW_HOLD_MS    (400U)
#define K_DIST_EQ            (2.00f)   /* m/s per m of wheel mismatch   */
/* Straight runs without line following: the heading / distance hold
 * may change each wheel by at most this fraction of the speed, so
 * neither wheel stops or reverses (0.5: each keeps >= half). TUNE.     */
#define HOLD_STEER_MAX_FRAC  (0.50f)
#define TURN_EARLY_FRACTION  (0.60f)   /* new branch accepted after 60 % */
#define TURN_XCHECK_DEG      (20.0f)   /* encoder vs IMU disagreement   */

/* Data older than this is not used (Tribunal 1 interface table).       */
#define LINE_MAX_AGE_MS      (20U)
#define IMU_MAX_AGE_MS       (60U)

/* ------------------------------------------------------------------ */
/* Line / barcode (Week 6 Section 6.1)                                  */
/* ------------------------------------------------------------------ */
/* Raw barcode edges are filtered in the task (bar_filter.c): an element
 * shorter than this is LM393 chatter and is removed. Must stay below the
 * narrowest real bar: ~2 mm at SPEED_STEER (8 cm/s) lasts ~25 ms. TUNE:
 * raise toward the narrowest width on debug/barcode if chatter passes. */
#define BAR_GLITCH_US        (300U)
/* Interrupt ring for raw edges, sized for chatter bursts between two
 * 5 ms polls; a read that loses edges to a full ring is rejected.      */
#define BAR_RING_LEN         (128U)
/* Edges that must arrive before the quiet gap can end a read: a '*X*'
 * symbol is 30 edges, so require most of them to reject a lone stray
 * edge starting a bogus capture. The decoder still needs a full 30 to
 * accept anything; this only gates WHEN the capture is cut off.        */
#define BAR_EDGES_MIN        (24U)
/* A read ends after BAR_QUIET_MS of steady white once BAR_EDGES_MIN
 * edges arrived (at 8 cm/s the widest element is under 100 ms).
 * BAR_IDLE_TIMEOUT_US ends a partial read that never gets there. TUNE. */
#define BAR_QUIET_MS         (300U)
#define BAR_IDLE_TIMEOUT_US  (500000U)
/* While the car drives (moved BAR_MIN_TRAVEL_COUNTS since the first
 * edge) the quiet gap is a distance instead, so a slow car does not end
 * the read early. Must exceed the widest element (~21 mm; a symbol is
 * ~245 mm). It ends any read, so a stray one cannot swallow the next
 * barcode. BAR_STALL_TIMEOUT_US ends a read if the car stops on it.
 * TUNE.                                                                */
#define BAR_QUIET_MM         (40.0f)
#define BAR_STALL_TIMEOUT_US (5000000U)
#define BAR_EDGE_BUF_LEN     (64U)     /* clean edges of one read       */
/* Reads are decoded by the distance the sensor travelled (wheel encoders
 * stamped on every edge), so the car's speed may change during a read
 * (measured: line-following wiggle doubled and halved the speed inside
 * one character, and time-based reads failed). Time is the fallback
 * when the car has not moved this far during the read (hand swipe) or
 * runs open loop.                                                       */
#define BAR_MIN_TRAVEL_COUNTS (100)    /* ~14 mm                        */
/* Barcode sensor offset to the LEFT of the car's centreline, mm (right
 * is negative). Corrects the sensor's travel for the car's turning while
 * it follows the line. The barcode lies 20 mm left of an 18 mm line, so
 * a sensor over it is at least 9 + 20 = 29 mm left of the centreline.
 * TUNE: measure the sensor on the car.                                 */
#define BAR_SENSOR_Y_MM      (29.0f)
/* Narrowest wide element / widest narrow element, x10. Code 39 uses
 * 2:1 to 3:1; a ratio under 1.5 is not a clean read (Tribunal 1).      */
#define BAR_MIN_WIDE_RATIO_X10 (15U)
/* 1: send every barcode capture to the web app (topic debug/barcode:
 * element widths and the decoded letter), for checking the reader.
 * 0 for normal runs.                                                   */
#define BAR_LOG              (1)
/* With BAR_LOG, the barcode sensor status (level now, interrupts, edges
 * kept, reads) is sent this often (event BAR_STATUS).                  */
#define BAR_STATUS_MS        (1000U)
#define CAL_WINDOW_MS        (5000U)
/* Two straddling sensors cannot tell "centred" from "lost": the line
 * counts as lost when neither sensor has seen black for this long
 * while line following (Week 6: 3 s). TUNE on long straights.          */
#define LINE_LOST_MS         (3000U)
/* During a read the car stops line steering and drives straight at
 * SPEED_STEER_CMS: steering pivots the barcode sensor (far out in front)
 * over the bars and spoils the read. The hold starts on the first edge
 * outside a bend (BAR_BEND_MAX_DEG) and ends with the read, or after
 * BAR_HOLD_MAX_MM (symbol ~245 mm + BAR_QUIET_MM) so stray edges cannot
 * leave the car unsteered; BAR_HOLD_MAX_MS is a backstop. TUNE.        */
#define BAR_HOLD_MAX_MM      (300.0f)
#define BAR_HOLD_MAX_MS      (5000U)
/* Bend detector for the hold: the car is in a bend when the wheel
 * encoders show it turned more than BAR_BEND_MAX_DEG over the last
 * BAR_BEND_WINDOW_MM. A corner of radius R turns 100 mm / R rad in the
 * window (R 300 mm: 19 deg); a straight only wiggles. TUNE: above a
 * straight's swing, below a corner's (the dashboard yaw is the compass,
 * not this encoder value).                                             */
#define BAR_BEND_MAX_DEG     (15.0f)
#define BAR_BEND_WINDOW_MM   (100.0f)
/* A sensor that saw black keeps reporting black for this long, so a
 * reading that flickers white at the line edge (measured: missed black
 * on curves, car ran straight off) still steers. Junctions and
 * calibration use the raw reading. 30 ms = 6.6 mm at cruise. TUNE.     */
#define LINE_BLACK_HOLD_MS   (30U)
/* Junction = left and right raw black within this window, not only in
 * the same 5 ms sample: the follower steers toward the first sensor to
 * reach the cross line, so the other one can get there a little later.
 * One junction per crossing: both must read white for the window before
 * another is reported. 150 ms = 3.4 cm at cruise. Raise it if junctions
 * are still missed; lower it if the car acts early on a curve. TUNE.  */
#define JUNCTION_WINDOW_MS   (250U)

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
/* Level calibration: the car stands still on flat ground at power-up
 * for this many IMU samples (50 x 20 ms = 1 s).                       */
#define IMU_LEVEL_SAMPLES    (50U)
/* One-sided hump: the ramp is under the right wheel only (2 wheels +
 * front caster), so the car rolls and the right wheel's lift is
 * HUMP_TRACK_MM x tan(roll). TUNE.                                     */
#define HUMP_TRACK_MM        (130.0f)  /* MEASURE: wheel centre to centre */
#define HUMP_ROLL_ON_DEG     (2.5f)    /* roll that starts a hump        */
#define HUMP_ROLL_OFF_DEG    (1.0f)    /* roll that ends it              */
#define HUMP_ROLL_HOLD       (3U)      /* samples in a row (60 ms)       */
#define HUMP_MAX_MS          (5000U)   /* give up on a hump after this   */
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
/* Compass calibration spin (dashboard cmd): more than a full turn, so
 * min / max cover the whole circle even when encoder turns come up
 * short (measured: a 360 deg request stopped short).                   */
#define MAG_CAL_SPIN_DEG     (450U)
#define MAG_CAL_SPEED_CMS    (8U)      /* slow spin while calibrating   */
#define MAG_CAL_MIN_SPAN_GS  (0.10f)   /* each axis must change by this */

/* ------------------------------------------------------------------ */
/* Ultrasonic, servo and obstacle handling (Week 6 Section 6.3)         */
/* ------------------------------------------------------------------ */
#define US_OBSTACLE_CM       (15U)
#define US_SCAN_RANGE_CM     (40U)     /* hits beyond this are clear    */
#define US_ECHO_TIMEOUT_US   (30000U)
#define US_MIN_GAP_MS        (60U)
/* Servo settle = one 20 ms PWM frame + per degree of travel. 2 ms/deg
 * (SG90 datasheet, no load) pinged mid-swing on the car, so 4: a 75 deg
 * jump waits 320 ms, a 5 deg step the 60 ms ping gap; a scan takes
 * about 2 s. TUNE.                                                     */
#define SERVO_SETTLE_BASE_MS (20U)
#define SERVO_MS_PER_DEG     (4U)
#define SERVO_CENTRE_DEG     (90U)
/* Sweep limits (Week 6 requirement; the servo mount is fixed).        */
#define SCAN_MIN_DEG         (15U)
#define SCAN_MAX_DEG         (165U)
#define SCAN_COARSE_RAYS     (5U)
#define SERVO_FINE_STEP_DEG  (5U)
#define SERVO_FINE_SPAN_DEG  (15U)
#define ROBOT_WIDTH_CM       (15U)     /* TUNE                          */
#define CLEAR_MARGIN_CM      (5U)
#define MIN_SIDE_CLEAR_CM    (30U)
/* Rectangular bypass with 90 deg spot turns: the out-leg runs parallel
 * to the obstacle's face (the car stops ~12 cm from it).               */
#define BYPASS_TURN_DEG      (90U)
/* Step-and-check bypass in small steps (team flow; measured car: sensor
 * 7.5 cm ahead of the axle, half width 7.5 cm, front corners 10.6 cm
 * from the axle, so a spot turn swings them 3.1 cm past the side).
 *  width:  turn out, drive BYPASS_STEP_CM, face forward and check ahead;
 *          blocked -> repeat; clear -> one BYPASS_WIDTH_MARGIN_CM step
 *          and check again.
 *  length: drive along (BYPASS_ALONG_FIRST_CM, then BYPASS_ALONG_STEP_CM)
 *          and check sideways after each step, without turning; clear
 *          -> drive BYPASS_PAST_CM, turn toward the line, check ahead;
 *          blocked -> turn back and continue along.
 * Ahead checks ping PEEK_RIGHT_DEG, centre, PEEK_LEFT_DEG (blocked under
 * BYPASS_PEEK_CM); side checks ping BYPASS_SIDE_DEG (under
 * BYPASS_SIDE_CM). TUNE.                                               */
#define BYPASS_STEP_CM       (20U)
#define BYPASS_PEEK_CM       (20U)
/* Bypass side: 1 = left (anti-clockwise runs), 0 = right (clockwise),
 * so the car goes round on the track's inside. The scan still decides
 * whether a bypass is needed and safe; its side choice is not used.   */
#define BYPASS_ON_LEFT       (1)
/* After the first clear width check. At that check the obstacle face is
 * 12-15 cm ahead of the sensor, where a 60 deg ray is 6.9-8.7 cm to the
 * side, so "clear" can leave the edge 0.6 cm inside the half width. 8 cm
 * leaves a side gap of >= 7.4 cm: the turn toward the line clears it
 * (swing 3.1 cm), and afterwards the sensor faces the obstacle at the
 * side gap, 7.4-27.4 cm: reliable range.                              */
#define BYPASS_WIDTH_MARGIN_CM (8U)
/* First drive along puts the sensor >= 5 cm past the obstacle's face
 * (stop gap at most US_OBSTACLE_CM 15), so the first side check is
 * already beside the obstacle; then BYPASS_ALONG_STEP_CM steps.       */
#define BYPASS_ALONG_FIRST_CM (20U)
#define BYPASS_ALONG_STEP_CM (20U)
/* Side check: obstacle on the right of a left bypass (servo 15 deg, the
 * sweep end), on the left of a right one. It sits side gap + half width
 * = 15-35 cm from the sensor; under BYPASS_SIDE_CM means still there.  */
#define BYPASS_SIDE_DEG ((0 != BYPASS_ON_LEFT) ? SCAN_MIN_DEG : SCAN_MAX_DEG)
#define BYPASS_SIDE_CM       (45U)
/* Once the side reads clear the obstacle ends at most ~1 cm ahead of the
 * sensor (the 15 deg ray leans forward), i.e. 8.5 cm ahead of the axle.
 * Driving 20 cm more puts it behind the car's back side once it turns
 * toward the line (7.5 cm behind the axle), with 4 cm spare.           */
#define BYPASS_PAST_CM       (20U)
/* Check rays, fixed and not SCAN_MIN/MAX_DEG: at BYPASS_PEEK_CM a 60 deg
 * ray is 10 cm to the side and 17 cm ahead, just outside the car's
 * half width (7.5 cm), so the check sees the strip the body drives
 * into. A 15 deg ray would point sideways (19 cm out, 5 cm ahead) and
 * miss an obstacle edge in front of the car's side.                    */
#define PEEK_RIGHT_DEG       (60U)
#define PEEK_LEFT_DEG        (120U)
/* Steps per side (width or length) before giving up with FC_NO_ROUTE
 * detail 3 ("obstacle too large"): 10 x 20 = 200 cm.                   */
#define BYPASS_MAX_STEPS     (10U)
/* Return leg toward the line: the measured sideways steps plus this
 * much extra before the car gives up (FC_REACQUIRE).                   */
#define BYPASS_RETURN_EXTRA_CM (15U)
/* Line sensors to wheel axle. After the sensors cross the line the car
 * drives this far so the axle is on the line, then spot-turns onto it;
 * otherwise the turn leaves the car parallel to, but beside, the line.
 * TUNE: measure on the car. 0 turns as soon as the line is seen.       */
#define LINE_SENSOR_AHEAD_CM (8U)
/* Backstop over the longest return leg (BYPASS_MAX_STEPS x
 * BYPASS_STEP_CM + margin + extra, ~2.2 m at SPEED_SEARCH_CMS ~25 s),
 * the axle step and the turn. TUNE.                                    */
#define REACQUIRE_TIMEOUT_MS (30000U)

/* ------------------------------------------------------------------ */
/* Navigation                                                           */
/* ------------------------------------------------------------------ */
#define INIT_TIMEOUT_MS      (15000U)
#define BARCODE_TURN_DEG     (90U)
#define BARCODE_STRAIGHT_CM  (10U)
/* At a junction the line sensors reach the cross line before the axle:
 * drive this far first so the spot turn pivots on the junction and the
 * sensors swing onto the branch (Left, Right, U-turn). Line sensors to
 * axle (measured 6 cm) plus about half the line width. 0 = turn at once.
 * TUNE: turns too early -> raise; overshoots the branch -> lower.     */
#define JUNCTION_AHEAD_CM    (LINE_SENSOR_AHEAD_CM)
#define UTURN_DEG            (180U)
/* Dashboard calibration: after the compass spin and after the 90 deg
 * check turn the car waits this long, motors off, before reading the
 * heading (> 10 IMU samples, offsets applied).                       */
#define CAL_SETTLE_MS        (300U)

/* ------------------------------------------------------------------ */
/* Safety: per-task heartbeat deadlines (Week 6 feedback)               */
/* ------------------------------------------------------------------ */
#define SAFETY_BOOT_GRACE_MS (2000U)
#define HB_DEADLINE_LINE_MS  (100U)    /* period 5 ms                   */
#define HB_DEADLINE_MOTION_MS (100U)   /* period 10 ms                  */
#define HB_DEADLINE_IMU_MS   (200U)    /* period 20 ms + I2C timeouts   */
#define HB_DEADLINE_US_MS    (900U)    /* 180 deg settle 740 + ping 31  */
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
# define MQTT_BROKER_IP      "172.20.10.6"   /* fallback: beacon wins    */
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
#define MQTT_OFFLINE_BUF     (10U)     /* 1 s of telemetry at 100 ms    */

#endif /* CAR_CONFIG_H */

/*** end of file ***/
