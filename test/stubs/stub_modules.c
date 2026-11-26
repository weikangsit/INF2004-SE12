/** @file    stub_modules.c  (TEST STUB)
 *  @brief   Fake firmware modules. A group is left out when the test builds
 *           the real module: the build passes -DREAL_<GROUP>.
 *
 *  Every fake only records what it was asked to do (see stub_modules.h) and
 *  returns a value the test can set. None of them does real work.
 */
#include <string.h>

#include <tk/tkernel.h>

#include "barcode.h"
#include "barcode_map.h"
#include "comms.h"
#include "cyw43_utk.h"
#include "fault.h"
#include "hal.h"
#include "i2c_drv.h"
#include "imu_terrain.h"
#include "motion_control.h"
#include "motor_driver.h"
#include "nav_fsm.h"
#include "safety.h"
#include "sensor_line.h"
#include "stub_modules.h"
#include "telemetry.h"
#include "ultrasonic_scan.h"
#include "vehicle_state.h"

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

stub_event_t    stub_events[STUB_MAX_RECORDS];
int             stub_event_count;
uint32_t        stub_safety_reports[STUB_MAX_RECORDS];
int             stub_safety_report_count;
int             stub_inhibit_calls;
bool            stub_inhibit_state;
int             stub_motion_stop_calls;
int             stub_forward_calls;
uint32_t        stub_forward_speed_cms;
uint32_t        stub_forward_dist_cm;
int             stub_turn_calls;
bool            stub_turn_left;
uint32_t        stub_turn_deg;
bool            stub_radio_alive = true;
bool            stub_mag_calibrated;
bool            stub_latched;
vehicle_state_t stub_vstate;
vehicle_state_t stub_imu_out;

static uint8_t  g_seq = 0U;

void
stub_modules_reset (void)
{
    (void) memset(stub_events, 0, sizeof(stub_events));
    (void) memset(stub_safety_reports, 0, sizeof(stub_safety_reports));
    (void) memset(&stub_vstate, 0, sizeof(stub_vstate));
    (void) memset(&stub_imu_out, 0, sizeof(stub_imu_out));
    stub_event_count         = 0;
    stub_safety_report_count = 0;
    stub_inhibit_calls       = 0;
    stub_inhibit_state       = false;
    stub_motion_stop_calls   = 0;
    stub_forward_calls       = 0;
    stub_forward_speed_cms   = 0U;
    stub_forward_dist_cm     = 0U;
    stub_turn_calls          = 0;
    stub_turn_left           = false;
    stub_turn_deg            = 0U;
    stub_radio_alive         = true;
    stub_mag_calibrated      = false;
    stub_latched             = false;
    g_seq                    = 0U;
}

int
stub_events_of_kind (int kind)
{
    int count = 0;
    int k     = 0;

    for (k = 0; k < stub_event_count; k++)
    {
        if (stub_events[k].kind == kind)
        {
            count++;
        }
    }

    return (count);
}

/** Next command sequence number 1..7, as Motion Control hands out. */
static uint8_t
seq_next (void)
{
    g_seq = (uint8_t) ((g_seq % 7U) + 1U);

    return (g_seq);
}

/* -------------------------------------------------------------- telemetry */
#ifndef REAL_TELEMETRY
void
telemetry_post_event (proto_event_t kind, uint8_t code, int16_t value1,
                      int16_t value2)
{
    if (stub_event_count < STUB_MAX_RECORDS)
    {
        stub_events[stub_event_count].kind   = (int) kind;
        stub_events[stub_event_count].code   = code;
        stub_events[stub_event_count].value1 = value1;
        stub_events[stub_event_count].value2 = value2;
        stub_event_count++;
    }
}

void
telemetry_post_debug (uint8_t const * p_frame, uint32_t len)
{
    (void) p_frame;
    (void) len;
}

void telemetry_init (void) {}
void telemetry_task (INT stacd, void * p_exinf) { (void) stacd; (void) p_exinf; }
#endif

/* ----------------------------------------------------------------- safety */
#ifndef REAL_SAFETY
void
safety_heartbeat (uint32_t task_bit)
{
    (void) task_bit;
}

void
safety_report (uint32_t events)
{
    if (stub_safety_report_count < STUB_MAX_RECORDS)
    {
        stub_safety_reports[stub_safety_report_count] = events;
        stub_safety_report_count++;
    }
}

bool
safety_is_latched (void)
{
    return (stub_latched);
}

void safety_clear (void) { stub_latched = false; }
void safety_init (void) {}
void safety_task (INT stacd, void * p_exinf) { (void) stacd; (void) p_exinf; }
#endif

/* ----------------------------------------------------------- motor driver */
#ifndef REAL_MOTOR
void
motor_driver_inhibit (bool b_inhibit)
{
    stub_inhibit_calls++;
    stub_inhibit_state = b_inhibit;
}

void motor_driver_init (void) {}
void motor_driver_set (int32_t duty_l_pm, int32_t duty_r_pm) { (void) duty_l_pm; (void) duty_r_pm; }
void motor_driver_brake (void) {}
int32_t motor_driver_count_left (void) { return (0); }
int32_t motor_driver_count_right (void) { return (0); }
uint32_t motor_driver_edge_ms_left (void) { return (0U); }
uint32_t motor_driver_edge_ms_right (void) { return (0U); }
void motor_driver_encoder_isr (uint32_t now_ms) { (void) now_ms; }
#endif

/* ---------------------------------------------------------- motion control */
#ifndef REAL_MOTION
void
motion_control_stop (void)
{
    stub_motion_stop_calls++;
}

uint8_t
motion_control_forward (uint32_t speed_cms, uint32_t dist_cm, bool b_follow_line)
{
    (void) b_follow_line;
    stub_forward_calls++;
    stub_forward_speed_cms = speed_cms;
    stub_forward_dist_cm   = dist_cm;

    return (seq_next());
}

uint8_t
motion_control_turn (bool b_left, uint32_t speed_cms, uint32_t angle_deg,
                     bool b_end_on_line)
{
    (void) speed_cms;
    (void) b_end_on_line;
    stub_turn_calls++;
    stub_turn_left = b_left;
    stub_turn_deg  = angle_deg;

    return (seq_next());
}

uint8_t
motion_control_turn_search (bool b_left, uint32_t speed_cms, uint32_t max_deg,
                            uint32_t min_deg)
{
    (void) speed_cms;
    (void) min_deg;
    stub_turn_calls++;
    stub_turn_left = b_left;
    stub_turn_deg  = max_deg;

    return (seq_next());
}

uint8_t motion_control_self_test (void) { return (seq_next()); }
uint8_t motion_control_motor_test (void) { return (seq_next()); }
uint8_t motion_control_deadband_test (void) { return (seq_next()); }
bool motion_control_done_take (motion_done_t * p_done) { (void) p_done; return (false); }
bool motion_control_is_open_loop (void) { return (false); }
void motion_control_encoder_signs (int32_t * p_sign_l, int32_t * p_sign_r) { *p_sign_l = 1; *p_sign_r = 1; }
void motion_control_init (void) {}
void motion_control_task (INT stacd, void * p_exinf) { (void) stacd; (void) p_exinf; }
#endif

/* ------------------------------------------------------------ line sensor */
#ifndef REAL_SENSOR_LINE
bool sensor_line_read (uint8_t * p_bits) { *p_bits = 0U; return (true); }
uint32_t sensor_line_black_age_ms (void) { return (0U); }
uint32_t sensor_line_both_black_ms (void) { return (0U); }
uint32_t sensor_line_events_take (void) { return (0U); }
void sensor_line_request_calibration (void) {}
bool sensor_line_is_ready (void) { return (true); }
void sensor_line_init (void) {}
void sensor_line_task (INT stacd, void * p_exinf) { (void) stacd; (void) p_exinf; }
#endif

/* ---------------------------------------------------------------- barcode */
#ifndef REAL_BARCODE
void barcode_init (void) {}
void barcode_poll (void) {}
bool barcode_take (uint8_t * p_slot) { (void) p_slot; return (false); }
bool barcode_is_reading (void) { return (false); }
#endif

#ifndef REAL_BARCODE_MAP
void barcode_map_init (void) {}
barcode_map_action_t barcode_map_get (char letter) { (void) letter; return (BA_NONE); }
void barcode_map_apply (uint32_t table) { (void) table; }
uint32_t barcode_map_table (void) { return (0U); }
bool barcode_map_is_loaded (void) { return (false); }
#endif

/* ---------------------------------------------------------- vehicle state */
#ifndef REAL_VSTATE
void vehicle_state_init (void) {}
void vehicle_state_read (vehicle_state_t * p_out) { *p_out = stub_vstate; }

void
vehicle_state_set_motion (int32_t speed_l_mms, int32_t speed_r_mms,
                          int32_t duty_l_pct, int32_t duty_r_pct,
                          int32_t dist_mm)
{
    (void) speed_l_mms; (void) speed_r_mms; (void) duty_l_pct;
    (void) duty_r_pct; (void) dist_mm;
}

void vehicle_state_set_imu (vehicle_state_t const * p_imu) { stub_imu_out = *p_imu; }
void vehicle_state_set_line (uint8_t ir_black) { (void) ir_black; }
void vehicle_state_set_barcode (uint8_t last_barcode) { (void) last_barcode; }
void vehicle_state_set_nav (uint8_t nav_state) { stub_vstate.nav_state = nav_state; }
#endif

/* ---------------------------------------------------------------- ultrasonic */
#ifndef REAL_US
void ultrasonic_scan_request (void) {}
void ultrasonic_scan_abort (void) {}
bool ultrasonic_scan_result_take (us_result_t * p_result) { (void) p_result; return (false); }
uint16_t ultrasonic_scan_front_cm (void) { return (US_RANGE_NO_ECHO); }
void ultrasonic_scan_peek (void) {}
void ultrasonic_scan_side_peek (void) {}
bool ultrasonic_scan_is_ok (void) { return (true); }
void ultrasonic_scan_init (void) {}
void ultrasonic_scan_task (INT stacd, void * p_exinf) { (void) stacd; (void) p_exinf; }
#endif

/* -------------------------------------------------------------------- imu */
#ifndef REAL_IMU
bool imu_terrain_event_take (imu_terrain_event_t * p_event) { (void) p_event; return (false); }
void imu_terrain_mag_cal_start (void) {}
void imu_terrain_mag_cal_finish (void) {}
bool imu_terrain_mag_is_calibrated (void) { return (stub_mag_calibrated); }
bool imu_terrain_is_ok (void) { return (true); }
void imu_terrain_init (void) {}
void imu_terrain_task (INT stacd, void * p_exinf) { (void) stacd; (void) p_exinf; }
#endif

/* ------------------------------------------------------------------ comms */
#ifndef REAL_COMMS
bool comms_publish (comms_topic_t topic, uint8_t const * p_data, uint32_t len) { (void) topic; (void) p_data; (void) len; return (true); }
bool comms_is_online (void) { return (true); }
bool comms_radio_is_alive (void) { return (stub_radio_alive); }
void comms_init (void) {}
#endif

/* ----------------------------------------------------------------- nav fsm */
#ifndef REAL_NAV
void nav_fsm_post_remote (uint8_t cmd) { (void) cmd; }
void nav_fsm_init (void) {}
void nav_fsm_task (INT stacd, void * p_exinf) { (void) stacd; (void) p_exinf; }
#endif

/* -------------------------------------------------------------------- hal */
void hal_led_set (hal_led_colour_t colour) { (void) colour; }
void hal_led_init (void) {}
bool hal_btn_start_raw (void) { return (false); }
bool hal_btn_cal_raw (void) { return (false); }

/* ------------------------------------------------------------ i2c driver */
void i2c_drv_init (uint32_t pin_sda, uint32_t pin_scl, uint32_t baud) { (void) pin_sda; (void) pin_scl; (void) baud; }
bool i2c_drv_write (uint8_t addr, uint8_t const * p_src, uint32_t len, bool b_nostop, uint32_t timeout_us) { (void) addr; (void) p_src; (void) len; (void) b_nostop; (void) timeout_us; return (true); }
bool i2c_drv_read (uint8_t addr, uint8_t * p_dst, uint32_t len, uint32_t timeout_us) { (void) addr; (void) p_dst; (void) len; (void) timeout_us; return (true); }

/* ----------------------------------------------- lwIP, MQTT and radio fakes */
ip_addr_t const stub_ip_any = { { 0U }, 0U };

mqtt_client_t * mqtt_client_new (void) { static mqtt_client_t c; return (&c); }
err_t mqtt_client_connect (mqtt_client_t * p_client, ip_addr_t const * p_addr, u16_t port, mqtt_connection_cb_t cb, void * p_arg, struct mqtt_connect_client_info_t const * p_info) { (void) p_client; (void) p_addr; (void) port; (void) cb; (void) p_arg; (void) p_info; return (ERR_OK); }
u8_t mqtt_client_is_connected (mqtt_client_t * p_client) { (void) p_client; return (0U); }
void mqtt_disconnect (mqtt_client_t * p_client) { (void) p_client; }
void mqtt_set_inpub_callback (mqtt_client_t * p_client, mqtt_incoming_publish_cb_t pub_cb, mqtt_incoming_data_cb_t data_cb, void * p_arg) { (void) p_client; (void) pub_cb; (void) data_cb; (void) p_arg; }
err_t mqtt_subscribe (mqtt_client_t * p_client, char const * p_topic, u8_t qos, mqtt_request_cb_t cb, void * p_arg) { (void) p_client; (void) p_topic; (void) qos; (void) cb; (void) p_arg; return (ERR_OK); }
err_t mqtt_publish (mqtt_client_t * p_client, char const * p_topic, void const * p_payload, u16_t len, u8_t qos, u8_t retain, mqtt_request_cb_t cb, void * p_arg) { (void) p_client; (void) p_topic; (void) p_payload; (void) len; (void) qos; (void) retain; (void) cb; (void) p_arg; return (ERR_OK); }
int ip4addr_aton (char const * p_text, ip4_addr_t * p_addr) { (void) p_text; p_addr->addr = 0x0100007FU; return (1); }
char const * ipaddr_ntoa (ip_addr_t const * p_addr) { (void) p_addr; return ("0.0.0.0"); }
uint16_t pbuf_copy_partial (struct pbuf const * p_buf, void * p_dst, uint16_t len, uint16_t offset) { (void) p_buf; (void) p_dst; (void) len; (void) offset; return (0U); }
uint8_t pbuf_free (struct pbuf * p_buf) { (void) p_buf; return (1U); }
struct udp_pcb * udp_new_ip_type (u8_t type) { static struct udp_pcb pcb; (void) type; return (&pcb); }
err_t udp_bind (struct udp_pcb * p_pcb, ip_addr_t const * p_addr, u16_t port) { (void) p_pcb; (void) p_addr; (void) port; return (ERR_OK); }
void udp_recv (struct udp_pcb * p_pcb, udp_recv_fn recv, void * p_arg) { (void) p_pcb; (void) recv; (void) p_arg; }
void udp_remove (struct udp_pcb * p_pcb) { (void) p_pcb; }

void
cyw43_utk_get_status (T_CYW43_UTK_STATUS * p_status)
{
    (void) memset(p_status, 0, sizeof(*p_status));
}

/*** end of file ***/
