/** @file    main.c
 *  @brief   usermain(): initialise every module and start the tasks
 *           (Week 6 Sections 3 and 4).
 *
 *  Each module creates and owns its kernel objects in its *_init()
 *  function (no shared globals), so the order below only has to brake
 *  the motors first and create every object before any task runs.
 *
 *  NOTE: (void) casts mark return values that are deliberately ignored.
 */
#include <string.h>

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "barcode_map.h"
#include "car_config.h"
#include "comms.h"
#include "fault.h"
#include "hal.h"
#include "hal_irq.h"
#include "imu_terrain.h"
#include "motion_control.h"
#include "motor_driver.h"
#include "nav_fsm.h"
#include "safety.h"
#include "sensor_line.h"
#include "telemetry.h"
#include "ultrasonic_scan.h"
#include "vehicle_state.h"

/* Stack sizes in bytes. */
#define STACK_SMALL          (1024)
#define STACK_SCAN           (1536)
#define STACK_DEFAULT        (2048)

static void task_start (FP entry, PRI priority, SZ stack_bytes);

/*!
 * @brief  Entry point called by micro T-Kernel after kernel start-up. The
 *         template has already started the USB console and the CYW43 radio
 *         task, which also runs the MQTT link (comms.c).
 * @return 0 (never returns while the system runs).
 */
EXPORT INT
usermain (void)
{
    tm_printf((UB *) "\n=== Group 12 robot car (micro T-Kernel) ===\n");

    /* Brake first, so the wheels cannot move during boot. */
    motor_driver_init();
    hal_servo_init();
    hal_buttons_init();
    hal_led_init();

    vehicle_state_init();
    fault_init();
    safety_init();
    telemetry_init();
    barcode_map_init();
    comms_init();
    motion_control_init();
    sensor_line_init();
    imu_terrain_init();
    ultrasonic_scan_init();
    nav_fsm_init();
    hal_isr_init();          /* last: the interrupt uses the objects above */

    task_start((FP) safety_task,          PRI_SAFETY, STACK_SMALL);
    task_start((FP) sensor_line_task,     PRI_LINE,   STACK_DEFAULT);
    task_start((FP) motion_control_task,  PRI_MOTION, STACK_DEFAULT);
    task_start((FP) nav_fsm_task,         PRI_NAV,    STACK_DEFAULT);
    task_start((FP) imu_terrain_task,     PRI_IMU,    STACK_DEFAULT);
    task_start((FP) ultrasonic_scan_task, PRI_US,     STACK_SCAN);
    task_start((FP) telemetry_task,       PRI_TELEM,  STACK_DEFAULT);
    /* No Comms task: MQTT runs inside the template's CYW43 task. */

    tm_printf((UB *) "car: all tasks started\n");

    /* usermain may sleep for ever; the tasks run on. */
    (void) tk_slp_tsk(TMO_FEVR);

    return (0);
}

/*!
 * @brief  Create and start one task.
 * @param  entry        Task function (cast to the kernel FP type).
 * @param  priority     Priority, 1 = highest.
 * @param  stack_bytes  Stack size.
 */
static void
task_start (FP entry, PRI priority, SZ stack_bytes)
{
    T_CTSK create;
    ID     task = 0;

    (void) memset(&create, 0, sizeof(create));
    create.tskatr  = TA_HLNG | TA_RNG3;
    create.task    = entry;
    create.itskpri = priority;
    create.stksz   = stack_bytes;
    task           = tk_cre_tsk(&create);

    if (task > 0)
    {
        (void) tk_sta_tsk(task, 0);
    }
    else
    {
        tm_printf((UB *) "car: task create failed (%d)\n", (int) task);
    }
}

/*** end of file ***/
