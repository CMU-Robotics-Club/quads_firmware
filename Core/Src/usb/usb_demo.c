/**
 * @file usb_demo.c
 * @brief Demo task: shows how firmware code reads USB commands with qp_update().
 * @author Ivan Lok
 *
 * Pattern for any consumer (e.g. ControlTask later):
 *   1. Keep one version counter per channel you read, start at 0.
 *   2. Call qp_update(): returns 1 and copies the struct only when new data arrived.
 *   3. Use your own copy; never touch the channel storage directly.
 *
 * Test from the host: python3 scripts/quads_host.py joints / gains
 */
#include "usb_demo.h"

#include "cmsis_os2.h"
#include "usb_proto.h"
#include "rtt_log.h"

#define USB_DEMO_JOINTS   2U    /* knee (0), hip (1) */
#define USB_DEMO_PERIOD   200U  /* ms */

void StartUsbDemoTask(void *argument)
{
  (void)argument;
  uint32_t joints_ver = 0;
  uint32_t gains_ver = 0;

  for (;;) {
    osDelay(USB_DEMO_PERIOD);

    qp_cmd_joints_t sp;
    if (qp_update(&qp_ch_CMD_JOINTS, &sp, &joints_ver) > 0) {
      for (uint8_t i = 0; i < USB_DEMO_JOINTS; i++) {
        /* milli-units: fast_printf avoids float formatting */
        fast_printf("JOINTS ver=%lu j%u q=%ld mrad qd=%ld mrad/s tau=%ld mNm\r\n",
                    (unsigned long)joints_ver, (unsigned)i,
                    (long)(sp.j[i].q_des * 1000.0f), (long)(sp.j[i].qd_des * 1000.0f),
                    (long)(sp.j[i].tau_ff * 1000.0f));
      }
    }

    qp_cmd_gains_t g;
    if (qp_update(&qp_ch_CMD_GAINS, &g, &gains_ver) > 0) {
      for (uint8_t i = 0; i < USB_DEMO_JOINTS; i++) {
        fast_printf("GAINS j%u kp=%ld/1000 kd=%ld/1000 tau_max=%ld mNm\r\n",
                    (unsigned)i, (long)(g.j[i].kp * 1000.0f), (long)(g.j[i].kd * 1000.0f),
                    (long)(g.j[i].tau_max * 1000.0f));
      }
    }
  }
}
