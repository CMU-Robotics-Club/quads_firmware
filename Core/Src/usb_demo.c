/**
 * @file usb_demo.c
 * @brief Demo task: shows how firmware code reads USB commands with qp_update().
 * @author Ivan Lok
 *
 * Pattern for any consumer (e.g. ControlTask later):
 *   1. Keep a version counter per slot you read, start at 0.
 *   2. Call qp_update(): returns 1 and copies the struct only when new data arrived.
 *   3. Use your own copy; never touch the channel storage directly.
 *
 * Test from the host: python3 scripts/quads_host.py joint / gains
 */
#include "usb_demo.h"

#include "cmsis_os2.h"
#include "quads_proto.h"
#include "rtt_log.h"

#define USB_DEMO_JOINTS   2U    /* knee (0), hip (1) */
#define USB_DEMO_PERIOD   200U  /* ms */

void StartUsbDemoTask(void *argument)
{
  (void)argument;
  uint32_t joint_ver[USB_DEMO_JOINTS] = {0};
  uint32_t gains_ver[USB_DEMO_JOINTS] = {0};

  for (;;) {
    osDelay(USB_DEMO_PERIOD);

    for (uint8_t id = 0; id < USB_DEMO_JOINTS; id++) {
      qp_cmd_joint_t j;
      if (qp_update(&qp_ch_CMD_JOINT, id, &j, &joint_ver[id]) > 0) {
        /* milli-units: fast_printf avoids float formatting */
        fast_printf("CMD_JOINT j%u ver=%lu q=%ld mrad qd=%ld mrad/s tau=%ld mNm\r\n",
                    (unsigned)id, (unsigned long)joint_ver[id],
                    (long)(j.q_des * 1000.0f), (long)(j.qd_des * 1000.0f),
                    (long)(j.tau_ff * 1000.0f));
      }

      qp_cmd_gains_t g;
      if (qp_update(&qp_ch_CMD_GAINS, id, &g, &gains_ver[id]) > 0) {
        fast_printf("CMD_GAINS j%u kp=%ld/1000 kd=%ld/1000 tau_max=%ld mNm\r\n",
                    (unsigned)id, (long)(g.kp * 1000.0f), (long)(g.kd * 1000.0f),
                    (long)(g.tau_max * 1000.0f));
      }
    }
  }
}
