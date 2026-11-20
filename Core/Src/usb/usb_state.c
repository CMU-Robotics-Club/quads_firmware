/**
 * @file usb_state.c
 * @brief STATE report: ControlTask hands over a snapshot, UsbTask sends it on CDC0.
 */
#include "usb_state.h"

#include <stdbool.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "tusb.h"
#include "usb_proto.h"

static qp_state_t latest;      /* written by ControlTask, read by UsbTask, both under the lock */
static bool       have_state;  /* false until the first publish */
static TickType_t next_send;   /* UsbTask only */

void usb_state_publish(const qp_state_t *state)
{
  taskENTER_CRITICAL();
  memcpy(&latest, state, sizeof(latest));
  have_state = true;
  taskEXIT_CRITICAL();
}

void usb_state_send_if_due(void)
{
  const TickType_t now = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(USB_STATE_PERIOD_MS);

  /* Signed difference handles tick wrap-around: "now is before next_send" */
  if ((int32_t)(now - next_send) < 0) {
    return;
  }
  /* Keep a fixed 10 ms grid so the rate stays 100 Hz even if a call comes late.
   * After a long gap (startup, host away) restart from now instead of sending a burst. */
  next_send = ((now - next_send) > 2U * period) ? now + period : next_send + period;

  /* No host reading the port (not plugged in, or port closed: DTR low): sending would only
   * fill the TX FIFO and count drops. */
  if (!tud_cdc_n_connected(QUADS_CDC_CMD)) {
    return;
  }

  qp_state_t copy;
  bool have;
  taskENTER_CRITICAL();
  have = have_state;
  memcpy(&copy, &latest, sizeof(copy));
  taskEXIT_CRITICAL();
  if (!have) {
    return;
  }

  (void)qp_send(QP_STATE, &copy, (uint8_t)sizeof(copy));  /* FIFO full: counted in tx_drops */
}
