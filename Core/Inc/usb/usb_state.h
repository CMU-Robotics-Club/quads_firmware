/**
 * @file usb_state.h
 * @brief STATE report (STM32 -> host): joint state + system state, sent every USB_STATE_PERIOD_MS.
 *
 * ControlTask calls usb_state_publish() every control period with the latest values.
 * UsbTask calls usb_state_send_if_due() in its loop and sends the latest copy on CDC0.
 * qp_send() is UsbTask-only, so ControlTask never sends; it only hands over a snapshot.
 * Both sides copy the whole struct inside a critical section, so a send never mixes
 * two control periods (same idea as the CMD_JOINTS channel in usb_proto.c).
 */
#ifndef USB_STATE_H
#define USB_STATE_H

#include <stdint.h>
#include "usb_msgs.h"

/** Type byte on the wire (0x80-0xFF = STM32 -> host). Reserved for STATE in doc/usb_protocol.md. */
#define QP_STATE            0x84U

/** Send period. 100 Hz: fresh enough for a ~50 Hz policy, small USB load (~16 KB/s). */
#define USB_STATE_PERIOD_MS 10U

/** One joint, joint side. All 0 until the leg is RUNNING. */
typedef struct __attribute__((packed)) {
  float q;        /**< rad, from the zero set at power-on */
  float qd;       /**< rad/s */
  float tau_cmd;  /**< Nm, torque the PD COMMANDED (after the clamp), not a measurement.
                       With PD_DRY_RUN 1 it is computed but not sent to the ODrive. */
} qp_joint_state_t;

/** STATE payload: 12 x 12 B + 1 B = 145 B. Joints without hardware stay 0. */
typedef struct __attribute__((packed)) {
  qp_joint_state_t j[QP_MAX_JOINTS];
  uint8_t sys_state;  /**< SystemState_t (joint_interface.h): 0 INIT, 1 WAIT_CAN, 2 CLEAR_ERRORS,
                           3 ARM_MOTORS, 4 CALIBRATE_OFFSETS, 5 RUNNING, 6 FAULT */
} qp_state_t;

_Static_assert(sizeof(qp_state_t) == 145U, "STATE payload must be 145 bytes (host decodes it)");

/** @brief Store the latest state. Any task (ControlTask calls it every 1 ms). */
void usb_state_publish(const qp_state_t *state);

/**
 * @brief Send the latest state if USB_STATE_PERIOD_MS has passed. UsbTask only.
 * Does nothing until the first publish, or while no host has the port open.
 */
void usb_state_send_if_due(void);

#endif /* USB_STATE_H */
