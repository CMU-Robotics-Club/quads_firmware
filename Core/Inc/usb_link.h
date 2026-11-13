/**
 * @file usb_link.h
 * @brief Jetson <-> STM32 link over the user USB port (TinyUSB, 2x CDC-ACM).
 * @author Ivan Lok
 */
#ifndef USB_LINK_H
#define USB_LINK_H

#include <stdbool.h>

/**
 * @brief UsbTask entry: USB hardware init, then runs tud_task() forever.
 * @param argument Unused CMSIS-RTOS2 task argument.
 * @note Created in freertos.c (USER CODE RTOS_THREADS), priority AboveNormal.
 */
void StartUsbTask(void *argument);

/**
 * @brief True when the host has enumerated and configured the device.
 * @note No VBUS sensing, so an unplug may not clear this. Use command timeouts for safety.
 */
bool usb_link_connected(void);

#endif /* USB_LINK_H */
