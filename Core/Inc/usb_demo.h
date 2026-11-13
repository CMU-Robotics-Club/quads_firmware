/**
 * @file usb_demo.h
 * @brief Demo task: shows how firmware code reads USB commands with qp_update().
 * @author Ivan Lok
 */
#ifndef USB_DEMO_H
#define USB_DEMO_H

/**
 * @brief UsbDemoTask entry: print new CMD_JOINT / CMD_GAINS values over RTT.
 * @param argument Unused CMSIS-RTOS2 task argument.
 * @note Created in freertos.c (USER CODE RTOS_THREADS), priority Low. Remove the
 * osThreadNew line there to disable it.
 */
void StartUsbDemoTask(void *argument);

#endif /* USB_DEMO_H */
