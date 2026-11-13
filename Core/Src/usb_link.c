/**
 * @file usb_link.c
 * @brief USB bring-up, UsbTask body and USB IRQ for the Jetson <-> STM32 link.
 * @author Ivan Lok
 *
 * Threading model (no polling):
 *   OTG_HS IRQ (prio 6)  -> tud_int_handler(): only queues an event, returns fast
 *   UsbTask (AboveNormal) -> tud_task(): blocks on that queue, runs class drivers
 *                            and the tud_cdc_* callbacks below
 *   ControlTask never calls tud_*; it only reads the latest command snapshot.
 *
 * CDC0 = command frames (quads_proto.c), CDC1 = log (echo until the log backend lands).
 */
#include "usb_link.h"

#include "main.h"
#include "usb_otg.h"
#include "tusb.h"
#include "quads_proto.h"

/**
 * @brief Configure PA11/PA12 and reuse CubeMX MspInit (HSI48 clock, voltage detector, NVIC).
 */
static void usb_hw_init(void)
{
  /* PA11 = DM, PA12 = DP, AF10. CubeMX's MspInit does not configure these pins. */
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  gpio.Pin       = GPIO_PIN_11 | GPIO_PIN_12;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_NOPULL;
  gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(GPIOA, &gpio);

  /* Reuse CubeMX MspInit: USB clock = HSI48, USB voltage detector, core clock,
   * NVIC prio 6. We do NOT call MX_USB_OTG_HS_PCD_Init(): TinyUSB owns the core. */
  hpcd_USB_OTG_HS.Instance = USB_OTG_HS;
  HAL_PCD_MspInit(&hpcd_USB_OTG_HS);
}

/**
 * @brief UsbTask entry: init USB hardware and TinyUSB, then run tud_task() forever.
 * @param argument Unused CMSIS-RTOS2 task argument.
 */
void StartUsbTask(void *argument)
{
  (void)argument;

  usb_hw_init();

  /* No VBUS sense pin on either board: core forces "session valid". */
  tud_configure_dwc2_t dwc2_cfg = CFG_TUD_CONFIGURE_DWC2_DEFAULT;
  dwc2_cfg.vbus_sensing = false;
  tud_configure(BOARD_TUD_RHPORT, TUD_CFGID_DWC2, &dwc2_cfg);

  const tusb_rhport_init_t dev_init = {
    .role  = TUSB_ROLE_DEVICE,
    .speed = TUSB_SPEED_FULL,
  };
  tusb_init(BOARD_TUD_RHPORT, &dev_init);

  for (;;) {
    /* Blocks until the IRQ posts an event. Zero CPU when the bus is idle. */
    tud_task();
  }
}

/**
 * @brief True when the host has enumerated and configured the device.
 */
bool usb_link_connected(void)
{
  return tud_mounted();
}

/*------------- IRQ -------------*/

/**
 * @brief USB IRQ: hand the event to TinyUSB, which queues it for UsbTask.
 * @note Defined here, not in stm32h7xx_it.c. Keep "Generate IRQ handler" for
 * USB_OTG_HS unticked in CubeMX, or this becomes a duplicate symbol.
 */
void OTG_HS_IRQHandler(void)
{
  tud_int_handler(BOARD_TUD_RHPORT);
}

/*------------- CDC callbacks (run in UsbTask) -------------*/

/**
 * @brief P1 test: echo every received byte back on the same CDC.
 * @param itf CDC index (QUADS_CDC_CMD or QUADS_CDC_LOG).
 */
static void usb_echo(uint8_t itf)
{
  uint8_t buf[64];
  while (tud_cdc_n_available(itf) > 0U) {
    uint32_t n = tud_cdc_n_read(itf, buf, sizeof(buf));
    tud_cdc_n_write(itf, buf, n);
  }
  tud_cdc_n_write_flush(itf);
}

/**
 * @brief TinyUSB callback on CDC data received (runs in UsbTask).
 * @param itf CDC index (QUADS_CDC_CMD or QUADS_CDC_LOG).
 * @note Name is fixed by TinyUSB (weak symbol override); do not rename.
 * CDC0 (CMD) -> COBS frame parser. CDC1 (LOG) is TX-only for now; echo kept for debug.
 */
void tud_cdc_rx_cb(uint8_t itf)
{
  if (itf == QUADS_CDC_CMD) {
    uint8_t buf[64];
    while (tud_cdc_n_available(itf) > 0U) {
      uint32_t n = tud_cdc_n_read(itf, buf, sizeof(buf));
      qp_rx_bytes(buf, n);
    }
  } else {
    usb_echo(itf);
  }
}
