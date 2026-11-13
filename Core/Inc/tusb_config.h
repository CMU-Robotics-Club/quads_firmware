/**
 * @file tusb_config.h
 * @brief TinyUSB configuration for Quads (STM32H723, USB_OTG_HS core + internal FS PHY).
 * @author Ivan Lok
 *
 * Composite device, 2x CDC-ACM:
 *   CDC0 (itf 0) = CMD : binary COBS frames, Jetson <-> STM32
 *   CDC1 (itf 1) = LOG : text log stream (QUADS_LOG_BACKEND)
 *
 * Same build runs on Nucleo-H723ZG (micro-USB CN13) and Quads_MCU_PCB (USB-C J1):
 * both use PA11/PA12 and no VBUS sense pin.
 */
#ifndef TUSB_CONFIG_H_
#define TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/*------------- Board / port -------------*/
/* H72x/H73x has one USB core (USB1_OTG_HS). TinyUSB's dwc2_stm32.h aliases it to
 * rhport 0 when used with the internal FS PHY (same as TinyUSB's stm32h723nucleo BSP). */
#define BOARD_TUD_RHPORT        0

#define CFG_TUSB_MCU            OPT_MCU_STM32H7
#define CFG_TUSB_OS             OPT_OS_FREERTOS
#define CFG_TUSB_DEBUG          0

/* CubeMX's FreeRTOS (10.x) lacks pdTICKS_TO_MS, which TinyUSB's osal uses.
 * Expanded only after FreeRTOS.h is included, so configTICK_RATE_HZ is visible. */
#ifndef pdTICKS_TO_MS
#define pdTICKS_TO_MS(xTicks)   ((uint32_t)(((uint64_t)(xTicks) * 1000U) / configTICK_RATE_HZ))
#endif

/*------------- Device stack -------------*/
#define CFG_TUD_ENABLED         1
#define CFG_TUD_MAX_SPEED       OPT_MODE_FULL_SPEED   /* 12 Mbps, 1 ms frames */

/* No DMA: buffers live in DTCM (not reachable by the OTG DMA) and we avoid
 * D-cache maintenance. CPU copies to/from the core's FIFO instead (slave mode). */
#define CFG_TUD_DWC2_DMA_ENABLE 0

#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN      __attribute__((aligned(4)))

#define CFG_TUD_ENDPOINT0_SIZE  64

/*------------- Classes -------------*/
#define CFG_TUD_CDC             2
#define CFG_TUD_MSC             0
#define CFG_TUD_HID             0
#define CFG_TUD_MIDI            0
#define CFG_TUD_VENDOR          0

/* CDC interface indices (tud_cdc_n_* first argument) */
#define QUADS_CDC_CMD           0
#define QUADS_CDC_LOG           1

/* Software FIFO per CDC (shared size for both CDCs in TinyUSB).
 * RX 256: a few max-size CMD frames; TX 1024: absorbs log bursts. */
#define CFG_TUD_CDC_RX_BUFSIZE  256
#define CFG_TUD_CDC_TX_BUFSIZE  1024

/* Endpoint transfer buffer = FS bulk max packet */
#define CFG_TUD_CDC_RX_EPSIZE   64
#define CFG_TUD_CDC_TX_EPSIZE   64

#ifdef __cplusplus
}
#endif

#endif /* TUSB_CONFIG_H_ */
