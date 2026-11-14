/**
 * @file usb_descriptors.c
 * @brief USB descriptors: composite device, 2x CDC-ACM with IAD.
 * @author Ivan Lok
 *
 * Host sees two serial ports:
 *   macOS : /dev/cu.usbmodem<serial>1 (CMD), /dev/cu.usbmodem<serial>3 (LOG)
 *   Linux : /dev/ttyACM0 (CMD), /dev/ttyACM1 (LOG) -> udev symlinks quads_cmd / quads_log
 */
#include <string.h>
#include "main.h"
#include "tusb.h"

/* TODO: 0xCafe is TinyUSB's test VID; fine for the lab, not for distribution. */
#define QUADS_USB_VID   0xCafe
#define QUADS_USB_PID   0x4C51   /* "LQ" */
#define QUADS_USB_BCD   0x0200

/*------------- Device descriptor -------------*/
static const tusb_desc_device_t desc_device = {
  .bLength            = sizeof(tusb_desc_device_t),
  .bDescriptorType    = TUSB_DESC_DEVICE,
  .bcdUSB             = QUADS_USB_BCD,
  /* IAD: required so the host groups each CDC's 2 interfaces together */
  .bDeviceClass       = TUSB_CLASS_MISC,
  .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
  .bDeviceProtocol    = MISC_PROTOCOL_IAD,
  .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
  .idVendor           = QUADS_USB_VID,
  .idProduct          = QUADS_USB_PID,
  .bcdDevice          = 0x0100,
  .iManufacturer      = 0x01,
  .iProduct           = 0x02,
  .iSerialNumber      = 0x03,
  .bNumConfigurations = 0x01,
};

/**
 * @brief TinyUSB callback: return the device descriptor.
 */
const uint8_t *tud_descriptor_device_cb(void)
{
  return (const uint8_t *)&desc_device;
}

/*------------- Configuration descriptor -------------*/
enum {
  ITF_NUM_CDC_CMD = 0,
  ITF_NUM_CDC_CMD_DATA,
  ITF_NUM_CDC_LOG,
  ITF_NUM_CDC_LOG_DATA,
  ITF_NUM_TOTAL
};

/* Endpoints: CMD on EP1/EP2, LOG on EP3/EP4 (DWC2 allows IN+OUT on same number) */
#define EPNUM_CDC_CMD_NOTIF  0x81
#define EPNUM_CDC_CMD_OUT    0x02
#define EPNUM_CDC_CMD_IN     0x82
#define EPNUM_CDC_LOG_NOTIF  0x83
#define EPNUM_CDC_LOG_OUT    0x04
#define EPNUM_CDC_LOG_IN     0x84

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + CFG_TUD_CDC * TUD_CDC_DESC_LEN)

static const uint8_t desc_configuration[] = {
  /* config number, interface count, string index, total length, attribute, power mA */
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
  /* itf, string idx, notif EP, notif size, OUT EP, IN EP, EP size */
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_CMD, 4, EPNUM_CDC_CMD_NOTIF, 8, EPNUM_CDC_CMD_OUT, EPNUM_CDC_CMD_IN, 64),
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_LOG, 5, EPNUM_CDC_LOG_NOTIF, 8, EPNUM_CDC_LOG_OUT, EPNUM_CDC_LOG_IN, 64),
};

/**
 * @brief TinyUSB callback: return the configuration descriptor (2x CDC).
 * @param index Configuration index (only one configuration).
 */
const uint8_t *tud_descriptor_configuration_cb(uint8_t index)
{
  (void)index;
  return desc_configuration;
}

/*------------- String descriptors -------------*/
enum {
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
  STRID_CDC_CMD,
  STRID_CDC_LOG,
};

static const char *const string_desc_arr[] = {
  [STRID_MANUFACTURER] = "CMU Robotics Club",
  [STRID_PRODUCT]      = "Quads MCU",
  [STRID_SERIAL]       = NULL,            /* from chip UID */
  [STRID_CDC_CMD]      = "Quads CMD",
  [STRID_CDC_LOG]      = "Quads LOG",
};

static uint16_t desc_str[32 + 1];

/**
 * @brief Convert the 96-bit chip UID to 24 hex chars (UTF-16) for the serial number.
 * @note Stable per chip, so udev/macOS can tell boards apart.
 */
static size_t uid_to_utf16(uint16_t *out, size_t max_chars)
{
  static const char hex[] = "0123456789ABCDEF";
  const uint32_t uid[3] = {
    *(const volatile uint32_t *)(UID_BASE + 0U),
    *(const volatile uint32_t *)(UID_BASE + 4U),
    *(const volatile uint32_t *)(UID_BASE + 8U),
  };
  size_t n = 0;
  for (size_t w = 0; w < 3U; w++) {
    for (int nib = 7; nib >= 0 && n < max_chars; nib--) {
      out[n++] = (uint16_t)hex[(uid[w] >> (nib * 4)) & 0xFU];
    }
  }
  return n;
}

/**
 * @brief TinyUSB callback: return string descriptor `index` as UTF-16.
 * @param index String index (STRID_*).
 * @param langid Ignored (English only).
 */
const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void)langid;
  const size_t max_chars = (sizeof(desc_str) / sizeof(desc_str[0])) - 1U;
  size_t chr_count;

  if (index == STRID_LANGID) {
    desc_str[1] = 0x0409;   /* English (US) */
    chr_count = 1;
  } else if (index == STRID_SERIAL) {
    chr_count = uid_to_utf16(&desc_str[1], max_chars);
  } else {
    if (index >= (sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) || string_desc_arr[index] == NULL) {
      return NULL;
    }
    const char *str = string_desc_arr[index];
    chr_count = strlen(str);
    if (chr_count > max_chars) {
      chr_count = max_chars;
    }
    for (size_t i = 0; i < chr_count; i++) {
      desc_str[1 + i] = (uint16_t)str[i];
    }
  }

  /* byte 0: total length, byte 1: descriptor type */
  desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2U * chr_count + 2U));
  return desc_str;
}
