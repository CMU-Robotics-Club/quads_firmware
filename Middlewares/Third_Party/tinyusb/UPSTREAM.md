Source: https://github.com/hathach/tinyusb
Tag: 0.21.0
Commit: dae3f9a366bfcddbf9dcf1b48d7500286a849539

Upstream sources are unchanged. Only a subset of the repo is vendored:
- LICENSE
- src/ (everything except src/typec/)
- src/portable/: only synopsys/dwc2 (STM32H7 USB_OTG_HS = DWC2 core)

Project configuration (tusb_config.h, usb_descriptors.c) lives outside this folder.
To update: replace src/ with the new upstream src/, re-apply the same trimming,
and update the tag/commit above.
