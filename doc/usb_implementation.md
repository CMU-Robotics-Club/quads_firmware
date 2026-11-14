# USB implementation

How the USB command link works inside the firmware. For adding or sending messages, see [usb_protocol.md](usb_protocol.md).

## Files

| File | Does | Runs in |
|---|---|---|
| `usb_link.c` | USB init, UsbTask loop, USB IRQ, CDC RX callback | ISR + UsbTask |
| `usb_descriptors.c` | Composite device: 2x CDC-ACM, serial number = chip UID | UsbTask |
| `usb_proto.c` | Frame parser, COBS, CRC, channel read/write, TX | UsbTask (`qp_update` in any task) |
| `usb_msgs.h/.c` | Message list, payload structs, validate functions | compile time |
| `usb_demo.c` | Example reader | UsbDemoTask (Low) |

## Threads

```
OTG_HS IRQ (prio 6)    tud_int_handler(): queue an event, return
UsbTask (AboveNormal)  tud_task(): block on that queue, run CDC callbacks and the parser
ControlTask / others   qp_update(): copy the latest value out of a channel
```

All protocol code runs in UsbTask. Other tasks never call `tud_*`. `qp_send()` is UsbTask-only.

## Receive path

1. The host sends 64-byte USB packets. TinyUSB puts them in the CDC0 RX FIFO (256 B). When the FIFO is full the endpoint is not re-armed, so the host gets NAK and waits. Nothing is lost.
2. `tud_cdc_rx_cb()` reads the FIFO in 64-byte chunks and passes them to `qp_rx_bytes()`. A frame can span several calls.
3. `qp_rx_bytes()` collects non-zero bytes in `enc_buf`. On `0x00` it decodes the frame and resets. If the buffer fills up first, it drops everything until the next `0x00`. That reset on every `0x00` is how the parser resyncs after garbage.
4. `qp_cobs_decode()` undoes COBS, then `handle_frame()` runs.

## Frame format

```
COBS( type u8 | seq u16 | len u8 | payload | crc16 u16 ) 0x00     little-endian
```

COBS replaces every `0x00` in the frame with the distance to the next zero, so `0x00` only appears as the end marker. Example, an ACK frame:

```
raw : 83 07 00 04 03 2A 00 00 53 4E
wire: 03 83 07 | 04 04 03 2A | 01 | 03 53 4E | 00
```

Each block starts with a code byte `c`: copy `c-1` bytes, then add a `0x00` (not after a `0xFF` block or at the end).

The CRC is CRC16-CCITT (`0x1021`, init `0xFFFF`) over type through payload. USB already has its own CRC, so this one catches software problems: a tty not in raw mode, or two programs writing to one port.

Largest frame: 4 + 255 + 2 = 261 raw bytes, 263 after COBS. Anything longer is garbage.

## handle_frame

Cheap checks first. Nothing in the header is trusted until the CRC passes.

1. At least 6 bytes (header + CRC).
2. CRC. On failure drop silently. No NACK, since type and seq can't be trusted.
3. `header.len` matches the real length.
4. Count seq gaps. `gap = seq - last - 1` (u16). Only `0 < gap < 0x8000` counts, so a resend or a host restart is not counted as loss.
5. Find the channel by type (linear search). Unknown type always gets `ACK UNKNOWN_TYPE`.
6. Payload length must equal `sizeof(TYPE)`, else `BAD_LEN`.
7. Dedupe, ACK channels only, within 1 s (`QP_DEDUPE_MS`) of the last frame on that channel. `diff = (int16_t)(seq - last_seq)`:
   - `diff == 0`: the host missed our ACK and resent. Reply with the saved result, don't store again.
   - `diff < 0`: a late resend of an older command. A newer one is already stored, so reply `STALE` and drop it.

   After 1 s without a frame the check is skipped, so a restarted host (new seq) is always accepted. The signed difference makes the u16 wrap harmless.
8. Run `validate()` if there is one, then store and ACK.

The payload starts at `raw[4]`, which is not 4-byte aligned. An unaligned float load (`VLDR`) HardFaults on the M7, so always `memcpy` the payload into a local struct. Never cast the pointer.

## Channels

Each message in `QP_MESSAGES` becomes one channel: a storage buffer of `sizeof(TYPE)`, a `version` counter and a timestamp. It holds only the latest value.

```c
// Core/Src/usb/usb_proto.c (simplified)
// write, UsbTask                      // read, any task: qp_update()
taskENTER_CRITICAL();                  taskENTER_CRITICAL();
memcpy(ch->storage, payload, len);     if (ch->version != *last_version) {
ch->stamp_ms = now;                      memcpy(out, ch->storage, len);
ch->version++;                           *last_version = ch->version;
taskEXIT_CRITICAL();                   }
                                       taskEXIT_CRITICAL();
```

- The lock is needed because ControlTask has higher priority and can preempt UsbTask in the middle of a write. A 144 B copy takes under 1 µs.
- A slow reader skips versions (1, 3, 5) but always gets the newest one.
- `!=` instead of `>` makes the u32 wrap harmless.
- `qp_age_ms()` takes no lock: two aligned 32-bit reads are each atomic on the M7. It returns `UINT32_MAX` if the channel was never written, so "no command yet" already looks like a timeout.
- Task context only. In an ISR the critical section would need the `FromISR` variant.

## X-macro

`QP_MESSAGES(X)` is a list. Passing a different `X` generates different code from the same list:

```c
// Core/Inc/usb/usb_msgs.h: one line of the list...
X(CMD_JOINTS, 0x01, qp_cmd_joints_t, false, validate_joints)

// ...and what it expands to

enum { QP_CMD_JOINTS = 0x01 };                       // QP_X_ENUM, usb_msgs.h
extern struct qp_channel qp_ch_CMD_JOINTS;           // QP_X_EXTERN, usb_msgs.h
static uint8_t CMD_JOINTS_storage[144];              // QP_X_CHANNEL, usb_msgs.c
qp_channel_t qp_ch_CMD_JOINTS = { .type = 0x01, .len = 144, ... };
qp_channel_t *const qp_channels[] = { &qp_ch_CMD_JOINTS, ... };   // QP_X_PTR
switch (id) { case 0x01: break; ... }                // QP_X_CASE: duplicate ID = compile error
```

`_Static_assert` also rejects a payload over 255 B or an ID outside `0x01`–`0x7F`.

## Transmit

`qp_send()` builds the header with the STM32's own `tx_seq`, appends the CRC, COBS-encodes and adds `0x00`. If the TX FIFO can't hold the whole frame it sends nothing and counts `tx_drops`. A half frame would merge with the next one on the host side.

## Errors

| Case | Counter | ACK | Stored |
|---|---|---|---|
| No `0x00` before buffer full | `overflows` | no | no |
| Bad COBS, or shorter than 6 B | `cobs_errors` | no | no |
| CRC mismatch | `crc_errors` | no | no |
| `header.len` wrong | `bad_len` | no | no |
| Unknown type | `unknown_type` | always, `UNKNOWN_TYPE` | no |
| Payload size wrong | `bad_len` | ACK channels, `BAD_LEN` | no |
| Validate failed | `bad_arg` | ACK channels, `BAD_ARG` | no |
| Resent seq | `dup_frames` | previous result | no (already stored) |
| Older seq | `stale_frames` | `STALE` | no |
| OK | `rx_frames` | ACK channels, `OK` | yes |
| TX FIFO full | `tx_drops` | lost | yes, host resend hits dedupe |

## Design choices

- TinyUSB 0.21.0, vendored in `Middlewares/Third_Party/tinyusb/`. Same code on Nucleo and PCB, and composite CDC is simple. Upgrades are manual, see `UPSTREAM.md`.
- 2x CDC (commands, log). No driver needed on macOS, Linux or Jetson, and text logs don't mix with binary frames.
- No VBUS sensing. `tud_mounted()` may not notice an unplug, so safety has to come from `qp_age_ms()` timeouts.
- One seq counter for all host messages. Enough for loss counting and ACK matching. Hosts must not keep one counter per message.
- ACK only for rare commands that must arrive (gains, mode). Setpoints are not ACKed; the next one replaces a lost one.
- One message carries all 12 joints. The controller never sees half a new pose. 144 B at 200 Hz is about 29 KB/s, well within full speed.
- Latest value, not a queue. Old setpoints are useless. A message where every value matters would need a queue.
- Copy under a critical section instead of sharing pointers. Double buffering or a seqlock isn't worth it at this size.

## Known limits

- A host that restarts within 1 s of its last ACKed message can get `STALE` on its first one (if its new seq happens to be lower). `quads_host.py` starts at a random seq.
- `qp_stats_t` is not locked. Fine for debugging, don't use it for control decisions.
- Only UsbTask can send. STATE from ControlTask needs a TX channel that UsbTask drains (planned).
