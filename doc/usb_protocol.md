# USB protocol (Jetson ↔ STM32)

The host talks to the STM32 over the **user USB port**: CN13 on the Nucleo, J1 on the PCB. The ST-Link is not involved.
The STM32 shows up as 2 serial ports: **CDC0 = commands** (`/dev/cu.usbmodem…1`, `/dev/ttyACM0`) and **CDC1 = log**.

All USB code lives in `Core/Src/usb/` and `Core/Inc/usb/`. To add a message you only touch `usb_msgs.h`, `usb_msgs.c`, the task that uses it, and `scripts/quads_host.py`.

## Tutorial: add a new command

Goal: the Jetson sends **`CMD_MODE`** (0 = idle, 1 = stand, 2 = walk). The STM32 must ACK it, because a lost mode change matters.

### Step 1: define the payload

```c
// Core/Inc/usb/usb_msgs.h
typedef struct __attribute__((packed)) {
  uint8_t mode;               // 0 idle, 1 stand, 2 walk
} qp_cmd_mode_t;
```

Always mark the struct `packed`. Use fixed-width types and `float` (f32). Byte order is little-endian, which is the native order on both sides.
Each message keeps **one latest value**. If the data is per joint, put an array of all 12 joints in the struct, as `qp_cmd_joints_t` does, so the joints always update together.

### Step 2: register it (one line)

```c
// Core/Inc/usb/usb_msgs.h
#define QP_MESSAGES(X)                                                          \
  /* NAME        ID    TYPE             ACK    VALID          */     \
  X(CMD_JOINTS,  0x01, qp_cmd_joints_t, false, validate_joints)     \
  X(CMD_GAINS,   0x03, qp_cmd_gains_t,  true,  validate_gains)      \
  X(CMD_MODE,    0x02, qp_cmd_mode_t,   true,  validate_mode)
```

| Column | What to put |
|---|---|
| `NAME` | Any name. You get the channel `qp_ch_CMD_MODE` and the ID `QP_CMD_MODE` |
| `ID` | An unused ID in `0x01`–`0x7F`. A duplicate is a compile error |
| `TYPE` | The struct from step 1. Frames with a different length are rejected |
| `ACK` | `true` for rare commands that must arrive (mode, gains). `false` for high-rate setpoints, because the next one replaces a lost one |
| `VALID` | Name of a validate function, or `NULL` |

### Step 3: validate it (optional)

```c
// Core/Src/usb/usb_msgs.c
static uint8_t validate_mode(const void *payload)
{
  qp_cmd_mode_t c;
  memcpy(&c, payload, sizeof(c));         // payload is unaligned: always memcpy
  return (c.mode <= 2U) ? QP_OK : QP_ERR_BAD_ARG;
}
```

A rejected frame is not stored, and the host gets `ACK err=BAD_ARG`.

### Step 4: read it in firmware (any task)

```c
// your task, e.g. Core/Src/usb/usb_demo.c
#include "usb_proto.h"

static uint32_t mode_ver = 0;                         // one per channel you read
qp_cmd_mode_t m;
if (qp_update(&qp_ch_CMD_MODE, &m, &mode_ver) > 0) {
  // a new mode arrived; m is your own copy
}
```

`qp_update()` returns `1` for new data and `0` when nothing changed. Call it from tasks only, never from ISRs. Never call `tud_*` from your task. `qp_age_ms(&qp_ch_X)` tells you how long ago the last value arrived, which you can use for timeouts.

### Step 5: send it from the host

Mirror the ID and struct, then add a subcommand:

```python
# scripts/quads_host.py
QP_CMD_MODE = 0x02
CMD_MODE = struct.Struct("<B")      # < little-endian | B u8  H u16  I u32  f float

def cmd_mode(link, args):
    seq = link.send(QP_CMD_MODE, CMD_MODE.pack(args.mode))
    ack = link.wait_ack(seq)        # None after 100 ms -> resend with the SAME seq
    print(f"ACK err={QP_ERR.get(ack[1])}" if ack else "no ACK")

# in main():
m = sub.add_parser("mode"); m.add_argument("mode", type=int)
# and add "mode": cmd_mode to the dispatch dict
```

A C++ / ROS 2 host does the same thing, see "Rules for host code" below.

### Step 6: test it

```bash
# terminal, from the repo root
python3 scripts/quads_host.py mode 1     # expect: ACK err=OK
python3 scripts/quads_host.py mode 9     # expect: ACK err=BAD_ARG
```

To see the value on the STM32, print it from your task (as `usb_demo.c` does) and watch RTT.

**Checklist:** struct → `QP_MESSAGES` line → (validate) → `qp_update` in your task → Python ID + struct → test.

## Use the link from your own Python script

`quads_host.py` is also a small library. Put your script in `scripts/` and import it.

**Send your own data.** `CMD_JOINTS` is 36 floats: joint `j` uses index `3*j` (`q_des`), `3*j+1` (`qd_des`), `3*j+2` (`tau_ff`).

```python
# scripts/my_test.py
from quads_host import QuadsLink, find_port, QP_CMD_JOINTS, CMD_JOINTS, QP_MAX_JOINTS

link = QuadsLink(find_port())
vals = [0.0] * (3 * QP_MAX_JOINTS)
vals[3 * 2 + 0] = 0.3                     # joint 2: q_des = 0.3 rad
link.send(QP_CMD_JOINTS, CMD_JOINTS.pack(*vals))   # send this at 50–200 Hz in a loop
link.close()
```

**Receive data from the STM32.** Register a handler for the message type. It runs on the rx thread, so keep it short.

```python
# scripts/my_test.py
import struct
STATE = struct.Struct("<" + "fff" * QP_MAX_JOINTS + "B")  # 12 x (q, qd, tau_cmd), sys_state
link.on(0x84, lambda p: print(STATE.unpack(p)[0], STATE.unpack(p)[-1]))  # joint 0 q, sys_state
```

The STM32 sends `ACK` and `STATE`. `qp_send()` is UsbTask-only: a firmware task that wants to send data hands it to UsbTask (see `usb_state.c` for the pattern).

## Existing messages

| ID | Name | Payload | ACK |
|---|---|---|---|
| `0x01` | `CMD_JOINTS` | 12 × (`q_des`, `qd_des`, `tau_ff`) f32 = 144 B | no |
| `0x03` | `CMD_GAINS` | 12 × (`kp`, `kd`, `tau_max`) f32 = 144 B | yes |
| `0x83` | `ACK` (STM32 → host) | `ack_type u8, ack_seq u16, err u8` | — |
| `0x84` | `STATE` (STM32 → host) | 12 × (`q`, `qd`, `tau_cmd`) f32 + `sys_state` u8 = 145 B, every 10 ms while the CMD port is open | — |

`STATE` notes:
- `tau_cmd` is the torque the PD loop commanded (after the clamp), not a measurement. With `PD_DRY_RUN 1` it is computed but not sent to the ODrives.
- Joint values are 0 until `RUNNING` and freeze after `FAULT`.
- `sys_state`: 0 INIT, 1 WAIT_CAN, 2 CLEAR_ERRORS, 3 ARM_MOTORS, 4 CALIBRATE_OFFSETS, 5 RUNNING, 6 FAULT.

## Rules for host code

- Frame: `COBS( type u8 | seq u16 | len u8 | payload | crc16 u16 ) 0x00`, little-endian, CRC16-CCITT (`0x1021`, init `0xFFFF`).
- Use one seq counter for every message you send. Start it at a random value, so a restarted program doesn't reuse old seqs.
- For ACKed messages, wait for the ACK. If none comes within about 100 ms, resend with the same seq. `err`: 0 OK, 1 UNKNOWN_TYPE, 2 BAD_LEN, 3 BAD_ARG, 4 STALE (a newer command was already applied).
- Setpoints are not ACKed. Keep sending them; if they stop, `qp_age_ms()` grows and the controller holds.
- On Linux open the port in raw mode (`cfmakeraw`), or the tty layer rewrites bytes.

Internals (parser, locking, error handling) are in [usb_implementation.md](usb_implementation.md).

## Test the existing messages

```bash
# terminal, from the repo root
pip install pyserial
python3 scripts/quads_host.py gains    # CMD_GAINS: ACK err=OK, rtt < 1 ms
python3 scripts/quads_host.py bad      # malformed frames, each prints its expected result
python3 scripts/quads_host.py joints   # 100 Hz CMD_JOINTS, RTT prints "JOINTS ver=... j0 q=..."
```
