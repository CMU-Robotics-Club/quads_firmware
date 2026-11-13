# USB protocol (Jetson ↔ STM32)

The host talks to the STM32 over the **user USB port**: CN13 on the Nucleo, J1 on the PCB. The ST-Link is not involved.
The STM32 shows up as 2 serial ports: **CDC0 = commands** (`/dev/cu.usbmodem…1`, `/dev/ttyACM0`) and **CDC1 = log**.

## Add a new message

Example: `CMD_MODE`, which sets the robot mode.

**1. Firmware, `Core/Inc/quads_msgs.h`.** Add a struct and one line:

```c
typedef struct __attribute__((packed)) {
  uint8_t mode;               // 0 idle, 1 stand, 2 walk
} qp_cmd_mode_t;

#define QP_MESSAGES(X)                                                       \
  /* NAME      ID    TYPE            SLOTS          ACK    VALID        */   \
  X(CMD_JOINT, 0x01, qp_cmd_joint_t, QP_MAX_JOINTS, false, validate_joint)   \
  X(CMD_GAINS, 0x03, qp_cmd_gains_t, QP_MAX_JOINTS, true,  validate_gains)   \
  X(CMD_MODE,  0x02, qp_cmd_mode_t,  1,             true,  NULL)
```

| Column | Meaning |
|---|---|
| `NAME` | Creates the channel `qp_ch_CMD_MODE` |
| `ID` | Wire ID, `0x01`–`0x7F`. A duplicate is a compile error |
| `TYPE` | Payload struct. A frame with the wrong length is rejected |
| `SLOTS` | `1` = keep one latest value. `N` = one latest value per key, where the key is payload byte 0 (e.g. `joint_id`) |
| `ACK` | `true` for rare, must-arrive commands. `false` for high-rate setpoints |
| `VALID` | Optional `uint8_t fn(const void *p)` in `quads_msgs.c` returning `QP_OK` / `QP_ERR_BAD_ARG` |

**2. Firmware, read it from any task:**

```c
static uint32_t ver = 0;
qp_cmd_mode_t m;
if (qp_update(&qp_ch_CMD_MODE, 0, &m, &ver) > 0) { /* new value in m */ }
qp_age_ms(&qp_ch_CMD_MODE, 0);   // ms since last update, for timeouts
```

Call it from tasks only, never from ISRs. You always get the latest value. See `Core/Src/usb_demo.c`.

**3. Host, `scripts/quads_host.py`.** Mirror the struct:

```python
QP_CMD_MODE = 0x02
CMD_MODE = struct.Struct("<B")      # < = little-endian; B u8, H u16, I u32, f float
seq = link.send(QP_CMD_MODE, CMD_MODE.pack(1))
ack = link.wait_ack(seq)            # None after 100 ms -> resend with the SAME seq
```

## How it works

```
COBS( type u8 | seq u16 | len u8 | payload | crc16 u16 )  0x00      (little-endian)
```

- **COBS:** USB CDC is a byte stream, so frames need a boundary. `0x00` appears only at the end of a frame, so after any garbage the parser resyncs at the next `0x00`.
- **CRC16-CCITT** (`0x1021`, init `0xFFFF`): catches software corruption, for example a Linux tty not in raw mode, or two writers on one port. Bad frames are dropped.
- **seq:** counts lost frames and matches ACKs. A repeated seq on an ACKed message is re-ACKed but not applied twice.
- **ACK** `0x83`: `ack_type u8, ack_seq u16, err u8`. `err` is 0 OK, 1 UNKNOWN_TYPE, 2 BAD_LEN, or 3 BAD_ARG.

```
USB IRQ → UsbTask: parse → validate → copy into channel (critical section) → ACK
                                                  ↑
                         ControlTask / UsbDemoTask: qp_update() copies out
```

Readers copy the struct instead of sharing a pointer. ControlTask can preempt UsbTask in the middle of a write, and a shared pointer would let it read half-updated data.
Setpoints have no ACK. If they stop arriving, `qp_age_ms()` grows and the controller should hold position.

## Test

```bash
pip install pyserial
python3 scripts/quads_host.py gains    # ACK err=OK, rtt < 1 ms
python3 scripts/quads_host.py bad      # malformed frames, each prints its expected result
python3 scripts/quads_host.py joint    # 100 Hz stream, RTT prints "CMD_JOINT j0 ..."
```
