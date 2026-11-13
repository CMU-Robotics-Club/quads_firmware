#!/usr/bin/env python3
"""
@file quads_host.py
@brief Host-side test tool for the Quads USB protocol (CDC0). Works on macOS and Linux.
@author Ivan Lok

Mirrors Core/Inc/quads_proto.h (framing) and Core/Inc/quads_msgs.h (messages). Needs: pip install pyserial

Examples:
  python3 scripts/quads_host.py joint --rate 100 --seconds 60   # stream CMD_JOINT (sine on joint 0)
  python3 scripts/quads_host.py gains --joint 0 --kp 20 --kd 0.5 --tau-max 5
  python3 scripts/quads_host.py bad                              # corrupted / malformed frames
  python3 scripts/quads_host.py --port /dev/ttyACM0 joint        # explicit port
"""
import argparse
import math
import struct
import sys
import threading
import time

# ---- protocol (keep in sync with quads_msgs.h) ----
QP_CMD_JOINT = 0x01
QP_CMD_GAINS = 0x03
QP_ACK = 0x83

QP_ERR = {0: "OK", 1: "UNKNOWN_TYPE", 2: "BAD_LEN", 3: "BAD_ARG"}

HDR = struct.Struct("<BHB")         # type, seq, len
CMD_JOINT = struct.Struct("<Bfff")  # joint_id, q_des, qd_des, tau_ff
CMD_GAINS = struct.Struct("<Bfff")  # joint_id, kp, kd, tau_max
ACK = struct.Struct("<BHB")         # ack_type, ack_seq, err

USB_VID, USB_PID = 0xCAFE, 0x4C51


def crc16(data: bytes) -> int:
    """CRC16-CCITT: poly 0x1021, init 0xFFFF, no reflect, no xorout."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])
    code_idx, code = 0, 1
    for b in data:
        if b == 0:
            out[code_idx] = code
            code_idx, code = len(out), 1
            out.append(0)
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx, code = len(out), 1
                out.append(0)
    out[code_idx] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        i += 1
        if code == 0 or i + code - 1 > len(data):
            raise ValueError("bad COBS")
        out += data[i:i + code - 1]
        i += code - 1
        if code != 0xFF and i < len(data):
            out.append(0)
    return bytes(out)


def build_frame(msg_type: int, seq: int, payload: bytes) -> bytes:
    body = HDR.pack(msg_type, seq & 0xFFFF, len(payload)) + payload
    raw = body + struct.pack("<H", crc16(body))
    return cobs_encode(raw) + b"\x00"


def parse_frame(encoded: bytes):
    """Return (type, seq, payload) or None if invalid."""
    try:
        raw = cobs_decode(encoded)
    except ValueError:
        return None
    if len(raw) < HDR.size + 2:
        return None
    body, crc_rx = raw[:-2], struct.unpack("<H", raw[-2:])[0]
    if crc16(body) != crc_rx:
        return None
    msg_type, seq, length = HDR.unpack(body[:HDR.size])
    payload = body[HDR.size:]
    if length != len(payload):
        return None
    return msg_type, seq, payload


# ---- link ----
class QuadsLink:
    """One CDC0 connection: send frames, background thread collects ACKs."""

    def __init__(self, port: str):
        import serial  # pyserial
        self.ser = serial.Serial(port, timeout=0.05)
        self.seq = 0
        self.acks = {}               # ack_seq -> (ack_type, err, t_rx)
        self.ack_cv = threading.Condition()
        self.rx_bad = 0
        self._stop = False
        self._t = threading.Thread(target=self._rx_loop, daemon=True)
        self._t.start()

    def close(self):
        self._stop = True
        self._t.join(timeout=1)
        self.ser.close()

    def send(self, msg_type: int, payload: bytes) -> int:
        seq = self.seq
        self.ser.write(build_frame(msg_type, seq, payload))
        self.seq = (self.seq + 1) & 0xFFFF
        return seq

    def send_raw(self, data: bytes):
        self.ser.write(data)

    def wait_ack(self, seq: int, timeout: float = 0.1):
        deadline = time.monotonic() + timeout
        with self.ack_cv:
            while seq not in self.acks:
                left = deadline - time.monotonic()
                if left <= 0:
                    return None
                self.ack_cv.wait(left)
            return self.acks.pop(seq)

    def _rx_loop(self):
        buf = bytearray()
        while not self._stop:
            # read(256) would block until 256 bytes or the 50 ms timeout -> fake 50 ms RTT.
            # Read whatever is waiting (at least 1 byte) so ACKs are seen immediately.
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if not chunk:
                continue
            for b in chunk:
                if b != 0:
                    buf.append(b)
                    continue
                frame = parse_frame(bytes(buf))
                buf.clear()
                if frame is None:
                    self.rx_bad += 1
                    continue
                msg_type, _seq, payload = frame
                if msg_type == QP_ACK and len(payload) == ACK.size:
                    ack_type, ack_seq, err = ACK.unpack(payload)
                    with self.ack_cv:
                        self.acks[ack_seq] = (ack_type, err, time.monotonic())
                        self.ack_cv.notify_all()


def find_port() -> str:
    """Pick the CMD port (CDC0) of the first Quads MCU found."""
    from serial.tools import list_ports
    ports = [p for p in list_ports.comports() if p.vid == USB_VID and p.pid == USB_PID]
    if not ports:
        sys.exit("No Quads MCU found (VID 0xCAFE PID 0x4C51). Use --port.")
    # CDC0 is interface 0: on macOS the name ends in ...1, on Linux it is the lower ttyACM.
    ports.sort(key=lambda p: p.device)
    return ports[0].device


# ---- commands ----
def cmd_joint(link: QuadsLink, args):
    period = 1.0 / args.rate
    n = int(args.seconds * args.rate)
    t0 = time.monotonic()
    late = 0
    for i in range(n):
        t = i * period
        q = args.amp * math.sin(2 * math.pi * args.freq * t)
        qd = args.amp * 2 * math.pi * args.freq * math.cos(2 * math.pi * args.freq * t)
        link.send(QP_CMD_JOINT, CMD_JOINT.pack(args.joint, q, qd, 0.0))
        target = t0 + (i + 1) * period
        sleep = target - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)
        else:
            late += 1
        if i % max(1, args.rate // 4) == 0:  # 4 Hz display (1 Hz would always hit sine zeros)
            print(f"\rsent {i + 1}/{n}  q={q:+.3f} rad", end="", flush=True)
    dt = time.monotonic() - t0
    print(f"\nsent {n} CMD_JOINT in {dt:.2f}s ({n / dt:.1f} Hz), late ticks: {late}")
    print("Check RTT on the STM32: 'CMD_JOINT j0 ver=...' every 200 ms, q following the sine.")


def cmd_gains(link: QuadsLink, args):
    payload = CMD_GAINS.pack(args.joint, args.kp, args.kd, args.tau_max)
    t_tx = time.monotonic()
    seq = link.send(QP_CMD_GAINS, payload)
    for attempt in range(3):
        if attempt > 0:  # resend with the SAME seq: firmware dedupes and only re-ACKs
            t_tx = time.monotonic()
            link.send_raw(build_frame(QP_CMD_GAINS, seq, payload))
        ack = link.wait_ack(seq)
        if ack:
            ack_type, err, t_rx = ack
            print(f"ACK seq={seq} type=0x{ack_type:02X} err={QP_ERR.get(err, err)} "
                  f"rtt={(t_rx - t_tx) * 1000:.2f} ms")
            return
        print(f"no ACK for seq={seq}, resending ({attempt + 1}/3)")
    sys.exit("CMD_GAINS failed: no ACK")


def cmd_bad(link: QuadsLink, args):
    good = build_frame(QP_CMD_GAINS, 0, CMD_GAINS.pack(0, 1.0, 0.1, 1.0))
    cases = []
    # 1. flipped bit -> CRC error, dropped silently
    raw = bytearray(cobs_decode(good[:-1]))
    raw[5] ^= 0x01
    cases.append(("bad CRC (expect: no ACK, crc_errors++)", cobs_encode(bytes(raw)) + b"\x00", None))
    # 2. unknown type -> ACK UNKNOWN_TYPE
    cases.append(("unknown type 0x7E (expect ACK UNKNOWN_TYPE)", None, (0x7E, b"\x01\x02")))
    # 3. wrong length for CMD_GAINS -> ACK BAD_LEN
    cases.append(("CMD_GAINS short payload (expect ACK BAD_LEN)", None, (QP_CMD_GAINS, b"\x00\x01")))
    # 4. NaN -> ACK BAD_ARG
    cases.append(("CMD_GAINS kp=NaN (expect ACK BAD_ARG)", None,
                  (QP_CMD_GAINS, CMD_GAINS.pack(0, float("nan"), 0.1, 1.0))))
    # 5. garbage without delimiter then a good frame -> good frame must still work
    cases.append(("300 bytes garbage, then valid CMD_GAINS (expect ACK OK)", b"\x55" * 300 + b"\x00",
                  (QP_CMD_GAINS, CMD_GAINS.pack(0, 1.0, 0.1, 1.0))))

    for name, raw_bytes, msg in cases:
        if raw_bytes:
            link.send_raw(raw_bytes)
        if msg is None:
            time.sleep(0.05)
            print(f"[sent] {name}")
            continue
        seq = link.send(*msg)
        ack = link.wait_ack(seq)
        result = f"err={QP_ERR.get(ack[1], ack[1])}" if ack else "no ACK"
        print(f"[{result}] {name}")


def main():
    ap = argparse.ArgumentParser(description="Quads USB protocol test tool")
    ap.add_argument("--port", help="CDC0 serial port (default: auto-detect)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    j = sub.add_parser("joint", help="stream CMD_JOINT")
    j.add_argument("--joint", type=int, default=0)
    j.add_argument("--rate", type=int, default=100, help="Hz")
    j.add_argument("--seconds", type=float, default=10)
    j.add_argument("--amp", type=float, default=0.5, help="rad")
    j.add_argument("--freq", type=float, default=0.5, help="Hz of the sine")

    g = sub.add_parser("gains", help="send CMD_GAINS and wait for ACK")
    g.add_argument("--joint", type=int, default=0)
    g.add_argument("--kp", type=float, default=20.0)
    g.add_argument("--kd", type=float, default=0.5)
    g.add_argument("--tau-max", type=float, default=5.0)

    sub.add_parser("bad", help="send malformed frames")

    args = ap.parse_args()
    link = QuadsLink(args.port or find_port())
    try:
        {"joint": cmd_joint, "gains": cmd_gains, "bad": cmd_bad}[args.cmd](link, args)
    finally:
        link.close()


if __name__ == "__main__":
    main()
