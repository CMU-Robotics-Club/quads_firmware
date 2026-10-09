#!/usr/bin/env python3
"""Start OpenOCD RTT after flashing, then stream channel 0 to stdout."""
import argparse
import socket
import sys
import time


class OpenOCD:
    """OpenOCD Tcl RPC: UTF-8 commands/responses delimited by SUB (0x1a)."""

    def __init__(self, host, port):
        self.socket = socket.create_connection((host, port), timeout=3)
        self.pending = b""

    def close(self):
        self.socket.close()

    def command(self, command):
        script = ('set status [catch {' + command +
                  '} result]; format "%d\\n%s" $status $result')
        self.socket.sendall(script.encode() + b"\x1a")
        while b"\x1a" not in self.pending:
            chunk = self.socket.recv(4096)
            if not chunk:
                raise ConnectionError("OpenOCD closed its control connection")
            self.pending += chunk
        response, self.pending = self.pending.split(b"\x1a", 1)
        status, _, result = response.decode(errors="replace").partition("\n")
        if status != "0":
            raise RuntimeError(result or response.decode(errors="replace"))
        return result


def start_rtt(control, port, wait_seconds):
    control.command('rtt setup 0x24000000 8192 "SEGGER RTT"')
    control.command("rtt stop")
    deadline = time.monotonic() + wait_seconds
    while True:
        try:
            control.command("rtt start")
            # OpenOCD 0.12 returns success even when no control block is found.
            control.command("set channels [rtt channellist]; "
                            "if {[llength [lindex $channels 0]] < 1} "
                            "{error {RTT output channel not ready}}")
            break
        except RuntimeError as error:
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "RTT not ready: flash/run the RTT firmware and retry. "
                    f"OpenOCD: {error}"
                ) from error
            time.sleep(0.25)
            control.command("rtt stop")
    # 0.12 registers a poll timer when changing this value: do it only once ready.
    control.command("rtt polling_interval 10")
    # Re-running this helper recreates its server after reset/reflash.
    try:
        control.command(f"rtt server stop {port}")
    except RuntimeError:
        pass  # First launch: no server exists yet.
    control.command(f"rtt server start {port} 0")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--control-port", type=int, default=6666)
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--wait", type=float, default=10,
                        help="seconds to wait for RTT initialization (default: 10)")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535 or not 1 <= args.control_port <= 65535:
        parser.error("ports must be between 1 and 65535")
    if args.wait < 0:
        parser.error("--wait must be nonnegative")
    control = None
    try:
        control = OpenOCD(args.host, args.control_port)
        start_rtt(control, args.port, args.wait)
        print("RTT channel 0 connected. Ctrl-C exits; restart after reset/reflash.",
              file=sys.stderr)
        with socket.create_connection((args.host, args.port), timeout=3) as stream:
            stream.settimeout(None)
            while True:
                data = stream.recv(4096)
                if not data:
                    raise ConnectionError("RTT stream closed; restart this helper")
                sys.stdout.buffer.write(data)
                sys.stdout.buffer.flush()
    except KeyboardInterrupt:
        return 0
    except (OSError, RuntimeError) as error:
        print(f"RTT: {error}. Ensure OpenOCD is running and firmware is initialized.",
              file=sys.stderr)
        return 1
    finally:
        if control is not None:
            control.close()


if __name__ == "__main__":
    sys.exit(main())
