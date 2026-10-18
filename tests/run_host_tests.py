#!/usr/bin/env python3
"""Run real RTT/backend code with mocked CMSIS registers, plus helper tests."""
import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("rtt_terminal", ROOT / "scripts/rtt_terminal.py")
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)


class FakeSocket:
    def __init__(self, fragments):
        self.fragments = iter(fragments)
        self.sent = []

    def sendall(self, data):
        self.sent.append(data)

    def recv(self, _size):
        return next(self.fragments, b"")


class HelperTests(unittest.TestCase):
    def control(self, fragments):
        control = helper.OpenOCD.__new__(helper.OpenOCD)
        control.socket = FakeSocket(fragments)
        control.pending = b""
        return control

    def test_fragmented_rpc_and_error(self):
        control = self.control([b"0\n", b"ok\x1a1\nnot ready\x1a"])
        self.assertEqual(control.command("rtt start"), "ok")
        self.assertTrue(control.socket.sent[0].endswith(b"\x1a"))
        self.assertIn(b"catch {rtt start}", control.socket.sent[0])
        with self.assertRaisesRegex(RuntimeError, "not ready"):
            control.command("rtt start")

    def test_closed_connection(self):
        with self.assertRaises(ConnectionError):
            self.control([]).command("rtt start")

    def test_start_sequence_and_missing_server(self):
        commands = []

        class Control:
            def command(self, cmd):
                commands.append(cmd)
                if cmd == "rtt server stop 9090":
                    raise RuntimeError("no server")

        helper.start_rtt(Control(), 9090, 0)
        self.assertEqual(commands, ['rtt setup 0x24000000 8192 "SEGGER RTT"',
                         "rtt stop", "rtt start",
                         "set channels [rtt channellist]; "
                         "if {[llength [lindex $channels 0]] < 1} "
                         "{error {RTT output channel not ready}}",
                         "rtt polling_interval 10",
                         "rtt server stop 9090", "rtt server start 9090 0"])

    def test_uninitialized_error(self):
        class Control:
            def command(self, cmd):
                if "rtt channellist" in cmd:
                    raise RuntimeError("control block not found")

        with self.assertRaisesRegex(RuntimeError, "flash/run"):
            helper.start_rtt(Control(), 9090, 0)


if __name__ == "__main__":
    with tempfile.TemporaryDirectory(prefix="quads-rtt-test-") as directory:
        binary = Path(directory) / "test_rtt"
        subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-Itests/stubs", "-ICore/Inc",
                        "-IMiddlewares/Third_Party/SEGGER_RTT",
                        "tests/test_rtt_log.c", "Core/Src/rtt_log.c",
                        "Middlewares/Third_Party/SEGGER_RTT/SEGGER_RTT.c",
                        "-o", str(binary)], cwd=ROOT, check=True)
        subprocess.run([str(binary)], check=True)
    unittest.main()
