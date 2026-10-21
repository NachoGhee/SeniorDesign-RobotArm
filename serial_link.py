import time

import serial

from gcode_formatter import format_joint_gcode
from telemetry_parser import parse_telemetry_line


class SerialConnection:
    def __init__(self, port_name, baud_rate=115200):
        self.port = serial.Serial(port_name, baud_rate, timeout=0)
        self.receive_buffer = bytearray()

    def _read_lines(self):
        self.receive_buffer.extend(self.port.read(self.port.in_waiting))
        while b"\n" in self.receive_buffer:
            raw_line, _, remainder = self.receive_buffer.partition(b"\n")
            self.receive_buffer[:] = remainder
            yield raw_line.decode("ascii", errors="replace").strip()

    def read_telemetry(self):
        latest = None
        for line in self._read_lines():
            telemetry = parse_telemetry_line(line)
            if telemetry is not None:
                latest = telemetry
            elif line.startswith(("ERR,", "FAULT,")):
                print(line)
        return latest

    def send_command(self, command):
        self.port.write((command + "\n").encode("ascii"))

    def send_gcode_line(self, command, timeout_s=2.0):
        self.send_command(command)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            for line in self._read_lines():
                telemetry = parse_telemetry_line(line)
                if telemetry is not None:
                    if telemetry["faults"]:
                        raise RuntimeError(f"Encoder fault mask: 0x{telemetry['faults']:02X}")
                elif line == "ok" or line.startswith("OK,"):
                    return
                elif line.startswith(("ERR,", "FAULT,")):
                    raise RuntimeError(line)
            time.sleep(0.005)
        raise TimeoutError(f"Timed out waiting for acknowledgement to: {command}")

    def run_gcode_stream(self, source):
        for raw_line in source:
            command = raw_line.split(";", 1)[0].strip()
            if not command:
                continue
            self.send_gcode_line(command)
            print(f"ok: {command}")
            if command.upper() == "M112":
                return
        self.send_gcode_line("M18")

    def close(self):
        self.port.close()