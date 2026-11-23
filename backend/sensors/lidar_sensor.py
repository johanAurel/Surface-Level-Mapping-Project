"""TFmini LiDAR sensor wrapper.

PSEUDOCODE:
    CLASS LidarSensor(port, baudrate):
        connect(): OPEN serial port lazily
        read():
            READ bytes until TFmini frame header 0x59 0x59 found
            VALIDATE checksum
            PARSE distance/strength
            RETURN LidarReading(distance_m, strength, quality, valid)
        close(): CLOSE serial port
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

try:
    import serial
except Exception:  # pragma: no cover - hardware dependency
    serial = None


@dataclass
class LidarReading:
    distance_m: Optional[float]
    strength: Optional[int] = None
    quality: Optional[int] = None
    valid: bool = False
    raw_cm: Optional[int] = None


class LidarSensor:
    """Read distance measurements from a TFmini over serial."""

    FRAME_HEADER = 0x59
    FRAME_LENGTH = 9

    def __init__(self, port: str = "/dev/ttyAMA0", baudrate: int = 115200, timeout: float = 1.0):
        # PSEUDOCODE: store connection config; do not open hardware yet.
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial = None

    def connect(self) -> None:
        # PSEUDOCODE: open serial device once; reuse existing open connection.
        if self._serial and self._serial.is_open:
            return
        if serial is None:
            raise RuntimeError("pyserial is required for TFmini LiDAR communication")
        self._serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)

    def read(self) -> LidarReading:
        # PSEUDOCODE: synchronize to frame header, parse valid frame, return structured reading.
        self.connect()
        assert self._serial is not None

        buffer = bytearray()
        while True:
            chunk = self._serial.read(self._serial.in_waiting or 1)
            if not chunk:
                return LidarReading(distance_m=None, valid=False)
            buffer.extend(chunk)
            idx = buffer.find(bytes([self.FRAME_HEADER, self.FRAME_HEADER]))
            if idx == -1:
                # PSEUDOCODE: keep last byte in case header splits across reads.
                buffer = buffer[-1:]
                continue
            if len(buffer) < idx + self.FRAME_LENGTH:
                continue
            frame = bytes(buffer[idx : idx + self.FRAME_LENGTH])
            del buffer[: idx + self.FRAME_LENGTH]
            if not self._valid_checksum(frame):
                continue

            # PSEUDOCODE: decode TFmini payload fields.
            dist_cm = frame[2] + (frame[3] << 8)
            strength = frame[4] + (frame[5] << 8)
            valid = dist_cm > 0 and strength > 0
            return LidarReading(
                distance_m=dist_cm / 100.0 if valid else None,
                strength=strength,
                quality=strength,
                valid=valid,
                raw_cm=dist_cm,
            )

    @staticmethod
    def _valid_checksum(frame: bytes) -> bool:
        # PSEUDOCODE: TFmini checksum is low byte of sum of first 8 bytes.
        return (sum(frame[:8]) & 0xFF) == frame[8]

    def close(self) -> None:
        # PSEUDOCODE: close serial handle safely.
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._serial = None
