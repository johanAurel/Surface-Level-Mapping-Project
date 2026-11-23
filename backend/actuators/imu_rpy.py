"""IMU orientation reader for BNO055.

PSEUDOCODE:
    CLASS ImuOrientation(address):
        INIT:
            OPEN I2C bus
            CREATE BNO055 sensor

        read():
            READ euler angles (heading/yaw, roll, pitch)
            READ temperature
            RETURN ImuReading dataclass

        close():
            NO-OP / bus owned by Blinka
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import adafruit_bno055
import board
import busio


@dataclass
class ImuReading:
    roll_deg: Optional[float]
    pitch_deg: Optional[float]
    yaw_deg: Optional[float]
    temperature_c: Optional[int] = None


class ImuOrientation:
    """Read fused Euler orientation from a BNO055."""

    def __init__(self, address: int = 0x28):
        # PSEUDOCODE: initialize I2C transport and IMU driver.
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c, address=address)

    def read(self) -> ImuReading:
        # PSEUDOCODE: query fused orientation and normalize missing values.
        heading, roll, pitch = self.sensor.euler
        return ImuReading(
            roll_deg=roll,
            pitch_deg=pitch,
            yaw_deg=heading,
            temperature_c=self.sensor.temperature,
        )

    def close(self) -> None:
        # PSEUDOCODE: Blinka manages I2C lifecycle; keep explicit hook for interface symmetry.
        pass


def main() -> None:
    # PSEUDOCODE: simple manual test loop for hardware debugging.
    imu = ImuOrientation()
    try:
        while True:
            print(imu.read())
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping IMU reader.")
    finally:
        imu.close()


if __name__ == "__main__":
    main()
