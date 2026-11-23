"""External/platform encoder readings.

PSEUDOCODE:
    CLASS SensorEncoder(pin_a, pin_b, counts_per_rev):
        INIT:
            CREATE QuadratureEncoder

        read():
            RETURN counts and degrees

        close():
            CLOSE encoder
"""

from backend.encoders.quadrature import QuadratureEncoder


class SensorEncoder:
    """Read external encoder used for platform/wheel motion estimation."""

    def __init__(self, pin_a, pin_b, counts_per_rev):
        # PSEUDOCODE: configure quadrature encoder for external movement source.
        self.encoder = QuadratureEncoder(pin_a, pin_b, counts_per_rev)

    def read(self):
        # PSEUDOCODE: return raw and converted encoder measurements.
        return {
            "sensor_encoder_counts": self.encoder.read_counts(),
            "sensor_encoder_degrees": self.encoder.read_degrees(),
        }

    def close(self):
        # PSEUDOCODE: release hardware resources.
        self.encoder.close()
