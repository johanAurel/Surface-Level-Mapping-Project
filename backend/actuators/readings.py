"""Actuator encoder readings for pan/tilt turret.

PSEUDOCODE:
    CLASS ActuatorReadings(pan pins, tilt pins, counts_per_rev):
        INIT:
            CREATE QuadratureEncoder for pan
            CREATE QuadratureEncoder for tilt

        read():
            READ pan counts/degrees
            READ tilt counts/degrees
            RETURN normalized dict

        close():
            CLOSE pan and tilt encoders
"""

from backend.encoders.quadrature import QuadratureEncoder


class ActuatorReadings:
    """Expose pan and tilt encoder measurements as one component."""

    def __init__(self, pan_pin_a, pan_pin_b, tilt_pin_a, tilt_pin_b, pan_counts_per_rev, tilt_counts_per_rev):
        # PSEUDOCODE: initialize two independent encoder channels.
        self.pan_encoder = QuadratureEncoder(pan_pin_a, pan_pin_b, pan_counts_per_rev)
        self.tilt_encoder = QuadratureEncoder(tilt_pin_a, tilt_pin_b, tilt_counts_per_rev)

    def read(self):
        # PSEUDOCODE: sample both axes and return a flat mapping for fusion.
        return {
            "pan_counts": self.pan_encoder.read_counts(),
            "pan_degrees": self.pan_encoder.read_degrees(),
            "tilt_counts": self.tilt_encoder.read_counts(),
            "tilt_degrees": self.tilt_encoder.read_degrees(),
        }

    def close(self):
        # PSEUDOCODE: release hardware handles.
        self.pan_encoder.close()
        self.tilt_encoder.close()
