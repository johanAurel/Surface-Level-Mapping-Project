"""Quadrature encoder abstraction.

PSEUDOCODE:
    CLASS QuadratureEncoder(pin_a, pin_b, counts_per_rev):
        INIT:
            CREATE gpiozero RotaryEncoder on pins A/B
            STORE counts_per_rev

        read_counts():
            RETURN current encoder step count

        read_degrees():
            RETURN (counts / counts_per_rev) * 360

        close():
            RELEASE encoder resources
"""

from gpiozero import RotaryEncoder


class QuadratureEncoder:
    """Read relative rotation from a quadrature encoder."""

    def __init__(self, pin_a, pin_b, counts_per_rev):
        # PSEUDOCODE: attach to hardware encoder pins and configure scale.
        self.encoder = RotaryEncoder(a=pin_a, b=pin_b, max_steps=0)
        self.counts_per_rev = counts_per_rev

    def read_counts(self):
        # PSEUDOCODE: return raw accumulated quadrature steps.
        return self.encoder.steps

    def read_degrees(self):
        # PSEUDOCODE: convert raw steps into degrees using calibration.
        return (self.read_counts() / self.counts_per_rev) * 360.0

    def close(self):
        # PSEUDOCODE: free GPIO/device resources.
        self.encoder.close()
