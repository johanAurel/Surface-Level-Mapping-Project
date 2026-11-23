# Encoders Module

## Purpose
Provides a reusable quadrature encoder abstraction used by actuator and external sensor readings.

## File
- `quadrature.py` - `QuadratureEncoder`

## Pseudocode
```text
CLASS QuadratureEncoder(pin_a, pin_b, counts_per_rev):
  init: create gpiozero.RotaryEncoder
  read_counts(): return encoder.steps
  read_degrees(): return steps / counts_per_rev * 360
  close(): release encoder
```

## Calibration
`counts_per_rev` must match the physical encoder and gearing. Incorrect values propagate into pan/tilt/platform angle and terrain-point derivation errors.
