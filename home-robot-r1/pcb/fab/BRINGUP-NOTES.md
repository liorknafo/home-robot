# Bay board: bring-up and firmware notes for B10

Collected in B7 from the B3–B6 rulings. B10 folds these into `BRINGUP.md`.

## Battery connector J1 (ruling PB20)

- J1 pin 1 = GND, pin 2 = BAT+ (silk "−" / "+" outside the housing). The battery lead has GND in cavity 1 and BAT+ in cavity 2.
- This is the reverse of the pre-B6 pinout. The harness (B9), the sim data and the docs follow `board_spec.J1_PINOUT`.
- There is no reverse-polarity protection: check the lead with a meter before the first plug-in.

## Motor drivers (DRV8874, Pololu 4035)

- **Continuous current limit: 1.8 A per channel.** At 2.1 A continuous, T_J is 88–136 °C (40 °C air under the carrier; R_θJA 40 °C/W
  JEDEC, 70 °C/W assumed for the 2-layer carrier). 1.8 A keeps T_J ≤ about 110 °C on the carrier (`power_check.drv8874_current_for_tj`).
  The figures are quoted from memory (see ORDERING.md, "Verify before ordering").
- Firmware enforces the limit from CS (about 1.1 V/A on the carrier's 2.49 kΩ, read on GP27 L_CS / GP28 R_CS). Short peaks (starts,
  stall up to the 2.1 A design current) are fine; a stall must be cut within seconds.
- **Motor PWM ≈ 20 kHz** (`board_spec.DRV_PWM_HZ`). C3 (100 µF FK at DRV_L's VIN) then carries 0.10 A of ripple against its 0.16 A
  rating, with C1 fitted. At 1 kHz C3 would carry 0.18 A, over its rating, so do not run the motors at low PWM frequencies, and never
  without C1.
- PMODE is tied high (PWM / IN-IN mode); SLEEP (GP7) wakes both drivers; FAULT (GP8, open drain) is shared.

## LEDs and test pads under the carrier

All three status LEDs and all five test pads sit under the Jetson carrier (u ≥ 28, robot x ≥ −38.5): they are visible and reachable only
before the Jetson is fitted. Do the board bring-up with the Jetson off.

| Ref | Shows / net | Robot (x, y) mm |
|---|---|---|
| LED1 | VBAT_SW (green, R21) | (−30.8, 46.0) |
| LED2 | 5V (green, R22) | (−26.3, 39.5) |
| LED3 | VMOT (yellow, R23) | (−26.0, −36.0) |
| TP1 | VBAT_SW | (−17.0, 43.5) |
| TP2 | VMOT | (−32.5, −40.0) |
| TP3 | 5V | (−5.5, 10.5) |
| TP4 | 3V3 | (−35.5, 45.0) |
| TP5 | GND | (−22.3, 27.4) |

TP2 sits under the carrier past the end of the right driver's U4L row, near the right edge: probe VMOT there with the Jetson off,
or at C1's "+" lead, which is behind the carrier.

## Other checks

- U3P.2 (DRV_L GND) joins its copper with one relief spoke per outer layer (ruling PB22). Its copper carries 3.25 A against 2.1 A; after a
  stall test, check that the joint is not hot.
- The air under the carrier is assumed to be 40 °C in the driver figures: measure it in the lid-on test.
