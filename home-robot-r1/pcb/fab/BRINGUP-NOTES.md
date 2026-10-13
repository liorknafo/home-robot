# Bay board: bring-up and firmware notes for B10

Collected in B7 from the B3–B6 rulings. B10 folds these into `BRINGUP.md`.

## Battery connector J1 (ruling PB20)

- J1 is an AMASS XT30UPB-M, the vertical XT30 PCB male. J1 pin 1 = GND, pin 2 = BAT+ (silk "−" / "+" beside each pin, on the side
  toward the fuses: the housing covers the pin ends).
- The battery lead W01 (Tamiya plug → 18 AWG → AMASS XT30U-F) has GND (black) in cavity 1, the XT30U-F's "−" contact, and BAT+ (red)
  in cavity 2, its "+" contact.
- This is the reverse of the pre-B6 pinout. The harness (B9), the sim data and the docs follow `board_spec.J1_PINOUT`.
- There is no reverse-polarity protection: check the lead with a meter before the first plug-in. The XT30 mates only one way round, but
  a lead with its wires soldered the wrong way round still plugs in.

## Power connectors and the P03 terminals

- J1 (battery) is an XT30, J2 (rocker) a JST VH, J3 (Jetson 5 V) a Molex Micro-Fit 3.0 (43650-0200): three families, so no lead fits a
  wrong header (ruling PB30). The leads keep their tags: "J1 BATT", "J2 ROCKER", "J3 JETSON 5V".
- J3 pin 1 = 5 V (silk "+"), pin 2 = GND. J3 is in the rear band, reachable with the lid off.
- U5A (P03 in) and U5B (P03 out): pin 1 = "+" on both (silk "+"), the screw nearer the robot centre line; pin 2 = GND. Before the
  Jetson is connected, with the rocker on: U5A pin 1 reads the pack voltage and U5B pin 1 reads 5.1 V, both positive, against GND.
- Fit the fuses only with the rocker off (F2 last). Shut the Jetson down before the rocker goes off; rocker off before the battery is
  plugged or unplugged.

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

## Firmware: inputs, buses, battery

- **Internal pull-ups:** BUMP_L (GP6), BUMP_R (GP15) and ARM (GP14) have no pull-up resistors on the board (the switches pull to GND).
  Enable the RP2040's internal pull-ups on them.
- **I2C:** I2C1 on GP2 (SDA) / GP3 (SCL) at 100 kHz. R19 / R20 are 4.7 kΩ pull-ups; the bus runs over cables to four ToF boards and
  the IMU.
- **ToF addresses:** the four VL53L0X all start at 0x29. At boot, hold all four in reset with XSHUT1–XSHUT4 (GP0, GP10, GP5, GP13),
  then release one at a time and give each a new address: 0x30, 0x31, 0x32, 0x33 (cliff 1–4 = J6, J7, J8, J9). The IMU stays at 0x68.
- **Battery low-voltage stop** (6-cell NiMH: 7.2 V nominal, 8.7 V full):
  - VSENSE (GP26, ADC0) = VBAT_SW × 33 kΩ / (100 kΩ + 33 kΩ) = 0.248 × VBAT_SW (R11 100 k top, R12 33 k bottom, C2 100 nF). 8.7 V
    reads 2.16 V, inside the 3.3 V ADC range.
  - Warn at 6.6 V (1.1 V per cell: VSENSE 1.64 V). Stop at 6.0 V (1.0 V per cell: VSENSE 1.49 V, ADC count about 1847 of 4095 at the
    3.3 V reference).
  - Average over about 2 s so motor starts do not trip it. At the stop: stop the motors, tell the Jetson to shut down; the user then
    switches the rocker off and charges. Do not run NiMH below 1.0 V per cell.
- **Camera LED:** GP20 (SPARE3, J15 pin 3) = CAMERA_LED, an output, default low. The controller sets it high (LED fully on, about 59 mA from
  J15 pin 5 through a 39 Ω resistor) while a task drives, mapping or Go-to, and low when idle. No PWM: a rolling-shutter camera would
  show bands (GP20 is PWM slice 2 A if dimming is ever wanted). A 100 kΩ pull-down at the MOSFET gate keeps the LED off while the Pico
  boots or resets. J15 stays DNP on the board: the LED lead's three wires are soldered into its holes as an assembly step (BRINGUP.md section 10 item 6).
- **Jetson:** barrel-jack power needs the Jetson Nano's J48 jumper fitted.
- **IMU:** the MPU-6050 runs DLPF_CFG = 4 (about 20 Hz) with 20 Hz forwarding (ruling PB27; BRINGUP.md section 14).

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
