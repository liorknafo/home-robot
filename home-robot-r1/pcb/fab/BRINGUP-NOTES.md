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
- **Pico power (LIO-226):** the Pico's VBUS is not connected to the board and its VSYS (pin 39, U1B.2) only through D2, an SS14 from
  the board's 5V, DNP. As delivered the Pico and the whole 3V3 rail run from the Jetson's USB port only: with the Jetson off, the
  sensors are off. Fit D2 if the robot must run its sensors without the Jetson's USB (it sits inside DRV_R's socket rows: fit it with
  the DRV_R carrier out); the Pico's own VBUS diode blocks the back-feed into the USB host.
- **Firmware with D2 fitted and the Jetson off (LIO-234):** the Pico then runs from the board's 5V, so once firmware enables USB the
  RP2040's 1.5 kΩ D+ pull-up feeds 1–2 mA into the unpowered Jetson's hub. Enable USB only while the Pico's VBUS sense (GPIO24) is
  high; without VBUS, hold XSHUT1–XSHUT4 low and idle. The Pico and its sensors (about 0.2 A) also keep running after the low-battery
  shutdown, until the rocker goes off.

## Motor drivers (DRV8874, Pololu 4035)

- **Continuous current limit: 1.8 A per channel.** At 2.1 A continuous, T_J is 88–136 °C (40 °C air under the carrier; R_θJA 40 °C/W
  JEDEC, 70 °C/W assumed for the 2-layer carrier). 1.8 A keeps T_J ≤ about 110 °C on the carrier (`power_check.drv8874_current_for_tj`).
  The figures are quoted from memory (see ORDERING.md, "Verify before ordering").
- Firmware enforces the limit from CS (1.1 V/A on the carrier's 2.49 kΩ, read on GP27 L_CS_ADC / GP28 R_CS_ADC). Short peaks
  (starts, stall up to the 2.1 A design current) are fine; a stall must be cut within seconds.
- **Under slow decay CS reads I × duty** (LIO-234). The DRV8874's IPROPI senses the high-side FETs only; in the low-side brake
  off-time that slow decay needs, CS is 0. So the filtered reading is V_ADC ≈ D × I × 1.1 V/A, while the driver and the motor carry the
  full I. Firmware computes **I = V_ADC / (1.1 V/A × D)** and applies the 1.8 A limit to that I. It is valid for D ≳ 0.1; below that
  the reading is too small to trust, so rely on the stall timer at low duty. BRINGUP.md section 7 step 6 checks it at about 50 % duty
  against a series ammeter.
- **CS filters (LIO-226):** each CS reaches its ADC pin through 4.7 kΩ (R24 / R25) with 100 nF to GND on the pin side (C4 / C5). CS
  is a current source into the carrier's 2.49 kΩ, so τ = (2.49 k + 4.7 k) × 100 nF ≈ 0.72 ms (about 220 Hz). The ADC reads the 20 kHz
  chopped CS averaged, settled within about 3.6 ms (5τ). At the current limit CS sits near 3.2–3.3 V, the top of the ADC range and
  close to the pin's IOVDD + 0.3 V rating: the 4.7 kΩ limits the current into the pin's clamp. The filters sit by the drivers' CS pins
  (U3P.7, U4P.7), so the long run to the Pico carries the filtered level. The cap, sitting at the averaged level, also pulls the
  IPROPI / ITRIP node below I × 2.49 kΩ in the on-time: the driver's cycle-by-cycle current limit rises from about 2.9 A to about
  3.5 A at D = 0.5 (up to about 4.4 A at low duty and in the first ms, before the cap charges), still under the overcurrent
  protection.
- **Motor PWM ≈ 20 kHz** (`board_spec.DRV_PWM_HZ`), **with slow decay** (LIO-227): forward IN1 = 1 and IN2 = inverted PWM (reverse:
  IN2 = 1, IN1 = inverted PWM), so the off-time brakes on the low sides and the duty is the fraction with IN2 low. C3 (100 µF FK at
  DRV_L's VIN) then carries 0.10 A of ripple against its 0.16 A rating, with C1 fitted. **Fast decay is not allowed:** PWM on IN1 with
  IN2 low coasts in the off-time, the motor current flows back into VIN and C3 carries about 0.20 A, over its rating. The 0.16 A is the
  datasheet's 100 kHz figure, not derated to 20 kHz. At 1 kHz C3 would carry 0.18 A even with slow decay, so do not run the motors at
  low PWM frequencies, and never without C1. The off-time brake lasts one PWM period: it is not the sustained short brake a disarm must
  avoid (BRINGUP.md section 8, braking), and the firmware ramp still sets the deceleration.
- PMODE is tied high (PWM / IN-IN mode); SLEEP (GP7) wakes both drivers; FAULT (GP8, open drain) is shared.

## Firmware: inputs, buses, battery

- **Switch inputs** (the switches pull to GND): ARM (GP14), a normally-open button on a 200 mm lead, has a 10 kΩ pull-up on the board
  (R26, LIO-226), not the RP2040's 50–80 kΩ beside the motor PWM. BUMP_L (GP6) and BUMP_R (GP15) are normally closed (an open lead
  reads as a bump, fail-safe) and have none: enable the RP2040's internal pull-ups on those two.
- **Encoders:** the right encoder's A / B are adjacent pins (GP11 / GP12), so a standard two-pin PIO quadrature program reads it. The
  left one's are not (GP1 / GP4, `PICO_GPIO`): read it with GPIO edge interrupts on both pins, or a PIO program that takes one pin as
  its `jmp pin` condition and the other through `in` / `wait`. No board change.
- **I2C:** I2C1 on GP2 (SDA) / GP3 (SCL) at 100 kHz. R19 / R20 are 10 kΩ pull-ups (LIO-226; 4.7 kΩ made the bus 0.94 kΩ, 3.1 mA at
  0.4 V, over I2C's 3 mA); the bus runs over cables to four ToF boards and the IMU. With the GY-521's 2.2 kΩ and the four ToF carriers'
  10 kΩ the bus totals about 1.05 kΩ (2.8 mA at 0.4 V). Measure it powered, with everything plugged in (BRINGUP.md section 4 item 4):
  the Pico on USB in BOOTSEL, a meter on its mA range from SDA (then SCL) to GND, R = 3.3 V / I; expect about 3.1 mA (1.05 kΩ), and
  leave R19 / R20 off if R < 1 kΩ. An ohmmeter to 3V3 with the Pico unpowered reads the RP2040's pad diodes and misses the GY-521's
  pull-ups, which hang on its LDO's output.
- **ToF addresses:** the four VL53L0X all start at 0x29. At boot, hold all four in reset with XSHUT1–XSHUT4 (GP0, GP10, GP5, GP13),
  then release one at a time and give each a new address: 0x30, 0x31, 0x32, 0x33 (cliff 1–4 = J6, J7, J8, J9). The IMU stays at 0x68.
- **MOTOR_ON_N (GP9) reports the toggle, not VMOT:** Q5's gate is on COIL_P, so it reads low ("on") whenever the J13 loop energises
  K1's coil, even with F2 blown or the relay contacts failing to close. Only LED3 (VMOT) shows motor power, and it sits under the
  carrier: firmware cannot tell a dead motor supply from MOTOR_ON_N (a motor that does not move with the drivers awake and CS near 0 is
  the sign).
- **Battery low-voltage stop** (6-cell NiMH: 7.2 V nominal, 8.7 V full):
  - VSENSE (GP26, ADC0) = VBAT_SW × 33 kΩ / (100 kΩ + 33 kΩ) = 0.248 × VBAT_SW (R11 100 k top, R12 33 k bottom, C2 100 nF). 8.7 V
    reads 2.16 V, inside the 3.3 V ADC range. With USB absent and the rocker on, the Pico is unpowered and about 65 µA (at 8.7 V; the divider's
    own current, rising toward 90 µA as the clamp pulls GP26 toward 0 V) flows from R11 into GP26's clamp diode: acceptable, no fix.
  - Warn at 6.6 V (1.1 V per cell: VSENSE 1.64 V). Stop at 6.0 V (1.0 V per cell: VSENSE 1.49 V, ADC count about 1847 of 4095 at the
    3.3 V reference).
  - The 6.0 V stop assumes the P03 holds 5 V at 4.71 A (`board_spec.P03_OUT_A`: the Jetson's 4.5 A + the 5V taps) from 6.0 V
    (`board_spec.P03_VIN_MIN`). BRINGUP.md section 5 tests it at 6.0 V in, 4.71 A out, with the input current read. If it fails
    there, raise the stop to the lowest passing input + 0.5 V (motor starts sag the pack 0.3–0.5 V), and `P03_VIN_MIN` with it.
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

- U3P.2 (DRV_L GND) joins its copper with one relief spoke per outer layer (ruling PB21). Its copper carries 3.25 A against 2.1 A with
  the B6 model's 20 µm via plating (`copper.VIA_PLATING_MM`), 3.11 A at JLCPCB's 18 µm minimum (ORDERING.md, "Verify before
  ordering"); after a stall test, check that the joint is not hot.
- The air under the carrier is assumed to be 40 °C in the driver figures: measure it in the lid-on test.
