# R1 bring-up guide (bay board under the Jetson)

How to solder, check, fit and first power the R1 power & I/O board (the 4-layer "bay board"), and the checks to make before the Jetson
and the lid go on.

**Where the numbers come from.** Written for commit 76232c4 (bay B10); the power connectors, the power rules and the assembly numbers
updated in bay B12a.
- The assembly order is `ASSY_ORDER` in `cad/build_robot.py`. Its resolved text is in `cad/component-manifest.json` (the `bay_board`
  note) and `design-review.html`. If this guide and `ASSY_ORDER` ever disagree, `ASSY_ORDER` wins. Section 6 follows its numbering.
- Soldering and ordering: `pcb/fab/ORDERING.md`.
- The firmware and board notes from B7 (`pcb/fab/BRINGUP-NOTES.md`) are folded in here.
- Heights and clearances: `cad/validation.json`.
- Part positions: `pcb/board_cad.json`, generated from the routed board. They are in robot coordinates (mm): x forward, y left, the
  board top at z 37.0.

---

## 0. Before anything: the battery connector J1 and the power rules

> **J1 pin 1 = GND, pin 2 = BAT+** (silk "−" and "+" beside each pin, on the side toward the fuses: the housing covers the pin ends;
> ruling PB20). This is the reverse of the usual "pin 1 = +" habit.
>
> **J1 is an XT30 header:** the AMASS XT30UPB-M, a vertical PCB-mount male rated 15 A continuous, on the board's rear edge. It is
> plugged from above with the lid off. The battery lead W01 runs from the battery's Tamiya plug, on 18 AWG silicone wire (red BAT+,
> black GND), to an AMASS XT30U-F (the female half). The XT30 housing is D-shaped, so it mates only one way round. The lead must carry:
> - **BAT+ (battery +, red) on the XT30U-F's "+" contact** (cavity 2), which mates J1 pin 2;
> - **GND (battery −, black) on its "−" contact** (cavity 1), which mates J1 pin 1.
>
> The D shape stops the plug going in the wrong way round. It does not stop a lead with its two wires soldered the wrong way round, so
> the meter check below still applies.
>
> **Three connector families: no lead fits a wrong header.** J1 (battery) is an XT30, J2 (rocker) a JST VH, J3 (Jetson 5 V) a Molex
> Micro-Fit 3.0. Tag each lead anyway, with a heat-shrink label near its plug: "J1 BATT", "J2 ROCKER", "J3 JETSON 5V".
>
> **The board has no reverse-polarity protection.** A reversed lead with the fuses fitted puts the battery backwards across the whole
> board. That is the costliest mistake you can make on it.
>
> **Fail-safe meter check, before the fuses ever go in** (ASSY step 0):
> 1. Leave F1 and F2 out.
> 2. Plug W01 into J1 and into the battery.
> 3. Measure **F1's input clip** (its BAT+ side, F1 pin 1) against GND. For GND, use an M3 hole ring (H1–H4), or TP5 if the GY-521 is
>    not yet plugged in (TP5 sits under it).
> 4. It must read **positive, about the pack voltage (≥ 6 V)**. A part-discharged pack reads lower.
> 5. A negative reading means the lead is reversed. With the fuses out, it has no current path: unplug it and swap the two wires at
>    the XT30U-F.
>
> Only after a positive reading may the fuses go in. **Fit a fuse only with the rocker off:** F1 first, F2 last (section 7, steps 1
> and 5).
>
> **The same check for the Jetson lead W04, before it ever touches the Jetson** (after section 7 step 3, with the P03 set and
> checked). The Micro-Fit only mates one way round, but its two crimps can still sit in the wrong cavities:
> 1. Plug W04 into J3. Leave its barrel plug free (not in the Jetson).
> 2. Rocker on. Measure the barrel plug's **centre against its sleeve**: it must read **positive, about 5.1 V**.
> 3. A negative reading means the crimps are swapped: rocker off, unplug W04, swap the two crimps in the receptacle.
>
> **Everyday rule:** switch the rocker off before you plug or unplug the battery, and shut the Jetson down before you switch the
> rocker off.

---

## 1. Solder the board

Order: **reflow SMD → inspect → iron THT, trimming leads to ≤ 1.4 mm → modules.** `pcb/fab/ORDERING.md` "Assembly" gives the full text.

1. **Reflow the SMD parts, top side only.** Tape the board down and print paste through the stencil. Place C2, C3, D1, Q1–Q5,
   LED1–LED3 and R1–R23, following the polarity marks on the silk: the LED and D1 cathode marks, SOT-23 pin 1 and C3 "+". Reflow on a
   hot plate or in an oven. TP1–TP5 stay bare copper.
2. **Inspect.**
   - Look for bridges on the SOT-23 and 0805 parts, and re-check the polarity marks.
   - Check the GND joints for solder wicking into the tented vias beside them. The mask web there is only about 0.1 mm.
3. **Iron the THT parts, lowest first:**
   - the sockets U1A / U1B, U2, U3L / U3P, U4L / U4P;
   - the connectors: the XH headers, the VH (J2), the XT30 (J1) and the Micro-Fit (J3; push its plastic peg through its hole);
   - J4 / J5, U5A / U5B;
   - the FLR fuse holders, K1, and C1 ("+" on the silk).

   The power pads keep thermal-relief spokes, so use a large tip. U3P.2 (DRV_L GND) has one spoke per outer layer by design (ruling
   PB22): give it time to wet.

   **Trim every lead to ≤ 1.4 mm under the board.** The board sits on 2.4 mm spacers and printed ribs, which leaves 1.0 mm of air.
4. **Modules come last** (section 4 onward): the Pico first, on USB (section 4), then the P03, set on the bench (section 5), then the
   two DRV8874 carriers and the GY-521.

## 2. Unpowered checks

Make these checks with the board bare: no battery, no USB, no modules, F1 and F2 out.
- With a meter, check for **no short** from GND to BAT+, VBAT_SW, VMOT, 5V and 3V3.
  - For VBAT_SW, VMOT, 5V and 3V3, use the test pads in section 7.
  - BAT+ has no pad: probe it at F1's input clip or J1 pin 2.
- C1 and C3 charge from the meter on the resistance range, so the reading climbs. That is normal; a steady near-0 Ω reading is not.

## 3. The J1 lead check

Do the fail-safe check in section 0 now, on the bench. It is ASSY step 0.

## 4. Pico on USB only (battery unplugged, Jetson off)

1. Plug the Pico into its sockets, with the USB end toward the board's left edge. Connect it to a computer by USB.
2. The Pico's VSYS and VBUS are not connected to the board, so USB powers only the Pico and, through its 3V3 pin, the board's 3V3 rail:
   the GY-521, the cliff ToF sensors (when plugged in) and the pull-ups.
3. **Expect:**
   - TP4 (3V3) at about 3.3 V.
   - TP3 (5V), TP1 (VBAT_SW) and TP2 (VMOT) dead.
   - LED1–LED3 off.
4. With a test sketch:
   - The I2C bus (GP2 SDA, GP3 SCL) finds the GY-521 at 0x68 (AD0 is tied to GND).
   - Each ToF sensor answers once its XSHUT line (GP0, GP10, GP5, GP13) is released.
5. The Pico's 3V3 regulator has about 300 mA for external loads (verify item in `ORDERING.md`).

## 5. Set the P03 5 V converter (before the Jetson)

1. Set the P03 on the bench, with a bench supply on its input: **trimpot to 5.1 V**.
2. Then verify it at **6.6 V input, with the intended load and motor starts**. Its rating alone is not acceptance.
3. Only then connect it to U5A (input) / U5B (output) and fit it (ASSY step 5).
4. **Polarity: pin 1 is "+" on both terminals.**
   - U5A pin 1 = P03 IN+ (VBAT_SW); U5B pin 1 = P03 OUT+ (5 V). Pin 2 is GND on both (IN− / OUT−).
   - Pin 1 is the screw nearer the robot's centre line. The silk "+" for U5A sits on the board just behind the terminal (the side away
     from the P03); for U5B, just past the terminal's end nearer the centre line.
   - Before the first power, check each "+" lead end to end with the meter's continuity range: P03 IN+ to U5A pin 1, P03 OUT+ to U5B
     pin 1. A swapped input lead puts the pack backwards on the P03 the moment the rocker goes on.
   - Section 7 step 3 checks both with power on, before the Jetson is connected.

Once the Jetson is on, its trimpot and tie cannot be reached.

## 6. Assembly into the robot (ASSY_ORDER, condensed)

The numbers below are copied from the resolved `ASSY_ORDER` (bay B12a build). The full text is the reference.

### Step 0. On the bench

**Make the two motor pigtails (C18) and tag them at once.**
- Tag one "**L / J4**" (left motor) and the other "**R / J5**" (right motor), right after making each one and before it is plugged on.
  They differ in length and in pin-1 order.
- Pre-cut the six wires to **61 mm (J4) / 57 mm (J5)**: the drawn route, plus the terminations, plus the full 12.7 mm pin spread.
  The six wires fan out 6.35 mm either side of the bundle's centre line at both ends: at the socket's sleeves and again at the male
  housing's cavities.
- Solder the socket end first: one wire on each tail of the right-angle socket, under a Ø1.8 sleeve. The red wire goes on pin 1, and
  the tails and wires point up.
- Push the socket onto its header. Leave the male ends long; they are finished in step 4.
- J4 pin 1 is at the rear end and J5 pin 1 at the front end (the square pads). The rule is the same for both: looking into the
  socket's mating face with its tails up, **the red wire (pin 1) is on the left**.
- The pigtails stay with the board.

**Tag the three power leads** "J1 BATT", "J2 ROCKER" and "J3 JETSON 5V". They are three different connector families and cannot be
cross-plugged (section 0); the tags are there anyway.

**Do the J1 fail-safe check** (section 0).

### Step 1. Ribs and front standoffs

- Bond the two printed bearing ribs into their 0.6 mm pockets in P_base with epoxy (F04), and let it cure.
- Fit the two front Jetson standoffs (24 mm stacks), screwed from below.

### Step 2. The board

Set the four 2.4 mm spacers and the two clamp spacers on P_base. Lower the board onto them and fix it with four M3 × 8 pan-head screws
into the inserts.

### Step 3. The battery tray, after the board

1. The tray's hold-down beams must be freed first: cut the breakaway posts.
2. **Set the tray 2 mm aft of its place**, then slide it forward. Its three spring fingers ride up their lead-ins onto the board's top
   face, between the rear connectors.
3. Screw the tray down.
4. **Hold-down feeler check:** at each finger, a **0.05 mm feeler must not pass under the tip.**
5. If it passes, that finger has play. Shim it with **0.1 mm PTFE tape (F03)** under the tip, one layer at a time (about 0.3 mm at
   most), until the feeler no longer passes.

### Step 4. The under-carrier cables

**Pigtails.**
1. Dress each one as drawn, with no slack (it rides up with the board when the joint is unplugged). The route: straight up out of the
   socket, forward about 12 mm over the board beside U5A (beside J9 on the right), out between the P03 and the motor face, and down to
   the inline joint just ahead of the motor (x 15.2..17.8, |y| 38..60).
2. Trim each wire at its own cavity of the male housing. Its finished length depends on where the wire sits at both ends:
   - J4: 35.4–60.8 mm (about 48.1 mm on the bundle's centre line);
   - J5: 31.1–56.5 mm (about 43.8 mm).
3. Crimp the male pins into the 1×6 housing, red wire = pin 1.

**Factory motor cables: measure, then coil.**
1. Measure each cable from the encoder cap to the female housing's rear face (the wire entry). Readings of **186–200 mm** all fit
   (2–3 turns). A longer cable, up to **220 mm**, still fits (up to 3.5 turns).
2. Lay the fixed path first, about 89 mm outside the coil:
   - a flat row along the motor's inner end face toward the rear;
   - round the motor's rear shoulder and forward over the motor top (never over the pigtail);
   - down the motor's front shoulder, under the bumper lead;
   - from the coil position into the housing.
3. Wind the remainder (the measured length less the fixed path: **97–111 mm**) into the **coil** on the base ahead of the motor:
   2–3.5 turns, about 16 mm across, at x 25, |y| 72. Tie it through its two slots.

**Inline joints.**
- Mate each motor's factory female housing onto its pigtail's male housing.
- The joint is **not keyed**: mate the pin-1 stripe to the pin-1 stripe (both halves are marked at the red wire), and check that the
  colours run straight across.
- Stand the joint on the base and tie it down: the tie goes down the motor opening and up through the slot beside it.

**Then plug the rest:** the bumpers J10 / J11 and the front cliffs J8 / J9.

### Step 5. The P03

Set and verify it on the bench (section 5). Connect it to U5A / U5B: pin 1 ("+", the screw nearer the robot's centre line) is IN+ on
U5A and OUT+ on U5B; the red wires go on pin 1. Fit it on its foam tape in the front bay, with the cable tie through the P_base slots at
(28, ±33.3). Before the Jetson is connected, check both terminals with the meter (section 7 step 3).

### Step 6. Rear standoffs

Stand the two rear 20 mm F-F standoffs on the board. Screw each one down with an M2.5 × 12 countersunk screw from under P_base,
through the base, the clamp spacer and the board. These two stacks clamp the board.

### Step 7. The Jetson

Fit the Jetson (four M2.5 top screws), then its six plugs. Its barrel-jack power needs the J48 jumper fitted (section 8). Before the
5 V barrel plug goes in, check the Jetson lead W04 (section 0): its centre reads positive 5.1 V against its sleeve.

Before step 7, do sections 7–9: the LEDs, the test pads and the under-carrier plugs can be reached only until the Jetson is on.

## 7. First power with the Jetson off

All three status LEDs and all five test pads sit **under the Jetson carrier** (robot x ≥ −38.5). They can be seen and reached only
before the Jetson is fitted, so do this whole section with it off.

| Ref | Shows / net | Robot (x, y) mm | Note |
|---|---|---|---|
| LED1 | VBAT_SW (green, R21 3.3 k) | (−30.8, 46.0) | |
| LED2 | 5V (green, R22 1 k) | (−26.3, 39.5) | |
| LED3 | VMOT (yellow, R23 3.3 k) | (−26.0, −36.0) | |
| TP1 | VBAT_SW | (−17.0, 43.5) | |
| TP2 | VMOT | (−32.5, −40.0) | right edge, past the end of the right driver's U4L row; or probe VMOT at C1 "+" (behind the carrier) |
| TP3 | 5V | (−5.5, 10.5) | |
| TP4 | 3V3 | (−35.5, 45.0) | |
| TP5 | GND | (−22.3, 27.4) | **under the GY-521 module**: probe it before the IMU is plugged in, or use an M3 hole ring as GND |

> **Note on the GY-521.** B3's note (carried into the B10 brief) placed TP1, TP3, TP4 and TP5 under the GY-521. On the routed board
> (`board_cad.json`, GY-521 outline x −36.6..−20.2, y 13.2..34.4) **only TP5 is under it**. TP1, TP3 and TP4 lie outside the module,
> though still under the carrier. No test pad or LED sits under the Pico or the DRV8874 carriers.

Steps:

1. **Rocker off; then battery in, F1 fitted, F2 out.** Expect LED1 off and TP1 at 0 V.
2. **Rocker on.** Expect:
   - LED1 on;
   - TP1 at about the pack voltage;
   - the Pico's VSENSE (GP26) at about 0.248 × TP1 (the R11 100 k / R12 33 k divider; 2.16 V at 8.7 V).
3. **5 V rail and the P03 polarity check** (P03 connected; the Jetson lead W04 unplugged from J3). Measure against a GND pad:
   - U5A pin 1 ("+", P03 IN+) reads the pack voltage, positive;
   - U5B pin 1 ("+", P03 OUT+) reads the set 5.1 V, positive. TP3 reads the same, and LED2 is on.

   A negative or 0 V reading at U5B pin 1 means the P03 leads are swapped or loose: rocker off, battery out, fix them before going on.
4. **Motor toggle on** (the J13 loop). K1 clicks, and MOTOR_ON_N (GP9) reads low. With F2 still out, VMOT stays dead.
5. **Rocker off, fit F2, rocker on.** F2 goes in only with the rocker off: fitted live, it would charge C1 + C3 (570 µF) through the
   fuse contacts. Expect:
   - TP2 (or C1 "+") at about the pack voltage;
   - LED3 on.
6. **Motors, wheels lifted.** Use the firmware limits in section 8.
   - Wake the drivers: DRV_SLEEP (GP7) high. R17 holds SLEEP low by default.
   - Spin each motor both ways at 20 kHz PWM.
   - Check that the encoder counts follow (GP1 / GP4 left, GP11 / GP12 right).
   - Check that L_CS (GP27) and R_CS (GP28) read about 1.1 V/A.
7. **After a stall test,** touch U3P.2 (DRV_L GND, ruling PB22): its joint must not be hot.

## 8. Firmware limits (from BRINGUP-NOTES.md)

- **DRV8874 continuous current: 1.8 A per channel.**
  - At 2.1 A continuous, the junction reaches 88–136 °C. This assumes 40 °C air under the carrier, R_θJA 40 °C/W (JEDEC), and 70 °C/W
    for the 2-layer carrier. 1.8 A keeps it at or below about 110 °C.
  - Firmware enforces the limit from CS: about 1.1 V/A on the carrier's 2.49 kΩ, read on GP27 (L_CS) and GP28 (R_CS).
  - Short peaks are fine: starts, and a stall up to the 2.1 A design current. A stall must be cut within seconds.
  - These datasheet figures are quoted from memory; see "Verify before ordering" in `ORDERING.md`.
- **Motor PWM about 20 kHz** (`board_spec.DRV_PWM_HZ`).
  - With C1 fitted, C3 (100 µF FK at DRV_L's VIN) then carries 0.10 A of ripple, against its 0.16 A rating. At 1 kHz it would carry
    0.18 A, over its rating.
  - **Never run the motors at low PWM frequencies, and never without C1.**
- **Driver control lines:**
  - PMODE is tied high (PWM / IN-IN mode).
  - SLEEP (GP7) wakes both drivers; R17 pulls it low.
  - FAULT (GP8) is open drain and shared, with an R18 pull-up.
  - Each motor's IN1 / IN2 pair is on one PWM slice: GP16 / 17 left, GP18 / 19 right.
- **Switch inputs need the internal pull-ups.** BUMP_L (GP6), BUMP_R (GP15) and ARM (GP14) have no pull-up resistors on the board; the
  switches pull them to GND. Enable the RP2040's internal pull-ups on these three pins in firmware.
- **I2C:** I2C1 on GP2 (SDA) / GP3 (SCL) at **100 kHz**. The board has 4.7 kΩ pull-ups (R19 / R20); the bus runs over cables to the
  four ToF boards and the IMU.
- **ToF addresses.** The four VL53L0X sensors all start at address 0x29.
  1. At boot, hold all four in reset: XSHUT1–XSHUT4 (GP0, GP10, GP5, GP13) low.
  2. Release one at a time, and give each a new address: **0x30, 0x31, 0x32, 0x33** for cliff 1–4 (J6, J7, J8, J9).
  3. The IMU stays at 0x68.
- **Battery low-voltage stop.** The pack is 6-cell NiMH: 7.2 V nominal, 8.7 V full.
  - VSENSE (GP26, ADC0) = VBAT_SW × 33 kΩ / (100 kΩ + 33 kΩ) = **0.248 × VBAT_SW** (R11 100 k top, R12 33 k bottom, C2 100 nF).
    Full charge (8.7 V) reads 2.16 V, inside the 3.3 V ADC range.
  - **Warn at 6.6 V** (1.1 V per cell; VSENSE 1.64 V).
  - **Stop at 6.0 V** (1.0 V per cell; VSENSE 1.49 V, an ADC count of about 1847 of 4095 at the 3.3 V reference).
  - Average over about 2 s, so motor starts do not trip it.
  - At the stop: stop the motors and tell the Jetson to shut down. Then switch the rocker off and charge the pack.
  - Do not run NiMH below 1.0 V per cell.
- **Jetson power:** barrel-jack power needs the Jetson Nano's **J48 jumper** fitted.
- **IMU:** the MPU-6050 runs DLPF_CFG = 4 (about 20 Hz), with samples forwarded at 20 Hz (ruling PB27; see section 14).

## 9. Before fitting the Jetson: plugs and the gap under the carrier

1. **Plug every under-carrier connector first.** Once the Jetson is on, none of them can be reached:
   - the J4 / J5 pigtails and their inline joints (step 4);
   - the bumpers J10 / J11;
   - the front cliffs J8 / J9;
   - the P03 leads at U5A / U5B;
   - the spares J14 / J15, if fitted (DNP as delivered).

   **J3 (Jetson 5 V)** is not under the carrier. It is in the rear band, behind the carrier (robot about (−43.9, −44.5)), with its
   mating face at the board's right edge, reachable with the lid off. Plug the Jetson lead W04 into it after the section 7 checks.
2. **Measure the gap.** The carrier PCB underside sits at z 57.0, which is **20.0 mm over the board top**. The plan's limit for any part
   under it, including a mated plug and its wire bend, is **16.27 mm over the board top** (z 53.27). That leaves 1.5 mm below the
   carrier's through-hole lead tails, which the CAD allows to reach 2.23 mm under the carrier PCB (`validation.carrier_lead_tail_slab`).

   The CAD's tallest under-carrier items:
   - the DRV8874 and GY-521 module stacks: 14.0 mm (z 51.0, `bay_under_carrier_max_top_z`);
   - the mated XH plugs J8–J11 with their wire bend: 14.8 mm;
   - the pigtail sockets and sleeves: 13.8 mm (z 50.8);
   - the U5A / U5B terminals: 13.8 mm.

   Measure two things:
   - **(a)** the tallest part over the board top, with every plug mated;
   - **(b)** how far the Jetson carrier's lead tails stand below its PCB. Measure this on the Jetson, upside down.

   Then require **(a) + (b) + 1.5 mm ≤ 20.0 mm**.

   One way to see the real gap: put a small ball of modelling clay on the tallest parts, fit the Jetson, take it off again, and measure
   the squashed clay.

   **If the rule fails, do not force the Jetson down.** Re-dress the cable that is too high, or report it. The DRV and GY-521 stacks
   have only 2.27 mm of margin (bay spec, risk 2).

Then fit the Jetson (ASSY step 7).

## 10. Lid-on checks

1. **Fan skirt feeler** (the lid note in `cad/component-manifest.json`).
   - Do this when the lid is first fitted, before the LiDAR tray is bolted onto it: the tray's grille closes the lid's fan window.
   - Set the lid on its posts. Put the feeler in through the Ø35 fan window, lay it flat on the fan frame, and slide it outward under the
     bottom edge of the skirt, all round.
   - A **0.5 mm feeler must pass** (no contact). If it touches, sand the skirt end.
   - A **1.0 mm feeler must not pass.** If it passes, shim the fan: otherwise the fan's bypass limit is lost. The bypass is 22.4 % of the
     grille's open area, against a 25 % limit that is reached at a 1.01 mm gap.
   - The nominal gap is 0.9 mm (`validation.jetson_cooling.fan_skirt`).
2. **Lid-on tegrastats test.**
   - Everything is fitted and the lid and LiDAR tray are on.
   - Run the Jetson under its real load (the autonomy stack, or a CPU + GPU stress) until the temperatures settle, about 30 minutes, and
     log `tegrastats` throughout.
   - Pass: no thermal throttling (no forced clock drop), and the temperatures level off.
   - Record the peak CPU / GPU / thermal-zone values.
   - Also record the air temperature under the carrier, near the drivers: the DRV8874 figures in section 8 assume 40 °C.

## 11. Service

**Step 0, every time, before you reach inside:** shut the Jetson down, switch the rocker off, then unplug the battery at its Tamiya
plug. BAT+ is live and unfused at F1's input whenever the battery is plugged in: the rocker switches only what comes after F1.

- **Everyday rule:** switch the rocker off before you plug or unplug the battery, and shut the Jetson down before you switch the
  rocker off. Fit or pull a fuse only with the rocker off.
- **With the lid off, these are reachable:** the board's rear band (Pico USB, the rear connector row, the Jetson 5 V lead J3, the
  fuses).
- **These need the Jetson off:** the motor inline joints, the other under-carrier connectors and the P03 (tie, trimpot). To take the
  Jetson off, unplug its six plugs (camera, LiDAR and Pico USB, Wi-Fi dongle, 5 V barrel plug, fan), then undo its four top screws.
- **To take the board out:**
  1. Jetson off.
  2. The two rear standoffs off (their M2.5 screws from under P_base).
  3. The battery out (undo the strap; its Tamiya plug is already out, step 0).
  4. The four tray screws out. Slide the tray 2 mm aft and lift it straight out.
  5. The four M3 board screws out.
  6. Unplug the rear-band plugs.
  7. **Unplug the motor inline joints before the board lifts:**
     - loosen each factory-cable coil tie first (the 8 mm each housing slides back comes out of the coil);
     - cut the joint ties;
     - slide each factory housing off its pigtail.

     The pigtails stay on J4 / J5 and ride up with the board.
  8. Unplug the bumper leads, the front cliff leads and the P03 leads.
  9. Lift the board.

  Fit new ties when it goes back, and re-tie the coils.

## 12. Robot behaviour rules (unchanged by the bay board)

- **No blind reversing.** The robot reverses only along the trail it has driven. Every simulator scenario audits this.
- **The circle rule for spins.** A turn in place is allowed only if every measured point is at least TURN_CLEAR (body radius +
  TURN_MARGIN) from base_link (`local_planner.turn_clear`).
- **Bumper and cliff behaviour are unchanged.**
  - A bumper press disarms the drive. The firmware re-arms only for a reverse with both wheels ≤ 0, which the controller drives only
    along its own trail, or for a pure spin toward the pressed side.
  - A cliff or tilt is a hard stop.

## 13. Simulator

On 2026-10-09 (bay B10) the simulator data were re-synced from `cad/mass-properties.json` at 76232c4:
- body 2.2838 kg, robot 2.4218 kg;
- robot COM x −18.48 mm (guard ≤ −15; the CAD worst case is −15.92);
- the body inertia tensor;
- the real bay board, modules, plugs, pigtails and joints in the visual meshes.

The Unreal pawn (rebuilt in B10 phase 2) takes its body inertia from `body_inertia_kg_m2`; the physics check reads it back exactly.
Its LiDAR collider is the turret-radius column from the lid top to the 155.15 mm robot top.

The first Unreal mapping benchmark with this robot entered all five rooms and mapped each at least 90 %. The robot did not report the
map complete within the 1500 s budget, the same way the earlier Unreal runs ended. Details are in the B10 report.

## 14. IMU calibration at bring-up

The GY-521 (MPU-6050) is not usable raw. Its datasheet allows a gyro zero-rate offset of up to ±20 °/s per axis, and accelerometer
zero-g offsets of ±50 mg (x, y) and ±80 mg (z). With the z offset alone the measured gravity can be 0.8 m/s² off 9.8: the controller's
gravity check would never pass, the attitude would never initialise, and the robot would stop with "tilted or attitude unknown". Bay
B11 added the calibration to the controller (`controller/autonomy/imu.py`) and tested it against a simulated board drawn near these
corners (`Content/RobotData/sensor-errors.json` in the simulator).

**Once, at bring-up** (and again after the bay board or the GY-521 is removed, re-seated or replaced):
1. **Where.** A flat, hard, plain floor (not a rug: the four cliff ToF sensors must see one plane), with 0.5 m clear all round.
2. **Warm up (preferred, not required).** About 10 min powered, Jetson running, puts the board near its working temperature. A cold
   calibration also works: the simulator calibrates cold, which is the worse case for the drift that follows. Either way the controller
   corrects the remaining temperature drift online, from the cliff floor plane, at every rest.
3. **Level, two headings.** The robot must be still with no motion command for 0.5 s; the motors may be on. Run the level
   calibration: `autonomy.imu.level_calibration`, driven by `controller/imu_level_calibration.py`. Today that tool reads the
   simulator; on the robot it must read the Pico's IMU stream, which is wired up with the firmware. Keep the robot still for the 5 s
   it records. Then turn it 180° by hand on the same spot and keep it still for 5 s more. The two headings cancel the floor's own tilt; the cliff ToF floor plane gives the
   reference attitude.
4. **Spin.** Motors on. The robot turns in place two full turns each way at 0.5 rad/s, so keep the 0.5 m clear. This measures how
   much of the yaw rate leaks into the x / y gyro (cross-axis sensitivity ±2 % plus the module's mounting tilt). The body's own
   vibration while spinning averages out over the two turns. The tilt of the spin axis, which the cliff floor plane sees, is taken
   out.
5. **Result.** `Content/RobotData/imu-calibration.json`: per-axis accelerometer offsets, the reference gravity, and the yaw coupling.
   The tool installs the file only if every check passes. Otherwise it writes `imu-calibration.rejected.json` and stops. The limits
   live in one place, `controller/autonomy/imu.py` (`LEVEL_HEADING_DIFF_LIMIT_M_S2`, `LEVEL_FLOOR_ATTITUDE_LIMIT_DEG`,
   `LEVEL_OFFSET_LIMIT_M_S2`, `YAW_COUPLING_LIMIT`); they are quoted here.

   **The real fault detectors:**
   - **Heading residuals.** The two headings' residuals must agree within 0.2 m/s² per axis. That is twice the floor's tilt times g,
     about 0.6°. A larger difference means the floor is not flat or the robot moved: move it and repeat.
   - **Floor attitude.** The cliff floor-plane attitude at rest must be ≤ 0.5°. More means the robot is not sitting flat on its wheels
     and caster (debris, an uneven spot) or a cliff ToF sensor reads off. Fix that rather than accept the numbers. A tilted GY-521 does
     not show here: it goes into the x / y offsets.

   **Gross-fault bounds:**
   - **Offsets:** |x|, |y| ≤ 0.75 and |z| ≤ 1.2 m/s². The stored offsets also absorb the accelerometer's sensitivity (±3 % of g) and
     cross-axis error (±2 % of g), so an in-spec MPU-6050 stores up to ±0.69 / ±0.69 / ±1.08 m/s² cold, and ±0.733 / ±0.733 /
     ±1.158 m/s² after the preferred 10-min warm-up (the zero-g temperature drift). Values beyond the bounds mean a broken sensor or a
     wrong range setting.
   - **Couplings:** ≤ 0.05 (cross-axis 2 % plus the mounting tilt).

**At every power-on** the robot calibrates the gyro bias itself. It must stand still for 2 s with no motion command; the motors may
be on. Do not touch it. It waits in its first measuring stop until this is done; if it never gets 2 s of stillness within 30 s it stops
with a message (Mapping and Go-to). After that, every stop re-estimates the bias (zero-velocity updates), and the
cliff floor plane keeps the accelerometer offsets right as the board warms.

**Firmware requirement (ruling PB27).**
- **Filter.** The Pico configures the MPU-6050 with **DLPF_CFG = 4**: gyro bandwidth 20 Hz, accelerometer 21 Hz.
- **Rate.** It forwards samples to the Jetson at **20 Hz**: either one sample per 20 Hz tick, or the samples in each 50 ms averaged.
- **Raw values.** Gyro at FS ±250 °/s, accelerometer at ±2 g, with no offset correction of its own.
- **Why it matters.** The controller's noise-dependent constants derive from these numbers (`IMU_DLPF_CFG`, `IMU_BANDWIDTH_HZ`,
  `IMU_RATE_HZ` in `imu.py`). Changing the firmware means changing them there.
- **Not calibrated.** The gyro scale factor (±3 %): the LiDAR corrects the heading every scan.
- **Floor-slope caveat.** The online offset correction treats the floor under the cliff sensors as level. Resting on a sloped floor
  it can learn up to ±0.5 m/s² (about 3°) of slope, with a 30 s time constant, and carry it into areas where the floor plane is
  invalid. That uses part of the tilt stop's 0.15 rad (8.6°) margin.
