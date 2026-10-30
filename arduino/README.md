# Arduino: the wheel controller

`balance_bot_atmega/` is the firmware for the Arduino (Uno / Nano, ATmega328P) between the Jetson (USB) and the two
ZS-X11H motor drivers. It runs a 500 Hz speed loop per wheel from the drivers' speed pulses (SC) and talks to the Jetson's
`wheel_serial_bridge` over USB serial at 115200 baud.

`esp32_legacy/` is the older ESP32 version of the same firmware (same protocol), kept for reference.

## Wiring

Each ZS-X11H has a 5-pin signal header (PWM throttle, speed pulse out, GND, GND, 5V out) plus its DIR input:

| ZS-X11H | Right driver | Left driver |
|---|---|---|
| PWM throttle | D9 | D10 |
| Speed pulse out (SC) | D2 | D3 |
| GND (one of the two) | Arduino GND | Arduino GND |
| 5V out | not connected | not connected |
| DIR | D6 | D7 |

* Arduino Mega 2560: same pins; compile with `--fqbn arduino:avr:mega`. D9 / D10 are on its 8-bit Timer2 there
  (7.8 kHz, dithered to 10 bits).
* D9 / D10 are fixed: the only pins of the 16-bit timer (fast 10-bit PWM, 15.6 kHz). They connect straight to the
  throttle inputs. If a motor does not respond smoothly to `D 0.1 0.1` .. `D 0.5 0.5`, set `PWM_SLOW 1` (1.95 kHz).
* D2 / D3 are fixed: the two interrupt pins. The speed pulses carry no direction, so the firmware takes the sign from
  the direction it commands.
* 5V out stays unconnected: the Arduino is powered over USB from the Jetson, and two 5 V supplies must not be tied
  together. The GND connection is needed.
* If the driver has an onboard speed potentiometer, turn it to minimum.
* Recommended: a 10 kΩ resistor from D9 and from D10 to GND. While the Arduino boots or is being flashed, those pins
  float and the drivers can see noise as throttle.
* Output protection in the firmware (`DUTY_SLEW_PER_S`, `REVERSE_MAX_RAD_S`, `DIR_MIN_HOLD_MS`): throttle ramps up,
  DIR only reverses with the throttle at 0 and the wheel nearly stopped. Hard reversals at speed trip the ZS-X11H
  (violent vibration, then everything cuts out).
* Better feedback, if you can: tap each motor's three hall wires (in parallel with the driver) onto A0-A2 (left) and
  A3-A5 (right) and set `FEEDBACK` to `FEEDBACK_HALLS` in the sketch: direction-aware and what training assumes.

## Flashing (from the PC)

1. Arduino IDE: open `balance_bot_atmega/balance_bot_atmega.ino`.
2. Tools -> Board: **Arduino Uno** or **Arduino Nano** (Nano clones: Processor -> ATmega328P (Old Bootloader)).
3. Tools -> Port, then **Upload**.
4. Serial Monitor at 115200 baud, line ending "Newline": send `?`, it answers `# balance bot ATmega ...`.

Then plug it into the Jetson.

## First-time setup: measure the motors (wheels off the ground)

Drivers powered from the battery (the speed pulses come from the drivers), Arduino on USB, the Arduino IDE's Serial
Monitor closed. `motor_test.py` talks to the sketch directly, from the PC or the Jetson (no ROS, needs
`python3-serial`). Use the port from `ls /dev/ttyUSB* /dev/ttyACM*`.

1. **`COUNTS_PER_REV`** (pulses per wheel turn):

    ```bash
    python3 arduino/motor_test.py counts /dev/ttyUSB0
    ```

    Put a tape mark on the tyre, turn one wheel slowly by hand exactly a few full turns (more turns = more exact),
    press Enter and type how many. It prints the pulses per turn. Set it in the sketch (line ~83):
    `const float COUNTS_PER_REV = 45.0f;` (your number), and flash again.
2. **Directions + `MOTOR_MAX_RAD_S`** (speed at 100 % throttle):

    ```bash
    python3 arduino/motor_test.py speed /dev/ttyUSB0
    ```

    Spins both wheels at 20 % and 40 % throttle for 3 s each. A wheel turning backward (or reading negative): flip
    its `*_MOTOR_INVERT` and flash. Otherwise it prints `MOTOR_MAX_RAD_S` per wheel: set the sketch's
    `MOTOR_MAX_RAD_S` (line ~88) to their average and flash again. Do step 1 first: a wrong pulse count scales these
    speeds.
3. **Speed loop**: on the Jetson, `wheel_check` (`../jetson/README.md`, "Calibration") tests the closed loop, or
   drive the wheels with the controller in `MODE=wheel_test`: measured speed close to target.

Flash again after changing a setting. If you change the speed loop (`KP`, `KI`, `MOTOR_MAX_RAD_S`) or the pulse
count differs from 90, the drive model used for training (`training/source/.../mdp/hoverboard.py`) should match,
and the policy should be retrained.

## Motors don't move

**Which pin moves what?** Flash `pin_finder/` and run `pin_finder.py` (wheels off the ground):

```bash
arduino-cli compile --upload -p /dev/ttyUSB0 --fqbn arduino:avr:mega pin_finder
python3 pin_finder.py scan /dev/ttyUSB0        # PWM on each PWM pin in turn; you say what moved
python3 pin_finder.py dir /dev/ttyUSB0 9       # hold PWM on D9, toggle D4-D13: which one reverses that wheel?
python3 pin_finder.py set /dev/ttyUSB0 9 60    # hold one pin (0-255, or H / L) until Ctrl-C
```

It also counts the SC pulses on D2 / D3 while each pin is on. Flash `balance_bot_atmega` again afterwards.

Or a simpler check:

Flash `pwm_test/pwm_test.ino` (same pins, plain `analogWrite`, no speed loop, wheels off the ground) and watch the
Serial Monitor at 115200:

```bash
arduino-cli compile --upload -p /dev/ttyUSB0 --fqbn arduino:avr:mega pwm_test
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200
```

* Motors turn with `pwm_test` but not with `balance_bot_atmega`: firmware problem. Note that `balance_bot_atmega`
  only drives while commands keep coming (`motor_test.py`, or the ROS bridge): a single `D` / `V` from a serial
  monitor switches off again after 200 ms.
* Motors don't turn with `pwm_test` either: wiring / driver.
  * Battery on, driver LED on? Arduino GND connected to a driver GND?
  * Onboard speed potentiometer turned fully to minimum (anticlockwise). Some ZS-X11H versions only take an external
    PWM signal after a solder bridge / jumper next to the pot is closed, or with the pot removed: check the board.
  * Driver STOP / BRAKE / EN inputs, if your version has them: at the level that lets the motor run.
  * A multimeter on D9 / D10 against GND: ~1.2 V average at the test's 60/255 when "forward" / "backward", 0 when "off".
* SC pulses/s stay 0 while the wheel turns: SC wire, or try `HALL_PULLUPS = true` (in pwm_test: `INPUT_PULLUP`).
