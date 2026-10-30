#!/usr/bin/env python3
"""Measure COUNTS_PER_REV and MOTOR_MAX_RAD_S for the wheel controller sketch, over USB serial (no ROS needed).

Wheels off the ground, drivers powered (36 V), sketch flashed, Arduino IDE's Serial Monitor closed:

    python3 arduino/motor_test.py counts /dev/ttyUSB0    # turn the wheels by hand, read the pulses per turn
    python3 arduino/motor_test.py speed  /dev/ttyUSB0    # spins the wheels at 20 % and 40 % throttle
    python3 arduino/motor_test.py run    /dev/ttyUSB0 --duty 0.02 --seconds 1 --wheel left
                                                         # fixed throttle (here 2 %) for a while, prints the speed

Needs pyserial (sudo apt install python3-serial). Ctrl-C stops (the motors switch off).
"""

import argparse
import re
import sys
import time

import serial


def open_port(port: str, baud: int) -> serial.Serial:
    ser = serial.Serial(port, baud, timeout=0.05)
    time.sleep(2.0)  # opening the port resets an Uno / Nano: wait for the bootloader
    ser.reset_input_buffer()
    return ser


def lines(ser: serial.Serial):
    buf = b""
    while True:
        buf += ser.read(256)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            yield line.decode(errors="replace").strip()
        if not buf:
            yield None


def read_counts(ser: serial.Serial) -> dict:
    """Send '?' and return {'left': count, 'right': count}."""
    ser.reset_input_buffer()
    ser.write(b"?\n")
    counts, end = {}, time.time() + 1.0
    for line in lines(ser):
        if time.time() > end:
            break
        m = line and re.match(r"# (left|right): count (-?\d+)", line)
        if m:
            counts[m.group(1)] = int(m.group(2))
            if len(counts) == 2:
                break
    if len(counts) != 2:
        sys.exit("no answer to '?': wrong port or baud, sketch not flashed, or Serial Monitor still open")
    return counts


def counts_test(ser: serial.Serial):
    print("Put a tape mark on each tyre. Turn ONE wheel slowly, exactly N full turns, forward or backward.")
    print("Counts update below; press Enter when done (Ctrl-C to quit).\n")
    start = read_counts(ser)
    import select

    while True:
        now = read_counts(ser)
        d = {k: now[k] - start[k] for k in now}
        print(f"\r  left {d['left']:+6d}   right {d['right']:+6d}   ", end="", flush=True)
        if select.select([sys.stdin], [], [], 0.2)[0]:
            sys.stdin.readline()
            break
    turns = float(input("\nHow many full turns did you make? ") or "1")
    for side in ("left", "right"):
        if d[side]:
            print(f"  {side}: {abs(d[side]) / turns:.1f} pulses per turn")
    print("\nSet COUNTS_PER_REV in the sketch to that number (e.g. 45.0f), flash, and check the other wheel the same way.")
    print("Nothing counted? Driver powered? GND connected? Then try HALL_PULLUPS = true.")


def spin(ser: serial.Serial, duty: float, seconds: float) -> tuple[float, float]:
    """Hold an open-loop duty ("D", resent every 50 ms), return the mean measured [left, right] speed of the last half."""
    samples, t0 = [], time.time()
    reader = lines(ser)
    while time.time() - t0 < seconds:
        ser.write(f"D {duty:.3f} {duty:.3f}\n".encode())
        t_send = time.time()
        while time.time() - t_send < 0.05:
            line = next(reader)
            m = line and re.match(r"W (-?[\d.]+) (-?[\d.]+)", line)
            if m and time.time() - t0 > seconds / 2:
                samples.append((float(m.group(1)), float(m.group(2))))
    ser.write(b"S\n")
    if not samples:
        sys.exit("no 'W' speed reports from the Arduino")
    n = len(samples)
    return sum(s[0] for s in samples) / n, sum(s[1] for s in samples) / n


def speed_test(ser: serial.Serial):
    print("Spinning both wheels: 20 % throttle, then 40 % (3 s each). Wheels off the ground!")
    input("Press Enter to start (Ctrl-C to quit) ")
    duties = (0.2, 0.4)
    results = []
    for duty in duties:
        left, right = spin(ser, duty, 3.0)
        results.append((left, right))
        print(f"  throttle {duty * 100:3.0f} %: left {left:+6.2f} rad/s   right {right:+6.2f} rad/s")
        time.sleep(1.0)
    for i, side in enumerate(("left", "right")):
        w1, w2 = results[0][i], results[1][i]
        if w1 <= 0 or w2 <= 0:
            print(f"  {side}: turns backward or reads negative -> flip {side.upper()}_MOTOR_INVERT, flash, run again")
            continue
        # speed is about linear in throttle above the friction offset: the slope between the two points is the
        # speed per unit throttle, i.e. the speed at 100 % without load
        slope = (w2 - w1) / (duties[1] - duties[0])
        print(f"  {side}: MOTOR_MAX_RAD_S ~ {slope:.1f} rad/s")
    print("\nSet MOTOR_MAX_RAD_S to the average of the two (both wheels share it). Check COUNTS_PER_REV first:")
    print("a wrong pulse count scales these speeds by the same factor. The training assumes about 51-75 rad/s at 33-41 V;")
    print("far outside that, the drive model needs updating and the policy retraining.")


def run_test(ser: serial.Serial, duty: float, seconds: float, wheel: str):
    if not -1.0 <= duty <= 1.0:
        sys.exit("--duty is a fraction: 0.02 = 2 %, negative = backward")
    left = duty if wheel in ("left", "both") else 0.0
    right = duty if wheel in ("right", "both") else 0.0
    print(f"Throttle {duty * 100:+.1f} % on {wheel} wheel(s) for {seconds} s. Wheels off the ground!")
    input("Press Enter to start (Ctrl-C stops) ")
    reader = lines(ser)
    t0 = last_print = time.time()
    peak = [0.0, 0.0]
    while time.time() - t0 < seconds:
        ser.write(f"D {left:.4f} {right:.4f}\n".encode())  # resent every 50 ms (the sketch stops after 200 ms)
        t_send = time.time()
        while time.time() - t_send < 0.05:
            line = next(reader)
            m = line and re.match(r"W (-?[\d.]+) (-?[\d.]+)", line)
            if m:
                w = (float(m.group(1)), float(m.group(2)))
                peak = [max(peak[i], abs(w[i])) for i in range(2)]
                if time.time() - last_print >= 0.1:
                    print(f"  t {time.time() - t0:4.1f} s   left {w[0]:+6.2f} rad/s   right {w[1]:+6.2f} rad/s")
                    last_print = time.time()
    ser.write(b"S\n")
    print(f"stopped. Peak speed: left {peak[0]:.2f}  right {peak[1]:.2f} rad/s"
          + ("   (0 = didn't turn, or no speed pulses: try a bit more --duty)" if max(peak) == 0 else ""))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("test", choices=("counts", "speed", "run"))
    ap.add_argument("port", help="e.g. /dev/ttyUSB0 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--duty", type=float, default=0.02, help="run: throttle fraction, 0.02 = 2 %%, negative = backward")
    ap.add_argument("--seconds", type=float, default=1.0, help="run: how long (default 1 s)")
    ap.add_argument("--wheel", choices=("left", "right", "both"), default="both", help="run: which wheel")
    args = ap.parse_args()
    ser = open_port(args.port, args.baud)
    try:
        if args.test == "counts":
            counts_test(ser)
        elif args.test == "speed":
            speed_test(ser)
        else:
            run_test(ser, args.duty, args.seconds, args.wheel)
    except KeyboardInterrupt:
        print()
    finally:
        ser.write(b"S\n")
        ser.close()


if __name__ == "__main__":
    main()
