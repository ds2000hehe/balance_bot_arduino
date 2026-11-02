#!/usr/bin/env python3
"""Measure COUNTS_PER_REV and MOTOR_MAX_RAD_S for the wheel controller sketch, over USB serial (no ROS needed).

Wheels off the ground, drivers powered (36 V), sketch flashed, Arduino IDE's Serial Monitor closed:

    python3 arduino/motor_test.py counts /dev/ttyUSB0    # turn the wheels by hand, read the pulses per turn
    python3 arduino/motor_test.py speed  /dev/ttyUSB0    # spins the wheels at 20 % and 40 % throttle
    python3 arduino/motor_test.py run    /dev/ttyUSB0 --duty 0.02 --seconds 1 --wheel left
                                                         # fixed throttle (here 2 %) for a while, prints the speed
    python3 arduino/motor_test.py loop   /dev/ttyUSB0    # speed loop: targets 5, 15, -5, 0 rad/s, target vs measured
    python3 arduino/motor_test.py brake  /dev/ttyUSB0    # BRAKE wiring (A4 / A5): which pin brakes which wheel,
                                                         # polarity, and the automatic braking
    python3 arduino/motor_test.py brakecal /dev/ttyUSB0  # brake strength of each wheel at 25-100 %: gains that
                                                         # make both wheels brake the same

Needs pyserial (sudo apt install python3-serial). Ctrl-C stops (the motors switch off).
"""

import argparse
import math
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


def loop_test(ser: serial.Serial, targets: list[float], seconds: float):
    print(f"Speed loop: targets {', '.join(f'{t:g}' for t in targets)} rad/s, {seconds} s each, both wheels.")
    print("Wheels off the ground! A good loop reaches the target within ~0.3 s and holds it without buzzing.")
    input("Press Enter to start (Ctrl-C stops) ")
    reader = lines(ser)
    summary = []
    for target in targets:
        t0 = last_print = time.time()
        samples = []
        while time.time() - t0 < seconds:
            ser.write(f"V {target:.3f} {target:.3f}\n".encode())  # resent every 20 ms like the ROS bridge
            t_send = time.time()
            while time.time() - t_send < 0.02:
                line = next(reader)
                m = line and re.match(r"W (-?[\d.]+) (-?[\d.]+)", line)
                if not m:
                    continue
                w = (float(m.group(1)), float(m.group(2)))
                if time.time() - t0 > seconds / 2:
                    samples.append(w)
                if time.time() - last_print >= 0.2:
                    print(f"  target {target:+6.1f}   t {time.time() - t0:3.1f} s   left {w[0]:+6.2f}   right {w[1]:+6.2f}")
                    last_print = time.time()
        if samples:
            n = len(samples)
            mean = [sum(x[i] for x in samples) / n for i in range(2)]
            spread = [max(x[i] for x in samples) - min(x[i] for x in samples) for i in range(2)]
            summary.append((target, mean, spread))
    ser.write(b"S\n")
    print("\nSecond half of each step (mean, max-min):")
    for target, mean, spread in summary:
        print(f"  target {target:+6.1f}:  left {mean[0]:+6.2f} ({spread[0]:.1f})   right {mean[1]:+6.2f} ({spread[1]:.1f})")
    print("Means within ~10 % of the target (0 at 0) and small spread = good. Way off or wildly swinging: tell Claude.")


def hold(ser: serial.Serial, reader, left: float, right: float, seconds: float, wheel: int) -> list[tuple[float, float]]:
    """Send "D left right" every 20 ms for a while; return (t, |speed|) of one wheel (0 = left, 1 = right)."""
    out, t0 = [], time.time()
    while time.time() - t0 < seconds:
        ser.write(f"D {left:.3f} {right:.3f}\n".encode())
        t_send = time.time()
        while time.time() - t_send < 0.02:
            line = next(reader)
            m = line and re.match(r"W (-?[\d.]+) (-?[\d.]+)", line)
            if m:
                out.append((time.time() - t0, abs(float(m.group(1 + wheel)))))
    return out


def slowdown(samples: list[tuple[float, float]], v0: float) -> tuple[float, float]:
    """Speed after 0.3 s and time to fall to half of v0."""
    at = [v for t, v in samples if t >= 0.3]
    half = next((t for t, v in samples if v <= 0.5 * v0), float("nan"))
    return (at[0] if at else float("nan")), half


def brake_test(ser: serial.Serial, duty: float):
    print(f"BRAKE wiring check: each wheel spins at {duty * 100:.0f} % throttle, then the throttle drops and the wheel")
    print("either coasts or is braked. Wheels off the ground!")
    input("Press Enter to start (Ctrl-C stops) ")
    reader = lines(ser)
    for wheel, name in enumerate(("left", "right")):
        trials = [("coast (no brake)", "B 0", "K -1 -1", 0.0),
                  ("brake pin of this wheel", "B 0", "K 1 0" if wheel == 0 else "K 0 1", 0.0),
                  ("brake pin of the OTHER wheel", "B 0", "K 0 1" if wheel == 0 else "K 1 0", 0.0),
                  (f"automatic (throttle to {duty / 4 * 100:.1f} %)", "B 1", "K -1 -1", duty / 4)]
        print(f"\n{name} wheel:")
        result = {}
        for label, auto, manual, low in trials:
            ser.write(b"K -1 -1\nB 0\n")
            d = (duty, 0.0) if wheel == 0 else (0.0, duty)
            spun = hold(ser, reader, *d, 1.5, wheel)
            v0 = sum(v for t, v in spun if t > 1.0) / max(1, sum(1 for t, v in spun if t > 1.0))
            ser.write(f"{auto}\n{manual}\n".encode())
            d = (low, 0.0) if wheel == 0 else (0.0, low)
            after = hold(ser, reader, *d, 1.2, wheel)
            ser.write(b"K -1 -1\nB 1\nS\n")
            v03, half = slowdown(after, v0)
            result[label] = v03
            print(f"  {label:34s}: {v0:5.1f} rad/s -> {v03:5.1f} after 0.3 s, half speed after "
                  + (f"{half * 1000:4.0f} ms" if half == half else "  > 1.2 s"))
            time.sleep(2.0)  # coast to a stop
        if v0 < 2.0:
            print(f"  -> the {name} wheel did not spin up (driver powered? try a higher --duty)")
            continue
        coast, own, other = (result[k] for k in list(result)[:3])
        def faster(x):  # speed after 0.3 s clearly below coasting's
            return x == x and coast == coast and x < 0.5 * coast
        if faster(own):
            print(f"  -> {name} brake works, pin and polarity right")
        elif faster(other):
            print(f"  -> the other wheel's pin brakes this wheel: set SWAP_BRAKE_PINS to 1 in the sketch")
        else:
            print("  -> no braking from either pin: flip BRAKE_ACTIVE_HIGH in the sketch and run again; still nothing:"
                  " check the BRAKE wires (and that they share GND with the Arduino)")
    ser.write(b"K -1 -1\nB 1\nS\n")
    print("\nThe automatic line should slow the wheel about as fast as the brake pin line (to the lower throttle's speed).")


def decay_rate(samples: list[tuple[float, float]], v0: float) -> float:
    """Exponential slow-down rate [1/s] (shorted motor: torque ~ speed, so v ~ exp(-rate t)) from ln(v) vs t."""
    pts = [(t, v) for t, v in samples if 0.02 <= t and v > max(3.0, 0.15 * v0)]
    if len(pts) < 3:
        pts = [(t, v) for t, v in samples if 0.0 <= t <= 0.1 and v > 0.5]
    if len(pts) < 2:
        return float("nan")
    n = len(pts)
    mt = sum(t for t, _ in pts) / n
    ml = sum(math.log(v) for _, v in pts) / n
    num = sum((t - mt) * (math.log(v) - ml) for t, v in pts)
    den = sum((t - mt) ** 2 for t, _ in pts)
    return -num / den if den > 0 else float("nan")


def speed_at(samples: list[tuple[float, float]], t: float) -> float:
    """Speed at time t after the throttle drop (linear interpolation of the reports)."""
    for (t0, v0), (t1, v1) in zip(samples, samples[1:]):
        if t0 <= t <= t1:
            return v0 + (v1 - v0) * (t - t0) / (t1 - t0) if t1 > t0 else v0
    return samples[-1][1] if samples and t > samples[-1][0] else float("nan")


PROFILE_T = (0.05, 0.1, 0.15, 0.2, 0.3)


def brake_trial(ser, reader, wheel: int, duty: float, frac: float, hz: float) -> tuple[float, list[float]]:
    """Spin one wheel up, then throttle 0 with brake fraction frac at pulse rate hz. Returns (v0, speeds at PROFILE_T)."""
    ser.write(f"K -1 -1\nB 0\nF {hz:.0f}\n".encode())
    d = (duty, 0.0) if wheel == 0 else (0.0, duty)
    spun = hold(ser, reader, *d, 1.5, wheel)
    tail = [v for t, v in spun if t > 1.0]
    v0 = sum(tail) / max(1, len(tail))
    k = (frac, 0.0) if wheel == 0 else (0.0, frac)
    ser.write(f"K {k[0]:.2f} {k[1]:.2f}\n".encode())
    after = hold(ser, reader, 0.0, 0.0, 0.4, wheel)
    ser.write(b"K -1 -1\nB 1\nS\n")
    time.sleep(1.5)
    return v0, [speed_at(after, t) / v0 if v0 > 0 else float("nan") for t in PROFILE_T]


def mean_profile(trials):
    n = len(trials)
    return [sum(tr[1][i] for tr in trials) / n for i in range(len(PROFILE_T))]


def brake_strength(profile, coast) -> float:
    """Extra speed lost by 150 ms compared with coasting, as a fraction of the starting speed."""
    i = PROFILE_T.index(0.15)
    return coast[i] - profile[i]


def brakecal_test(ser: serial.Serial, duty: float, repeats: int):
    fractions = (0.25, 0.5, 0.75, 1.0)
    rates = (25, 50, 100, 200, 500)
    print(f"Brake calibration: each wheel spins at {duty * 100:.0f} % throttle, then coasts or brakes. Part 1: brake "
          f"25-100 % at 200 Hz pulses; part 2: 50 % brake at {', '.join(map(str, rates))} Hz pulses. Wheels off the "
          f"ground! ~{int((2 * 5 * repeats + 2 * len(rates)) * 3.5)} s")
    input("Press Enter to start (Ctrl-C stops) ")
    reader = lines(ser)
    head = "  ".join(f"{int(t * 1000):4d}ms" for t in PROFILE_T)
    full, coast_of = {}, {}
    for wheel, name in enumerate(("left", "right")):
        print(f"\n{name} wheel, part 1: speed (fraction of the start speed) after the throttle drop")
        print(f"                 start   {head}   extra slowing by 150 ms")
        prof = {}
        for frac in (0.0,) + fractions:
            trials = [brake_trial(ser, reader, wheel, duty, frac, 200) for _ in range(repeats)]
            prof[frac] = mean_profile(trials)
            v0 = sum(tr[0] for tr in trials) / len(trials)
            extra = "" if frac == 0.0 else f"   {brake_strength(prof[frac], prof[0.0]):5.2f}"
            label = "coast" if frac == 0.0 else f"brake {frac:4.0%}"
            print(f"  {label:10s}: {v0:5.1f}   " + "  ".join(f"{x:6.2f}" for x in prof[frac]) + extra)
        coast_of[name] = prof[0.0]
        full[name] = brake_strength(prof[1.0], prof[0.0])
        print(f"  -> full brake removes {full[name]:.0%} of the speed by 150 ms beyond coasting; "
              f"ideal for 25/50/75 %: about 1/4, 1/2, 3/4 of that")
    print("\nPart 2: 50 % brake at different pulse rates (ideal: half of the full brake's extra slowing)")
    best = {}
    for wheel, name in enumerate(("left", "right")):
        for hz in rates:
            trials = [brake_trial(ser, reader, wheel, duty, 0.5, hz)]
            st = brake_strength(mean_profile(trials), coast_of[name])
            ratio = st / full[name] if full[name] > 0 else float("nan")
            best.setdefault(hz, []).append(ratio)
            print(f"  {name:5s} {hz:4d} Hz: extra slowing {st:5.2f} = {ratio:4.2f} x full brake")
    ser.write(b"K -1 -1\nB 1\nF 200\nS\n")
    score = {hz: abs(sum(r) / len(r) - 0.5) for hz, r in best.items() if all(x == x for x in r)}
    sl, sr = full["left"], full["right"]
    print()
    if score:
        hz = min(score, key=score.get)
        print(f"Most proportional pulse rate: {hz} Hz (50 % brake gives {sum(best[hz]) / len(best[hz]):.2f} x full).")
        print(f"    brake_pwm_hz: {hz}")
    if sl > 0 and sr > 0:
        weak = min(sl, sr)
        print(f"Full brake: left {sl:.2f}, right {sr:.2f} (left {sl / sr:.2f}x right). Equal braking:")
        print(f"    brake_gain_left: {weak / sl:.2f}")
        print(f"    brake_gain_right: {weak / sr:.2f}")
        print("Put these in jetson/config/robot.yaml (wheel_serial_bridge). Within ~10 % (gains > 0.9) is fine as is.")
    else:
        print("No usable full-brake measurement: did the wheels spin up? Is the brake wired (motor_test.py brake)?")
    print("Paste this output to Claude.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("test", choices=("counts", "speed", "run", "loop", "brake", "brakecal"))
    ap.add_argument("--repeats", type=int, default=2, help="brakecal: runs per brake strength")
    ap.add_argument("port", help="e.g. /dev/ttyUSB0 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--duty", type=float, default=None,
                    help="run: throttle fraction, 0.02 = 2 %% (default), negative = backward; brake: spin throttle (0.3)")
    ap.add_argument("--seconds", type=float, default=1.0, help="run: how long (default 1 s)")
    ap.add_argument("--targets", type=float, nargs="+", default=[5, 15, -5, 0], help="loop: speeds in rad/s")
    ap.add_argument("--step-seconds", type=float, default=2.0, help="loop: seconds per target")
    ap.add_argument("--wheel", choices=("left", "right", "both"), default="both", help="run: which wheel")
    args = ap.parse_args()
    ser = open_port(args.port, args.baud)
    try:
        if args.test == "counts":
            counts_test(ser)
        elif args.test == "speed":
            speed_test(ser)
        elif args.test == "run":
            run_test(ser, 0.02 if args.duty is None else args.duty, args.seconds, args.wheel)
        elif args.test == "brake":
            brake_test(ser, 0.3 if args.duty is None else args.duty)
        elif args.test == "brakecal":
            brakecal_test(ser, 0.3 if args.duty is None else args.duty, args.repeats)
        else:
            loop_test(ser, args.targets, args.step_seconds)
    except KeyboardInterrupt:
        print()
    finally:
        ser.write(b"S\n")
        ser.close()


if __name__ == "__main__":
    main()
