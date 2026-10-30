#!/usr/bin/env python3
"""Find out which Arduino pin moves which motor, with the pin_finder sketch (arduino/pin_finder/) flashed.

WHEELS OFF THE GROUND, drivers powered, no serial monitor open:

    python3 pin_finder.py scan /dev/ttyUSB0          # PWM on each PWM pin in turn: which one spins which wheel?
    python3 pin_finder.py dir /dev/ttyUSB0 9         # hold PWM on D9, toggle other pins: which one reverses it?
    python3 pin_finder.py set /dev/ttyUSB0 9 60      # hold PWM 60/255 on D9 until Ctrl-C (H / L for digital)

The speed pulses (SC) on D2 / D3 are counted while each pin is on, so a turning wheel shows up even if you miss it.
D0 / D1 (USB) and D2 / D3 (SC inputs, never driven) are skipped. Needs pyserial (sudo apt install python3-serial).
Ctrl-C stops everything at any time (the sketch also switches off by itself 0.5 s after the last command).
"""

import argparse
import sys
import time

import serial

SKIP = {0, 1, 2, 3}


class Board:
    def __init__(self, port: str, baud: int = 115200):
        self.ser = serial.Serial(port, baud, timeout=0.3)
        time.sleep(2.0)  # opening the port resets the board
        self.ser.reset_input_buffer()

    def cmd(self, text: str) -> str:
        self.ser.write((text + "\n").encode())
        while True:
            line = self.ser.readline().decode(errors="replace").strip()
            if not line:
                sys.exit(f"no answer to '{text}': pin_finder sketch flashed? right port? serial monitor closed?")
            if not line.startswith("#"):
                return line

    def pwm_pins(self) -> list[int]:
        reply = self.cmd("?")
        if not reply.startswith("P"):
            sys.exit(f"unexpected answer '{reply}': is the pin_finder sketch flashed?")
        return [int(p) for p in reply.split()[1:]]

    def hold(self, commands: list[str], seconds: float) -> tuple[int, int]:
        """Keep sending the commands for `seconds` (the sketch times out otherwise); return the SC pulses on D2, D3."""
        self.cmd("C")
        end = time.time() + seconds
        while time.time() < end:
            for c in commands:
                reply = self.cmd(c)
                if reply.startswith("err"):
                    sys.exit(f"'{c}': {reply}")
            time.sleep(0.1)
        d2, d3 = (int(x) for x in self.cmd("C").split()[1:])
        self.cmd("O")
        return d2, d3

    def off(self):
        try:
            self.ser.write(b"O\n")
            self.ser.close()
        except Exception:
            pass


def pulses_text(d2: int, d3: int) -> str:
    return f"SC pulses: D2 {d2:5d}  D3 {d3:5d}" + ("   <- a wheel turned" if d2 + d3 > 4 else "")


def scan(board: Board, args):
    pins = [p for p in board.pwm_pins() if p not in SKIP]
    print(f"PWM pins: {' '.join(f'D{p}' for p in pins)}")
    print(f"Each one gets PWM {args.pwm}/255 for {args.seconds} s, the rest stay off.\n")
    results = []
    for pin in pins:
        input(f"D{pin}: press Enter to start (Ctrl-C quits) ")
        d2, d3 = board.hold([f"A {pin} {args.pwm}"], args.seconds)
        print("   " + pulses_text(d2, d3))
        seen = input("   what moved? (Enter = nothing, e.g. 'left', 'right', 'both') ").strip()
        results.append((pin, seen, d2, d3))
    print("\nSummary:")
    for pin, seen, d2, d3 in results:
        if seen or d2 + d3 > 4:
            print(f"  D{pin:<3d} {seen or '(not seen)':12s} {pulses_text(d2, d3)}")
    if not any(seen or d2 + d3 > 4 for _, seen, d2, d3 in results):
        print("  nothing moved: check driver power, GND to the Arduino, the onboard pot (fully anticlockwise),"
              " and try a higher --pwm")


def direction(board: Board, args):
    candidates = [p for p in (args.pins or range(4, 14)) if p not in SKIP and p != args.pwm_pin]
    print(f"Holding PWM {args.pwm}/255 on D{args.pwm_pin}; each candidate DIR pin goes LOW for {args.seconds} s,"
          f" then HIGH for {args.seconds} s. Watch the wheel's direction.\n")
    found = []
    for pin in candidates:
        input(f"D{pin}: press Enter to start (Ctrl-C quits) ")
        a = f"A {args.pwm_pin} {args.pwm}"
        print("   LOW ...", end="", flush=True)
        board.hold([a, f"L {pin}"], args.seconds)
        print(" HIGH ...", flush=True)
        board.hold([a, f"H {pin}"], args.seconds)
        if input("   did the wheel change direction? [y/N] ").strip().lower().startswith("y"):
            found.append(pin)
    print(f"\nDIR pin(s) for the wheel on D{args.pwm_pin}: {' '.join(f'D{p}' for p in found) or 'none found'}")


def hold_one(board: Board, args):
    level = args.value.upper()
    command = f"{level} {args.pin}" if level in ("H", "L") else f"A {args.pin} {int(level)}"
    print(f"'{command}' until Ctrl-C ...")
    while True:
        d2, d3 = board.hold([command], 1.0)
        print("   " + pulses_text(d2, d3))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)
    s = sub.add_parser("scan", help="PWM on each PWM pin in turn")
    s.add_argument("port")
    d = sub.add_parser("dir", help="find the DIR pin of the wheel on a PWM pin")
    d.add_argument("port")
    d.add_argument("pwm_pin", type=int)
    d.add_argument("--pins", type=int, nargs="+", help="candidate DIR pins (default D4-D13)")
    h = sub.add_parser("set", help="hold one pin: PWM value 0-255, or H / L")
    h.add_argument("port")
    h.add_argument("pin", type=int)
    h.add_argument("value")
    for p in (s, d):
        p.add_argument("--pwm", type=int, default=60, help="PWM value 0-255 (default 60, ~25 %%)")
        p.add_argument("--seconds", type=float, default=2.0)
    args = ap.parse_args()
    if getattr(args, "pin", None) in SKIP or getattr(args, "pwm_pin", None) in SKIP:
        sys.exit("D0-D3 are the USB serial and the SC inputs: not driven")
    board = Board(args.port)
    try:
        {"scan": scan, "dir": direction, "set": hold_one}[args.mode](board, args)
    except KeyboardInterrupt:
        print()
    finally:
        board.off()


if __name__ == "__main__":
    main()
