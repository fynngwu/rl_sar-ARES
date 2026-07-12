#!/usr/bin/env python3
"""Monitor /dev/input/js0 raw events in real-time."""

import struct
import time
import sys
import os

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS   = 0x02
JS_EVENT_INIT   = 0x80

BUTTON_NAMES = {
    0: "A", 1: "B", 2: "X", 3: "Y",
    4: "LB", 5: "RB", 6: "Back", 7: "Start",
    8: "Guide", 9: "ThumbL", 10: "ThumbR",
}

AXIS_NAMES = {
    0: "LeftX", 1: "LeftY", 2: "LeftZ",
    3: "RightX", 4: "RightY", 5: "RightZ",
    6: "DPadX", 7: "DPadY",
}

# combo keys matching OnGamepadUpdate logic
prev = {}
combo_names = {
    "LB+Y":  (4, 3),
    "LB+A":  (4, 0),
    "LB+B":  (4, 1),
    "LB+X":  (4, 2),
    "RB+X":  (5, 2),
    "RB+A":  (5, 0),
    "RB+B":  (5, 1),
    "RB+Y":  (5, 3),
    "LB+RB": (4, 5),
}

JS_FORMAT = "IhBB"  # time(uint32), value(int16), type(uchar), number(uchar)
JS_SIZE = struct.calcsize(JS_FORMAT)

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "/dev/input/js0"
    if not os.path.exists(path):
        print(f"Device not found: {path}")
        sys.exit(1)

    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    print(f"Opened {path} — press buttons on gamepad, Ctrl+C to stop\n")
    print(f"{'Event':<12} {'Raw':>6}  {'Combo States'}")
    print("-" * 70)

    buttons = {}
    last_print = time.time()

    try:
        while True:
            try:
                data = os.read(fd, JS_SIZE * 10)
            except BlockingIOError:
                time.sleep(0.005)
                continue

            offset = 0
            while offset + JS_SIZE <= len(data):
                ts, value, eventType, number = struct.unpack_from(JS_FORMAT, data, offset)
                offset += JS_SIZE

                is_init = bool(eventType & JS_EVENT_INIT)
                etype = eventType & 0x7F

                if etype == JS_EVENT_BUTTON:
                    name = BUTTON_NAMES.get(number, f"Btn{number}")
                    state = "PRESS" if value else "RELEASE"
                    init_tag = " (INIT)" if is_init else ""
                    buttons[number] = value

                    combo_str = ""
                    active = []
                    for cname, (b1, b2) in combo_names.items():
                        if buttons.get(b1, 0) and buttons.get(b2, 0):
                            active.append(cname)
                    if active:
                        combo_str = f"  combos: {', '.join(active)}"

                    now = time.time()
                    dt = (now - last_print) * 1000
                    last_print = now
                    print(f"BTN {name:<6} {state:<8} val={value}{init_tag}{combo_str}  (+{dt:.1f}ms)")

                elif etype == JS_EVENT_AXIS:
                    name = AXIS_NAMES.get(number, f"Ax{number}")
                    norm = value / 32767.0
                    if abs(norm) > 0.1:
                        init_tag = " (INIT)" if is_init else ""
                        print(f"AXIS {name:<6} raw={value:>6} norm={norm:>+6.2f}{init_tag}")

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        os.close(fd)

if __name__ == "__main__":
    main()
