#!/usr/bin/env python3
import argparse
import select
import sys
import termios
import time
import tty

from pymavlink import mavutil


def clamp(value, lo=-1.0, hi=1.0):
    return max(lo, min(hi, value))


def axis_to_mavlink(value):
    return int(round(clamp(value) * 1000.0))


def heave_to_mavlink(value, z_mode):
    value = clamp(value)
    if z_mode == "centered":
        return int(round((value + 1.0) * 500.0))
    return axis_to_mavlink(value)


def read_key(timeout):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return None
    return sys.stdin.read(1)


def decay(value, rate):
    if value > 0:
        return max(0.0, value - rate)
    if value < 0:
        return min(0.0, value + rate)
    return 0.0


def send_manual_control(master, target_system, surge, sway, heave, yaw, z_mode):
    master.mav.manual_control_send(
        target_system,
        axis_to_mavlink(surge),
        axis_to_mavlink(sway),
        heave_to_mavlink(heave, z_mode),
        axis_to_mavlink(yaw),
        0,
    )


def main():
    parser = argparse.ArgumentParser(description="Send MAVLink MANUAL_CONTROL test input over UDP.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=14550)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--step", type=float, default=0.25)
    parser.add_argument("--decay", type=float, default=0.05)
    parser.add_argument("--target-system", type=int, default=1)
    parser.add_argument("--source-system", type=int, default=255)
    parser.add_argument("--z-mode", choices=("signed", "centered"), default="signed")
    args = parser.parse_args()

    master = mavutil.mavlink_connection(
        f"udpout:{args.host}:{args.port}",
        source_system=args.source_system,
    )

    period = 1.0 / max(1.0, args.rate)
    surge = 0.0
    sway = 0.0
    heave = 0.0
    yaw = 0.0

    print(f"Sending MANUAL_CONTROL to udp://{args.host}:{args.port}")
    print("Keys: w/s surge, a/d sway, r/f heave, q/e yaw, space zero, x exit")

    old_term = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        next_heartbeat = 0.0

        while True:
            now = time.time()
            if now >= next_heartbeat:
                master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    0,
                )
                next_heartbeat = now + 1.0

            key = read_key(0.0)
            if key == "x" or key == "\x1b":
                break
            if key == " ":
                surge = sway = heave = yaw = 0.0
            elif key == "w":
                surge = clamp(surge + args.step)
            elif key == "s":
                surge = clamp(surge - args.step)
            elif key == "d":
                sway = clamp(sway + args.step)
            elif key == "a":
                sway = clamp(sway - args.step)
            elif key == "r":
                heave = clamp(heave + args.step)
            elif key == "f":
                heave = clamp(heave - args.step)
            elif key == "e":
                yaw = clamp(yaw + args.step)
            elif key == "q":
                yaw = clamp(yaw - args.step)
            elif key is None:
                surge = decay(surge, args.decay)
                sway = decay(sway, args.decay)
                heave = decay(heave, args.decay)
                yaw = decay(yaw, args.decay)

            send_manual_control(master, args.target_system, surge, sway, heave, yaw, args.z_mode)
            print(
                f"\rsurge={surge:+.2f} sway={sway:+.2f} heave={heave:+.2f} yaw={yaw:+.2f}",
                end="",
                flush=True,
            )
            time.sleep(period)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)
        send_manual_control(master, args.target_system, 0.0, 0.0, 0.0, 0.0, args.z_mode)
        print("\nStopped.")


if __name__ == "__main__":
    main()
