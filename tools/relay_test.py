#!/usr/bin/env python3
"""Bench test for the mower's relay board, with no mower wiring attached.

Sets only the relay parameters, reboots if needed, then lets you switch R1, R2 and R3.
Close QGroundControl first: only one program can use the USB port at a time.

    python3 -m venv .venv && . .venv/bin/activate && pip install pymavlink pyserial
    python3 tools/relay_test.py
    python3 tools/relay_test.py --port /dev/cu.usbmodem101
"""

import argparse
import glob
import sys
import time

from pymavlink import mavutil

PARAMS = [
    ("SERVO9_FUNCTION", -1),
    ("SERVO10_FUNCTION", -1),
    ("SERVO11_FUNCTION", -1),
    ("RELAY1_FUNCTION", 1),
    ("RELAY2_FUNCTION", 1),
    ("RELAY3_FUNCTION", 1),
    ("RELAY1_PIN", 50),
    ("RELAY2_PIN", 51),
    ("RELAY3_PIN", 52),
    ("RELAY1_DEFAULT", 0),
    ("RELAY2_DEFAULT", 0),
    ("RELAY3_DEFAULT", 0),
    ("RELAY1_INVERTED", 1),
    ("RELAY2_INVERTED", 1),
    ("RELAY3_INVERTED", 1),
]
ROVER_TYPES = (mavutil.mavlink.MAV_TYPE_GROUND_ROVER, mavutil.mavlink.MAV_TYPE_SURFACE_BOAT)


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*"))
    if not ports:
        sys.exit("No Pixhawk port found. Pass --port.")
    # ArduPilot shows two USB ports. The first is the MAVLink one.
    return ports[0]


def connect(port):
    conn = mavutil.mavlink_connection(port, baud=115200)
    # ArduPilot only: GCS heartbeats from other sources are ignored here.
    deadline = time.time() + 30
    while time.time() < deadline:
        hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            conn.target_system = hb.get_srcSystem()
            conn.target_component = hb.get_srcComponent()
            if hb.type not in ROVER_TYPES:
                sys.exit(f"This is not Rover firmware (MAV_TYPE {hb.type}). Flash ArduRover first.")
            if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                sys.exit("Vehicle is armed. Disarm before running this test.")
            return conn
    sys.exit("No ArduPilot heartbeat. Is QGroundControl still open, or is the firmware missing?")


def read_param(conn, name):
    conn.mav.param_request_read_send(conn.target_system, conn.target_component,
                                     name.encode(), -1)
    deadline = time.time() + 2
    while time.time() < deadline:
        msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg and msg.param_id.rstrip("\x00") == name:
            return msg.param_value
    return None


def write_param(conn, name, value):
    for _ in range(3):
        conn.mav.param_set_send(conn.target_system, conn.target_component, name.encode(),
                                float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.2)
        if read_param(conn, name) == float(value):
            return True
    return False


def send_command(conn, command, *params):
    params = list(params) + [0] * (7 - len(params))
    conn.mav.command_long_send(conn.target_system, conn.target_component, command, 0, *params)
    deadline = time.time() + 3
    while time.time() < deadline:
        ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if ack and ack.command == command:
            return ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    return False


def reboot(conn, port):
    print("Rebooting the Pixhawk...")
    send_command(conn, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1)
    conn.close()
    time.sleep(3)
    deadline = time.time() + 30
    while time.time() < deadline:
        try:
            return connect(port)
        except (OSError, mavutil.mavlink.MAVError):
            time.sleep(1)
    sys.exit("The Pixhawk did not come back after rebooting.")


def setup(conn, port):
    """Parameters appear only after their enable parameter is set and the board reboots."""
    for attempt in range(3):
        changed, missing = False, []
        for name, value in PARAMS:
            current = read_param(conn, name)
            if current is None:
                missing.append(name)
            elif current != float(value):
                if not write_param(conn, name, value):
                    sys.exit(f"Could not set {name}.")
                print(f"  {name}: {current:g} -> {value}")
                changed = True
        if not changed and not missing:
            print("Relay parameters are correct.")
            return conn
        conn = reboot(conn, port)
    sys.exit(f"Relay parameters still wrong after reboots. Missing: {missing}")


def set_relay(conn, number, on):
    """number is 1, 2 or 3. MAVLink numbers relays from 0."""
    return send_command(conn, mavutil.mavlink.MAV_CMD_DO_SET_RELAY, number - 1, 1 if on else 0)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port")
    args = parser.parse_args()

    port = args.port or find_port()
    print(f"Connecting on {port}. Nothing from the mower should be on the relay terminals.")
    conn = setup(connect(port), port)

    print("\nAll relays should be OFF now: no click since boot finished, LEDs out.")
    print("Commands: 1on 1off 2on 2off 3on 3off, q to quit (all switched off on exit).")
    try:
        while True:
            text = input("> ").strip().lower()
            if text in ("q", "quit", "exit"):
                break
            if text not in ("1on", "1off", "2on", "2off", "3on", "3off"):
                print("Use 1on, 1off, 2on, 2off, 3on, 3off, or q.")
                continue
            ok = set_relay(conn, int(text[0]), text.endswith("on"))
            print("Sent." if ok else "The Pixhawk rejected that command.")
    except (EOFError, KeyboardInterrupt):
        print()
    finally:
        set_relay(conn, 1, False)
        set_relay(conn, 2, False)
        set_relay(conn, 3, False)
        print("All relays commanded off.")


if __name__ == "__main__":
    main()
