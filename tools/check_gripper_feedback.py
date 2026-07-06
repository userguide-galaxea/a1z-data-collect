"""
Quick check: enable the gripper motor and listen for feedback frames on CAN ID 7.
Usage: python tools/check_gripper_feedback.py --can can0
"""
import argparse
import time
import can

GRIPPER_CAN_ID = 7
LISTEN_SEC = 3.0


def enable_motor(bus, motor_id):
    data = bytes([0xFF] * 7 + [0xFC])
    msg = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
    bus.send(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--can", default="can0")
    args = parser.parse_args()

    print(f"[check] Opening {args.can} ...")
    bus = can.interface.Bus(channel=args.can, bustype="socketcan", bitrate=1_000_000)

    print(f"[check] Sending enable to motor ID {GRIPPER_CAN_ID} ...")
    enable_motor(bus, GRIPPER_CAN_ID)

    print(f"[check] Listening for {LISTEN_SEC}s — all frames on {args.can}:")
    t0 = time.time()
    total = 0
    gripper_frames = 0

    while time.time() - t0 < LISTEN_SEC:
        msg = bus.recv(timeout=0.1)
        if msg is None:
            continue
        total += 1
        marker = ""
        if int(msg.arbitration_id) == GRIPPER_CAN_ID:
            gripper_frames += 1
            marker = "  ← GRIPPER (ID=7)"
        print(f"  ID=0x{msg.arbitration_id:03X}  data={msg.data.hex()}{marker}")

    print(f"\n[check] Done. total={total} frames, gripper(ID=7)={gripper_frames} frames")
    if gripper_frames == 0:
        print("[check] FAIL: No gripper feedback received — motor may be off, wrong CAN bus, or wrong ID")
    else:
        print("[check] OK: Gripper feedback confirmed")
    bus.shutdown()


if __name__ == "__main__":
    main()
