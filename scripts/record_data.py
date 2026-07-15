"""Main data collection entry point.

One command starts everything: teleoperation + recording.

Usage:
    python -m scripts.record_data config_file=cfg/two_master_slave.yaml
"""

import time

from omegaconf import OmegaConf
from utils.cfg import DataCollectionCfg, build_config
from utils.key_handle import init_keyboard_listener
from utils.interfaces import LeaderArmInterface, FollowerArmInterface, TeleopControllerInterface
from collectors.arm_collector import ArmCollector
from collectors.camera_collector import CameraCollector
from dataset.hdf5_dataset import H5Dataset
from utils.homing import return_to_home
from robots.leader_a1z_t import A1ZTLeaderArm
from robots.follower_a1z import A1ZFollowerArm
from robots.teleop_delta_controller import DeltaTeleopController, DualDeltaTeleopController
from robots.camera_opencv import OpenCVCamera
from robots.camera_calibration import calibrate_cameras
from robots.dual_arm import DualArmLeader, DualArmFollower
from leader import DynamixelBus

from pathlib import Path

try:
    from a1z.robots.kinematics import Kinematics
    from robots.vr_bridge import VRBridge
    from robots.vr_utils import VRDataStore, init_vr_event_listener
    from robots.vr_control import VRControl
    _VR_AVAILABLE = True
except ImportError:
    _VR_AVAILABLE = False


def _default_urdf_path():
    import a1z.robots.get_robot as _gr
    return str(Path(_gr.__file__).parent.parent / "robot_models" / "a1z" / "A1Z_G1Z.urdf")


def _make_dual_leader(arm_cfg) -> DualArmLeader:
    """Build a DualArmLeader, using a shared bus when both ports are the same (daisy-chain)."""
    port_l = arm_cfg.leader_port_left
    port_r = arm_cfg.leader_port_right

    if port_l == port_r:
        shared_bus = DynamixelBus(port_l, 1_000_000, list(range(1, 15)))
        left  = A1ZTLeaderArm(port=port_l, side="left",  shared_bus=shared_bus)
        right = A1ZTLeaderArm(port=port_r, side="right", shared_bus=shared_bus)
        print(f"[DualArmLeader] daisy-chain mode: both arms on {port_l}")
        return DualArmLeader(left, right)
    else:
        left  = A1ZTLeaderArm(port=port_l, side="left")
        right = A1ZTLeaderArm(port=port_r, side="right")
        print(f"[DualArmLeader] independent ports: left={port_l}  right={port_r}")
        return DualArmLeader(left, right)


def make_arm_readers(
    cfg: DataCollectionCfg,
) -> tuple[LeaderArmInterface, FollowerArmInterface, TeleopControllerInterface | None]:
    arm_cfg = cfg.arm_cfg
    t = arm_cfg.collection_type

    if t == "one_master":
        # Leader IS the follower — no separate arm to control
        leader = A1ZTLeaderArm(port=arm_cfg.leader_port_left, side="left")
        follower = A1ZFollowerArm(can_channel=arm_cfg.follower_can_left)
        controller = None

    elif t == "two_master":
        leader = _make_dual_leader(arm_cfg)
        follower = DualArmFollower(
            A1ZFollowerArm(can_channel=arm_cfg.follower_can_left),
            A1ZFollowerArm(can_channel=arm_cfg.follower_can_right),
        )
        controller = None

    elif t == "one_master_slave":
        leader = A1ZTLeaderArm(port=arm_cfg.leader_port_left, side="left")
        follower = A1ZFollowerArm(can_channel=arm_cfg.follower_can_left)
        controller = DeltaTeleopController()

    elif t == "two_master_slave":
        leader = _make_dual_leader(arm_cfg)
        follower = DualArmFollower(
            A1ZFollowerArm(can_channel=arm_cfg.follower_can_left),
            A1ZFollowerArm(can_channel=arm_cfg.follower_can_right),
        )
        controller = DualDeltaTeleopController()

    elif t == "vr_teleop":
        if not _VR_AVAILABLE:
            raise ImportError("VR teleop requires a1z Kinematics and websockets. "
                              "Install with: pip install pin websockets")
        vr_store = VRDataStore()
        bridge = VRBridge(vr_store)
        bridge.start()

        urdf = arm_cfg.urdf_path or _default_urdf_path()
        ik_l = Kinematics(urdf)
        ik_r = Kinematics(urdf)

        leader = VRControl(vr_store=vr_store, ik_left=ik_l, ik_right=ik_r)
        leader._bridge = bridge
        follower = DualArmFollower(
            A1ZFollowerArm(can_channel=arm_cfg.follower_can_left),
            A1ZFollowerArm(can_channel=arm_cfg.follower_can_right),
        )
        controller = None

    else:
        raise ValueError(f"Unknown collection_type: {t}")

    leader.open()
    try:
        follower.start()
    except Exception:
        leader.close()
        raise

    if t == "vr_teleop":
        pos = follower.get_joint_pos()
        leader.sync_state(pos[:6], pos[7:13])

    return leader, follower, controller


def make_camera_readers(cfg: DataCollectionCfg, config_file: str | None = None) -> dict:
    cam_names = cfg.camera_cfg.camera_names
    device_map = cfg.camera_cfg.topics

    if not device_map or any(n not in device_map for n in cam_names):
        print("Running interactive camera calibration...")
        device_map = calibrate_cameras(cam_names)
        if config_file:
            raw = OmegaConf.load(config_file)
            OmegaConf.update(raw, "camera_cfg.topics", device_map, merge=False)
            OmegaConf.save(raw, config_file)
            print(f"Camera mapping saved to {config_file}")

    readers = {}
    for cam_name in cam_names:
        cam = OpenCVCamera(
            device_map[cam_name],
            width=cfg.camera_cfg.width,
            height=cfg.camera_cfg.height,
            fps=cfg.collector_cfg.camera_freq,
        )
        cam.open()
        readers[cam_name] = cam
    return readers


def run(cfg: DataCollectionCfg, config_file: str | None = None) -> None:
    collector_cfg = cfg.collector_cfg
    arm_cfg = cfg.arm_cfg

    import os, termios, sys
    os.system("stty -echo")

    leader, follower, controller = make_arm_readers(cfg)
    listener = None
    camera_readers = {}
    arm_collector = None
    camera_collector = None
    dataset = None
    try:
        camera_readers = make_camera_readers(cfg, config_file)
        dataset = H5Dataset(cfg)
        arm_collector = ArmCollector(
            leader=leader,
            follower=follower,
            controller=controller,
            freq=collector_cfg.arm_freq,
            cpu_affinity=collector_cfg.cpu_affinity,
            realtime_priority=collector_cfg.realtime_priority,
            use_velocity=arm_cfg.use_velocity,
            dataset=dataset,
        )
        camera_collector = CameraCollector(camera_readers, freq=collector_cfg.camera_freq, dataset=dataset)

        if arm_cfg.collection_type == "vr_teleop":
            listener, events = init_vr_event_listener(leader.vr_store)
        else:
            listener, events = init_keyboard_listener()
        cam_names = list(camera_readers.keys())

        count = 0

        try:
            while count < cfg.num_episodes:
                print(f"Press [S] to start episode {count + 1}/{cfg.num_episodes}")
                while not events["start_recording"]:
                    if events["stop"]:
                        break
                    time.sleep(0.02)
                if events["stop"]:
                    break
                events["start_recording"] = False

                dataset.open_episode(cam_names, use_velocity=arm_cfg.use_velocity)

                print("Recording... [E] finish  [R] rerecord  [Q] quit")
                arm_collector.start()
                camera_collector.start()

                while not events["finish_recording"]:
                    if events["stop"]:
                        break
                    time.sleep(0.02)

                arm_collector.stop()
                camera_collector.stop()

                if events["stop"]:
                    dataset.close_episode(discard=True)
                    break
                events["finish_recording"] = False

                if events["rerecord"]:
                    print(f"Rerecording episode {count + 1}")
                    events["rerecord"] = False
                    dataset.close_episode(discard=True)
                    continue

                dataset.close_episode(discard=False)
                count += 1
        except KeyboardInterrupt:
            print("\nInterrupted.")
            if arm_collector is not None:
                arm_collector.stop()
            if camera_collector is not None:
                camera_collector.stop()
            if dataset is not None:
                dataset.close_episode(discard=True)  # join writer procs, drop .tmp

        print("Returning to home...")
        return_to_home(follower, 3.0)  # home after normal finish / Q / interrupt
        print("Home reached.")
    except KeyboardInterrupt:
        print("\nAborted homing — emergency stop.")
    finally:
        if listener is not None:
            listener.stop()
            listener.join(timeout=1.0)  # pynput blocks on read; stop() won't wake it, so don't wait forever
        follower.stop()
        leader.close()
        for cam in camera_readers.values():
            cam.close()
        termios.tcflush(sys.stdin, termios.TCIOFLUSH)
        os.system("stty sane")


def main():
    cfg, config_file = build_config()
    print(OmegaConf.to_yaml(cfg))
    run(cfg, config_file)


if __name__ == "__main__":
    main()
