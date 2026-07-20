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

try:
    from a1z.robots.kinematics import Kinematics
    from robots.vr_bridge import VRBridge
    from robots.vr_utils import VRDataStore, init_vr_event_listener, init_vr_hand_supervisor
    from robots.vr_control import VRControl
    _VR_AVAILABLE = True
except ImportError:
    _VR_AVAILABLE = False


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

        if arm_cfg.urdf_path:
            urdf = arm_cfg.urdf_path
        else:
            from pathlib import Path
            import a1z.robots.get_robot as _gr
            urdf = str(Path(_gr.__file__).parent.parent / "robot_models" / "a1z" / "A1Z_G1Z.urdf")
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

    print("[启动] 主臂(leader)连接中...")
    leader.open()
    print("[启动] 主臂(leader)连接完成。")
    try:
        print("[启动] 从臂(follower)启动中（电机上电 + 夹爪归零）...")
        follower.start()
        print("[启动] 从臂(follower)启动完成。")
    except Exception:
        leader.close()
        raise

    if t == "vr_teleop":
        print("[启动] 同步从臂关节状态到 VR 控制器...")
        pos = follower.get_joint_pos()
        leader.sync_state(pos[:6], pos[7:13])
        print("[启动] 同步完成。")

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
    for i, cam_name in enumerate(cam_names):
        print(f"[启动] 相机 {cam_name} 初始化中 ({i+1}/{len(cam_names)})...")
        cam = OpenCVCamera(
            device_map[cam_name],
            width=cfg.camera_cfg.width,
            height=cfg.camera_cfg.height,
            fps=cfg.collector_cfg.camera_freq,
        )
        cam.open()
        readers[cam_name] = cam
    print(f"[启动] 全部 {len(cam_names)} 个相机初始化完成。")
    return readers


def run(cfg: DataCollectionCfg, config_file: str | None = None) -> None:
    collector_cfg = cfg.collector_cfg
    arm_cfg = cfg.arm_cfg

    import os, termios, sys
    os.system("stty -echo")

    leader, follower, controller = make_arm_readers(cfg)
    kb_listener = None
    vr_listener = None
    vr_hand_sup = None
    camera_readers = {}
    arm_collector = None
    camera_collector = None
    dataset = None
    is_vr = arm_cfg.collection_type == "vr_teleop"
    try:
        print("[启动] 相机初始化中...")
        camera_readers = make_camera_readers(cfg, config_file)
        print("[启动] 数据集初始化中...")
        dataset = H5Dataset(cfg)
        print("[启动] 数据集初始化完成。")
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

        # 键盘监听器始终启用（S/E/R/Q）。
        # VR 模式下:
        #   - 左手柄监听器与键盘共享同一组 events, 二者平行可相互替代;
        #   - 右手柄监听器(VRHandSupervisor)不进 events, 而是直接调用 leader
        #     上的 toggle_enabled / trigger_return_to_zero / emergency_stop,
        #     因此可在采集进行中即时控制机械臂起停, 不会被主循环的等待阻塞。
        kb_listener, events = init_keyboard_listener()
        if is_vr:
            vr_listener, events = init_vr_event_listener(leader.vr_store, events)
            vr_hand_sup = init_vr_hand_supervisor(leader.vr_store, leader)
        cam_names = list(camera_readers.keys())

        count = 0

        try:
            while count < cfg.num_episodes:
                if is_vr:
                    print(f"\n=== 等待开始 episode {count + 1}/{cfg.num_episodes} ===")
                    print("  → 按键盘 [S]  或  左手柄 X键(下)  开始本条采集")
                    print("  → 右手柄 A键(下): 使能/失能遥操作  B键(上): 回零  摇杆按下: 急停")
                else:
                    print(f"Press [S] to start episode {count + 1}/{cfg.num_episodes}")
                while not events["start_recording"]:
                    if events["stop"]:
                        break
                    time.sleep(0.02)
                if events["stop"]:
                    break
                events["start_recording"] = False

                dataset.open_episode(cam_names, use_velocity=arm_cfg.use_velocity)

                if is_vr:
                    enabled = getattr(leader, "teleop_enabled", False)
                    print("\n[录制中] episode {}/{}".format(count + 1, cfg.num_episodes))
                    print("  键盘: [E]结束  [R]重录  [Q]退出")
                    print("  左手柄: Y键(上)=结束  摇杆按下+左推=重录  摇杆按下+右推=退出")
                    print("  右手柄: A键(下)=使能/失能  B键(上)=回零  摇杆按下=急停")
                    print(f"  当前遥操作状态: {'使能(握住手柄即可操作)' if enabled else '失能(双臂冻结, 按右手A键使能)'}")
                    print("  → 握住手柄(side grip)即可控制机械臂; 松开则冻结目标。")
                else:
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
        if vr_hand_sup is not None:
            vr_hand_sup.stop()
            vr_hand_sup.join(timeout=1.0)
        if vr_listener is not None:
            vr_listener.stop()
            vr_listener.join(timeout=1.0)
        if kb_listener is not None:
            kb_listener.stop()
            kb_listener.join(timeout=1.0)  # pynput blocks on read; stop() won't wake it, so don't wait forever
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
