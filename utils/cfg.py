from dataclasses import dataclass, field
from omegaconf import OmegaConf, MISSING
from typing import Optional, List


@dataclass
class ArmCfg:
    collection_type: str = MISSING
    """one_master / two_master / one_master_slave / two_master_slave"""

    # leader (master arm) serial ports
    leader_port_left: Optional[str] = None
    leader_port_right: Optional[str] = None

    # follower (slave arm) CAN channels
    follower_can_left: Optional[str] = None
    follower_can_right: Optional[str] = None

    use_velocity: bool = False
    use_effort: bool = False  # reserved — effort logging not yet implemented

    vr_hand: str = "right"
    urdf_path: Optional[str] = None


@dataclass
class CameraCfg:
    camera_names: List[str] = field(default_factory=list)
    """Names used as keys in saved data, e.g. ['cam_high', 'cam_right_wrist']"""

    # topic or device per camera name, keyed by camera_name
    # e.g. cam_high: /camera_high/color/image_raw
    topics: Optional[dict] = field(default_factory=dict)

    width: int = 1280
    height: int = 720
    """Capture resolution applied to every camera."""


@dataclass
class CollectorCfg:
    arm_freq: int = 30
    """Arm state collection frequency in Hz."""

    camera_freq: int = 30
    """Camera capture frequency in Hz."""

    cpu_affinity: Optional[int] = None
    """Pin arm collector thread to this CPU core. None = no pinning."""

    realtime_priority: bool = False
    """Use SCHED_FIFO for arm collector thread. Requires root."""


@dataclass
class Hdf5Cfg:
    dataset_dir: str = "data/"
    task_name: str = MISSING
    start_episode: Optional[int] = None


@dataclass
class DataCollectionCfg:
    task_description: str = MISSING

    num_episodes: int = 1

    hdf5_cfg: Hdf5Cfg = field(default_factory=Hdf5Cfg)
    arm_cfg: ArmCfg = field(default_factory=ArmCfg)
    camera_cfg: CameraCfg = field(default_factory=CameraCfg)
    collector_cfg: CollectorCfg = field(default_factory=CollectorCfg)


def build_config() -> tuple[DataCollectionCfg, str]:
    base_cfg = OmegaConf.structured(DataCollectionCfg)
    cli_cfg = OmegaConf.from_cli()

    if "config_file" not in cli_cfg:
        raise ValueError("arg config_file=your_path must be provided.")

    config_file = str(cli_cfg["config_file"])
    yaml_cfg = OmegaConf.load(config_file)
    base_cfg = OmegaConf.merge(base_cfg, yaml_cfg)

    del cli_cfg["config_file"]
    return OmegaConf.merge(base_cfg, cli_cfg), config_file
