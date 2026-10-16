# a1z-data-collect

English | [简体中文](README.md)

A teleoperation data collection tool for an A1Z-T Leader and an A1Z Follower. One command starts Leader readings, Follower control, and data recording. It supports single-arm and dual-arm setups. Joint data and USB camera frames are collected at separately configured target rates and saved as HDF5.

## Hardware versions and branches

| Repository branch | Supported A1Z-T hardware | Teleoperation software |
|-------------------|--------------------------|------------------------|
| `legacy/open-a1z-t` | Galaxea's open-source OpenA1Z-T hardware | [OpenA1Z-T](https://github.com/userguide-galaxea/OpenA1Z-T) |
| `main` | Galaxea's current A1Z-T hardware | `a1z-t` |

The installation, configuration, and usage instructions below apply to `main`. For the open-source hardware, clone `legacy/open-a1z-t` directly and follow the README on that branch. This avoids fetching the `a1z-t` submodule from `main` first:

```bash
git clone --branch legacy/open-a1z-t --recurse-submodules https://github.com/userguide-galaxea/a1z-data-collect.git
cd a1z-data-collect
```

---

## Installation

### 1. Clone the repository

```bash
git clone --branch main --recurse-submodules https://github.com/userguide-galaxea/a1z-data-collect.git
cd a1z-data-collect
```

If you cloned without `--recurse-submodules`, initialize the submodules afterward:

```bash
git submodule update --init --recursive
```

### 2. Install dependencies

Ubuntu 22.04 and Python 3.11 or newer are required. The `a1z-t` submodule comes from its [GitHub repository](https://github.com/userguide-galaxea/a1z-t) and is pinned to a commit on its `main` branch. The A1Z SDK is a separate dependency whose version is pinned by `a1z-t/pyproject.toml`. Prepare a Python environment as described in `a1z-t/README.md`, then install both projects in **the same environment**:

```bash
python -m pip install -e ./a1z-t
python -m pip install -e .
```

A recursive clone also checks out the `a1z-t/GALAXEA-A1Z` SDK source submodule. Installing `a1z-t` obtains the pinned version of the `a1z` package through its project dependencies.

### 3. Configure the CAN bus

`scripts/setup_ports.sh` and `scripts/setup_can.sh` are symbolic links to the scripts of the same names in `a1z-t/scripts/`. First list the adapter serial numbers and bind the left and right CAN interfaces to stable names. The binding script also requires the corresponding Leader serial-port serial numbers. For a dual-arm setup, provide both right-side serial numbers as well:

```bash
bash scripts/setup_ports.sh --list
LEADER_LEFT_SERIAL=... CAN_LEFT_SERIAL=... bash scripts/setup_ports.sh
# For dual arms, use this command instead of the single-arm command above
LEADER_LEFT_SERIAL=... CAN_LEFT_SERIAL=... LEADER_RIGHT_SERIAL=... CAN_RIGHT_SERIAL=... bash scripts/setup_ports.sh
```

After installing the rules, reconnect the adapters or reboot. Confirm that `can_left` is present (and `can_right` for dual arms), then configure the CAN bus. For a single arm, run only the first command:

```bash
bash scripts/setup_can.sh can_left
bash scripts/setup_can.sh can_right
```

The example configuration files use these names for `follower_can_left` and `follower_can_right`. The binding script also creates `/dev/leader_left` and `/dev/leader_right`; if you want to use those stable serial-port names, update `leader_port_left` and `leader_port_right` in your configuration accordingly.

---

## Project layout

```
a1z-data-collect/
├── cfg/
│   └── two_master_slave.yaml       # Example configuration
├── collectors/
│   ├── arm_collector.py            # Teleoperation and joint-recording thread
│   └── camera_collector.py         # Camera-recording thread
├── dataset/
│   └── hdf5_dataset.py             # HDF5 storage
├── robots/
│   ├── leader_a1z_t.py             # Leader driver (LeaderArmInterface)
│   ├── follower_a1z.py             # Follower driver (FollowerArmInterface)
│   ├── teleop_delta_controller.py  # Delta teleoperation controller (TeleopControllerInterface)
│   ├── camera_opencv.py            # USB camera driver (CameraInterface)
│   └── camera_calibration.py       # Interactive camera calibration
├── utils/
│   ├── cfg.py                      # Configuration definitions
│   ├── interfaces.py               # Hardware abstraction interfaces
│   └── key_handle.py               # Keyboard controls
├── scripts/
│   ├── record_data.py              # Main entry point
│   ├── setup_ports.sh              # Symlink to the a1z-t device-naming script
│   └── setup_can.sh                # Symlink to the a1z-t CAN setup script
└── a1z-t/                          # Git submodule (Leader software with a separate A1Z SDK dependency)
```

---

## Configuration

Copy an example configuration file and edit it for your task:

```bash
cp cfg/two_master_slave.yaml cfg/my_task.yaml
```

The configuration fields are shown below:

```yaml
task_description: "pick and place"   # Task description stored in each episode's metadata
num_episodes: 100                     # Planned number of episodes

hdf5_cfg:
  dataset_dir: "data/"               # Root directory for recorded data
  task_name: "two_master_slave"      # Subdirectory: dataset_dir/task_name/
  start_episode: null                # Starting index; null finds the first unused index from 0

arm_cfg:
  collection_type: "two_master_slave"  # Collection mode; see the table below
  leader_port_left: "/dev/ttyACM0"    # Left Leader serial port
  leader_port_right: "/dev/ttyACM1"   # Right Leader serial port (not needed for one arm)
  leader_config_path: null             # null uses the a1z-t default EVT config; PVT can use a local copy
  follower_can_left: "can_left"       # Left Follower CAN interface
  follower_can_right: "can_right"     # Right Follower CAN interface (not needed for one arm)
  use_velocity: false                  # Record velocity; Leader-Follower modes also send velocity targets
  use_effort: false                    # Reserved; effort is not currently recorded

camera_cfg:
  camera_names: ["cam_high", "cam_right_wrist", "cam_left_wrist"]
  topics:                             # Map camera names to USB video device numbers or paths, not ROS topics
    cam_high: 0                       # An empty or missing mapping triggers interactive calibration
    cam_right_wrist: 1
    cam_left_wrist: 2

collector_cfg:
  arm_freq: 30           # Target joint collection rate (Hz)
  camera_freq: 30        # Target rate for each camera (Hz)
  cpu_affinity: null     # CPU core for the joint collection thread; null disables pinning
  realtime_priority: false  # Request SCHED_FIFO scheduling; requires system permission
```

**Collection modes (`collection_type`):**

| Mode | A1Z Follower | Follow commands while recording | `state` | `action` |
|------|--------------|---------------------------------|---------|----------|
| `one_master` | Start and read one arm | None | Follower feedback (7,) | Leader readings (7,) |
| `two_master` | Start and read both arms | None | Feedback from both Followers (14,) | Readings from both Leaders (14,) |
| `one_master_slave` | Start and read one arm | Sent | Follower feedback (7,) | Target sent to the Follower (7,) |
| `two_master_slave` | Start and read both arms | Sent | Feedback from both Followers (14,) | Targets sent to both Followers (14,) |

Every mode requires the corresponding A1Z Follower and CAN interface. `one_master` and `two_master` skip follow commands only while recording; they still send a return-to-zero trajectory on exit. `action_raw` is the Leader interface output after joint-direction conversion and gripper normalization, but before the Delta controller. In modes without follow commands, it equals `action`.

A 7-element position vector is `[j1..j6 rad, gripper_norm]`, where `gripper_norm` ranges from 0.0 (closed) to 1.0 (open). A dual-arm, 14-element vector is ordered `[left(7), right(7)]`.

Dual-arm Leader operation requires two separate serial ports. In the current `a1z-t`, both left and right Leaders use servo IDs 1–7, so they cannot share one port.

For a PVT Leader, copy the complete `a1z-t/config.yaml` into this repository (for example, as `cfg/a1z_t.yaml`), set `a1z_t_version` to `PVT`, and point `arm_cfg.leader_config_path` to that file. Both Leaders share this configuration. The Leader trigger's opening and closing direction is converted to the Follower gripper's normalized direction.

---

## Camera calibration

You can specify USB video devices directly. When multiple cameras of the same model cannot be distinguished by device name, you can calibrate the mapping interactively using their images.

**Option 1: Specify device numbers in YAML (recommended when wiring is fixed)**

```yaml
camera_cfg:
  topics:
    cam_high: 0
    cam_right_wrist: 1
    cam_left_wrist: 2
```

**Option 2: Interactive calibration (when device order varies)**

Leave `topics` empty:

```yaml
camera_cfg:
  topics: {}
```

When the program starts, it opens a calibration window:

1. Live previews of all detected cameras are displayed.
2. Click the preview for each camera as prompted.
3. Press Enter or Space to confirm. The mapping is written back to the current collection configuration file.

---

## Record data

### Start

Run this from the `a1z-data-collect` repository root:

```bash
python -m scripts.record_data config_file=cfg/my_task.yaml
```

The program opens the Leaders, starts the Followers, opens the cameras (calibrating them if needed), and then waits for the start command. The collection process itself controls the Followers; do not run an `a1z-t` teleoperation entry point against the same CAN devices at the same time.

You can override individual YAML parameters on the command line:

```bash
python -m scripts.record_data config_file=cfg/my_task.yaml task_description="cup stacking" num_episodes=50
```

`collector_cfg.realtime_priority=true` requests the system's `SCHED_FIFO` scheduling permission. If permission is unavailable, the program reports this and continues. Configured rates are targets; check data timestamps to determine actual sampling rates.

### Keyboard controls

| Key | Action |
|-----|--------|
| `S` | Start recording the current episode |
| `E` | End and save the current episode |
| `R` | Discard the current episode and start over |
| `Q` | Quit; discard the current episode if recording |

### Collection workflow

```
Start
  │
  ├── Initialize hardware (Leaders, Followers, cameras)
  ├── [If needed] Calibrate cameras interactively
  │
  └── Repeat for num_episodes
        │
        ├── Wait for S
        ├── S ──► Joint and camera threads start at their respective target rates
        │        Followers track the Leaders in real time
        │
        ├── E ──► Stop threads and save episode_N.hdf5
        ├── R ──► Discard the episode and wait for S again
        └── Q ──► Discard the current episode and quit
```

When recording finishes, `Q` is pressed, or Ctrl+C is pressed during recording, the program sends an approximately three-second return-to-zero trajectory to the Followers before closing the devices. Sending this trajectory does not mean feedback has confirmed that the Followers reached zero. A joint read or control error discards the current episode and stops collection.

---

## Data format

Data is stored at `dataset_dir/task_name/episode_N.hdf5`:

```
episode_N.hdf5
├── attrs["task"]                       # Task description string
├── arm/
│   ├── state               float32 [T, D]   # Follower feedback position; D=7 (one arm) or 14 (two arms)
│   ├── action              float32 [T, D]   # Control target in follow modes; Leader readings in record-only modes
│   ├── action_raw          float32 [T, D]   # Leader interface output before Delta mapping
│   ├── leader_timestamps   float64 [T]      # Completion time of Leader readings
│   ├── follower_timestamps float64 [T]      # Completion time of Follower readings
│   ├── velocity            float32 [T, V]   # Optional; V=6/12 in follow modes, 7/14 in record-only modes
│   └── velocity_raw        float32 [T, D]   # Optional raw Leader velocity; D=7/14
└── cameras/
    └── <cam_name>/
        ├── frames      vlen uint8            # JPEG bytes, stored separately for each frame
        └── timestamps  float64 [T]          # Frame-read completion time
```

`velocity` and `velocity_raw` are written only when `use_velocity=true`. In follow modes, `velocity` is the controller's velocity output for the six arm joints (12 for dual arms), excluding the gripper. The seventh element of `velocity_raw` is the Leader trigger servo's angular velocity. Effort is not currently written.

Joint and camera data are sampled separately. Their timestamps use the same monotonic clock with a fixed wall-clock offset, so align samples by timestamp rather than row number. Target rates do not guarantee that every frame is written; if the writer queue fills, the program reports dropped frames. Check dataset row counts and timestamp intervals before using the data.

---

## Add another input device

Create a file under `robots/` that implements the interfaces in `utils/interfaces.py`, then add a branch to `make_arm_readers()` in `scripts/record_data.py`. The collection, storage, and camera logic does not need to change.

```python
# Four interfaces defined in utils/interfaces.py

class LeaderArmInterface(ABC):
    def open(self): ...
    def read_as_vector(self) -> tuple[np.ndarray, np.ndarray]:
        # action (7,), velocity (7,)

class FollowerArmInterface(ABC):
    def start(self): ...
    def command(self, action: np.ndarray, velocity: np.ndarray | None = None) -> None: ...
    def get_joint_pos(self) -> np.ndarray: ...  # (7,)
    def stop(self): ...

class TeleopControllerInterface(ABC):
    def compute_command(
        self,
        leader_action: np.ndarray,
        follower_pos: np.ndarray,
        leader_velocity: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray | None]: ...

class CameraInterface(ABC):
    def open(self): ...
    def read(self) -> np.ndarray | None: ...  # HWC uint8 RGB
    def close(self): ...
```

**Example: Integrating VR input**

```
robots/leader_vr.py              # Implement LeaderArmInterface and read the VR controller
robots/teleop_vr_controller.py   # Implement TeleopControllerInterface and Cartesian mapping
```

Add this branch to `make_arm_readers()`:

```python
elif t == "vr_slave":
    leader = VRLeaderArm(...)
    follower = A1ZFollowerArm(...)
    controller = VRTeleopController(...)
```
