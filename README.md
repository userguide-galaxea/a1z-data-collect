# a1z-data-collect

[English](README_EN.md) | 简体中文

A1Z-T Leader 与 A1Z Follower 的遥操作数据采集工具。一条命令启动主臂读取、从臂控制和数据记录，支持单臂与双臂。关节和 USB 相机分别按配置的目标频率采集，保存为 HDF5。

## 硬件版本与分支

| 本仓库分支 | 适配的 A1Z-T 硬件 | 遥操软件 |
|----------|------------------|----------|
| `legacy/open-a1z-t` | 星海图开源版硬件 OpenA1Z-T  | [OpenA1Z-T](https://github.com/userguide-galaxea/OpenA1Z-T) |
| `main` | 星海图最新 A1Z-T 硬件 | `a1z-t` |

下文的安装、配置和运行说明适用于 `main`。使用开源版硬件时，请直接克隆 `legacy/open-a1z-t` 分支并按照该分支的 README 安装，避免先克隆 `main` 时拉取其 `a1z-t` 子模块：

```bash
git clone --branch legacy/open-a1z-t --recurse-submodules https://github.com/userguide-galaxea/a1z-data-collect.git
cd a1z-data-collect
```

---

## 安装

### 1. 克隆项目

```bash
git clone --branch main --recurse-submodules https://github.com/userguide-galaxea/a1z-data-collect.git
cd a1z-data-collect
```

如果已经 clone 但忘了带 `--recurse-submodules`：

```bash
git submodule update --init --recursive
```

### 2. 安装依赖

需要 Ubuntu 22.04 和 Python 3.11 或更高版本。`a1z-t` 子模块固定到 GitLab `main` 的一个提交；克隆时需要该仓库的读取权限。A1Z SDK 是独立依赖，其版本由 `a1z-t/pyproject.toml` 固定。先按 `a1z-t/README.md` 准备 Python 环境，再在**同一环境**中安装：

```bash
python -m pip install -e ./a1z-t
python -m pip install -e .
```

递归克隆还会检出 `a1z-t/GALAXEA-A1Z` SDK 源码子模块；安装 `a1z-t` 时会按其项目依赖取得固定版本的 `a1z` 包。

### 3. 配置 CAN 总线

`scripts/setup_ports.sh` 和 `scripts/setup_can.sh` 是指向 `a1z-t/scripts/` 同名脚本的软链接。先查看适配器序列号，并将左右 CAN 接口绑定为固定名称。绑定脚本也需要对应主臂串口的序列号；双臂模式再提供右侧的两个序列号：

```bash
bash scripts/setup_ports.sh --list
LEADER_LEFT_SERIAL=... CAN_LEFT_SERIAL=... bash scripts/setup_ports.sh
# 双臂使用下面这一条替代单臂绑定命令
LEADER_LEFT_SERIAL=... CAN_LEFT_SERIAL=... LEADER_RIGHT_SERIAL=... CAN_RIGHT_SERIAL=... bash scripts/setup_ports.sh
```

安装规则后重新插拔适配器或重启，确认系统中出现 `can_left`（双臂还需 `can_right`），再配置 CAN 总线。单臂只运行第一条：

```bash
bash scripts/setup_can.sh can_left
bash scripts/setup_can.sh can_right
```

示例配置中的 `follower_can_left` 和 `follower_can_right` 分别使用上述接口名。绑定脚本还会创建 `/dev/leader_left` 和 `/dev/leader_right`；如需使用固定的主臂串口名称，也要相应修改配置中的 `leader_port_left` 和 `leader_port_right`。

---

## 项目结构

```
a1z-data-collect/
├── cfg/
│   └── two_master_slave.yaml       # 示例配置
├── collectors/
│   ├── arm_collector.py            # 遥操作 + 关节记录线程
│   └── camera_collector.py         # 相机记录线程
├── dataset/
│   └── hdf5_dataset.py             # HDF5 存储
├── robots/
│   ├── leader_a1z_t.py             # 主臂驱动（LeaderArmInterface 实现）
│   ├── follower_a1z.py             # 从臂驱动（FollowerArmInterface 实现）
│   ├── teleop_delta_controller.py  # Delta 遥操作控制器（TeleopControllerInterface 实现）
│   ├── camera_opencv.py            # USB 相机驱动（CameraInterface 实现）
│   └── camera_calibration.py       # 交互式相机标定
├── utils/
│   ├── cfg.py                      # 配置定义
│   ├── interfaces.py               # 硬件抽象接口
│   └── key_handle.py               # 键盘控制
├── scripts/
│   ├── record_data.py              # 主入口
│   ├── setup_ports.sh              # 软链接到 a1z-t 的设备名称绑定脚本
│   └── setup_can.sh                # 软链接到 a1z-t 的 CAN 配置脚本
└── a1z-t/                          # git submodule（Leader 软件，引用独立 A1Z SDK）
```

---

## 配置

复制示例配置文件并按需修改：

```bash
cp cfg/two_master_slave.yaml cfg/my_task.yaml
```

配置文件各字段说明：

```yaml
task_description: "pick and place"   # 任务描述，写入每个 episode 的元数据
num_episodes: 100                     # 计划采集的 episode 总数

hdf5_cfg:
  dataset_dir: "data/"               # 数据保存根目录
  task_name: "two_master_slave"      # 子目录名，实际路径为 dataset_dir/task_name/
  start_episode: null                # 起始编号；null 从 0 开始找首个未占用编号

arm_cfg:
  collection_type: "two_master_slave"  # 采集模式，见下表
  leader_port_left: "/dev/ttyACM0"    # 左主臂串口
  leader_port_right: "/dev/ttyACM1"   # 右主臂串口（单臂模式不需要）
  leader_config_path: null             # null 使用 a1z-t 默认 EVT 配置；PVT 可指定本地副本
  follower_can_left: "can_left"       # 左从臂 CAN 通道
  follower_can_right: "can_right"     # 右从臂 CAN 通道（单臂模式不需要）
  use_velocity: false                  # 记录速度；主从模式还会向从臂发送速度目标
  use_effort: false                    # 预留字段，当前不记录力矩

camera_cfg:
  camera_names: ["cam_high", "cam_right_wrist", "cam_left_wrist"]
  topics:                             # 相机名到 USB 视频设备号或路径的映射，不是 ROS topic
    cam_high: 0                       # 映射为空或缺少相机名时会交互式标定
    cam_right_wrist: 1
    cam_left_wrist: 2

collector_cfg:
  arm_freq: 30           # 关节采集目标频率（Hz）
  camera_freq: 30        # 每路相机采集目标频率（Hz）
  cpu_affinity: null     # 关节采集线程绑定的 CPU 核号，null 表示不绑定
  realtime_priority: false  # 请求 SCHED_FIFO 调度，需要相应系统权限
```

**采集模式（collection_type）：**

| 模式 | A1Z 从臂 | 录制期间的跟随命令 | `state` | `action` |
|------|----------|----------|---------|----------|
| `one_master` | 启动并读取单臂 | 不发送 | 从臂反馈 (7,) | 主臂读数 (7,) |
| `two_master` | 启动并读取双臂 | 不发送 | 双臂从臂反馈 (14,) | 双臂主臂读数 (14,) |
| `one_master_slave` | 启动并读取单臂 | 发送 | 从臂反馈 (7,) | 发给从臂的目标 (7,) |
| `two_master_slave` | 启动并读取双臂 | 发送 | 双臂从臂反馈 (14,) | 发给双臂从臂的目标 (14,) |

所有模式都需要对应的 A1Z 从臂与 CAN 接口；`one_master` 和 `two_master` 只在录制期间跳过跟随命令，退出时仍会发送回零轨迹。`action_raw` 是主臂接口输出，已做关节方向与夹爪归一化处理，但尚未经过 Delta 控制器；在不发送跟随命令的模式中与 `action` 相同。

7 维位置向量为 `[j1..j6 rad, gripper_norm]`，其中 `gripper_norm` 为 0.0（闭合）到 1.0（张开）。双臂 14 维按 `[left(7), right(7)]` 排列。

双臂 Leader 需要配置两个独立串口；新版 `a1z-t` 的左右臂均使用 1–7 号舵机，不能把两侧指向同一个串口。

使用 PVT Leader 时，先复制完整的 `a1z-t/config.yaml` 到采集仓库内（例如 `cfg/a1z_t.yaml`），将其中的 `a1z_t_version` 改为 `PVT`，再将 `arm_cfg.leader_config_path` 指向该文件。双臂共用此配置。主臂扳机的开合方向会转换为从臂夹爪的归一化方向。

---

## 相机标定

可直接指定 USB 视频设备；多个同型号相机难以按设备名区分时，也可按画面交互式标定。

**方式一：在 yaml 里填设备号（推荐固定接线时使用）**

```yaml
camera_cfg:
  topics:
    cam_high: 0
    cam_right_wrist: 1
    cam_left_wrist: 2
```

**方式二：交互式标定（接线顺序不固定时使用）**

将 `topics` 留空：

```yaml
camera_cfg:
  topics: {}
```

启动后自动弹出标定窗口：

1. 屏幕显示所有检测到的相机实时画面
2. 按提示点击对应的相机预览
3. 按回车或空格确认；映射会写回当前采集配置文件

---

## 录制数据

### 启动

在 `a1z-data-collect` 仓库根目录运行：

```bash
python -m scripts.record_data config_file=cfg/my_task.yaml
```

启动后程序依次打开主臂、启动从臂、打开相机（必要时标定），然后等待开始指令。采集进程本身负责从臂控制，同一组 CAN 设备不要再运行 `a1z-t` 的遥操入口。

可以在命令行覆盖 yaml 中的单个参数：

```bash
python -m scripts.record_data config_file=cfg/my_task.yaml task_description="cup stacking" num_episodes=50
```

`collector_cfg.realtime_priority=true` 会请求系统的 `SCHED_FIFO` 调度权限；若未获授权，程序会输出提示并继续运行。配置的频率是目标值，实际采样率需从数据时间戳核对。

### 键盘操作

| 按键 | 说明 |
|------|------|
| `S` | 开始采集当前 episode |
| `E` | 结束并保存当前 episode |
| `R` | 丢弃当前 episode，重新采集 |
| `Q` | 退出；若正在录制，则丢弃当前 episode |

### 采集流程

```
启动
  │
  ├── 硬件初始化（主臂、从臂、相机）
  ├── [如需] 相机交互式标定
  │
  └── 循环 num_episodes 次
        │
        ├── 等待按 S
        ├── 按 S ──► 关节线程 + 相机线程按各自目标频率启动
        │             从臂实时跟随主臂运动
        │
        ├── 按 E ──► 线程停止，保存 episode_N.hdf5
        ├── 按 R ──► 丢弃本次，重新等待 S
        └── 按 Q ──► 丢弃当前 episode 并退出
```

录制完成、按 Q 退出或录制中按 Ctrl+C 后，程序会发送约 3 秒的从臂回零轨迹，再关闭设备；这表示已发送轨迹，不代表反馈已确认到位。关节读取或控制报错时会丢弃当前 episode 并停止采集。

---

## 数据格式

数据保存在 `dataset_dir/task_name/episode_N.hdf5`：

```
episode_N.hdf5
├── attrs["task"]                       # 任务描述字符串
├── arm/
│   ├── state               float32 [T, D]   # 从臂反馈位置，D=7(单臂)/14(双臂)
│   ├── action              float32 [T, D]   # 主从模式的控制目标；纯记录模式为主臂读数
│   ├── action_raw          float32 [T, D]   # 主臂接口输出，未经 Delta 映射
│   ├── leader_timestamps   float64 [T]      # 主臂读取完成时刻
│   ├── follower_timestamps float64 [T]      # 从臂读取完成时刻
│   ├── velocity            float32 [T, V]   # 可选；主从模式 V=6/12，纯记录模式 V=7/14
│   └── velocity_raw        float32 [T, D]   # 可选；主臂原始速度，D=7/14
└── cameras/
    └── <cam_name>/
        ├── frames      vlen uint8            # JPEG 字节，每帧独立存储
        └── timestamps  float64 [T]          # 帧读取完成时刻
```

`velocity` 和 `velocity_raw` 仅在 `use_velocity=true` 时写入。主从模式的 `velocity` 是六个手臂关节（双臂 12 个）的控制器速度输出，不含夹爪；`velocity_raw` 第七维是主臂扳机舵机的角速度。当前不写入力矩。

关节与相机分别采样，时间戳来自同一单调时钟加固定墙钟偏移，可按时间戳对齐，不能按行号直接配对。目标频率不保证每帧都写入；写入队列满时程序会打印丢帧提示，使用数据前应检查各数据集行数和时间戳间隔。

---

## 扩展其他输入设备

在 `robots/` 下新建文件，实现 `utils/interfaces.py` 中定义的接口，然后在 `scripts/record_data.py` 的 `make_arm_readers()` 里加一个分支即可。采集、存储、相机逻辑全部不需要改动。

```python
# utils/interfaces.py 中定义的四个接口

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

**接 VR 示例：**

```
robots/leader_vr.py              # 实现 LeaderArmInterface，读 VR 控制器
robots/teleop_vr_controller.py   # 实现 TeleopControllerInterface，做笛卡尔空间映射
```

`make_arm_readers()` 里加：

```python
elif t == "vr_slave":
    leader = VRLeaderArm(...)
    follower = A1ZFollowerArm(...)
    controller = VRTeleopController(...)
```
