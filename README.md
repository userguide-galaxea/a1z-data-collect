# a1z_data_collection

A1Z 机械臂遥操作数据采集工具。一条命令同时启动遥操作和数据记录，支持单臂/双臂主从模式，关节数据 30Hz、相机 30Hz 分离采集，存储为 HDF5 格式。

---

## 安装

### 1. 克隆项目

```bash
git clone --recurse-submodules https://github.com/userguide-galaxea/a1z-data-collect.git
cd a1z_data_collection
```

如果已经 clone 但忘了带 `--recurse-submodules`：

```bash
git submodule update --init --recursive
```

### 2. 安装依赖

> **注意**：`GALAXEA-A1Z` 是 `open-a1z-t` 内的嵌套 submodule。安装前请先确认已完整初始化所有 submodule（`--recurse-submodules` 或 `git submodule update --init --recursive` 会自动处理）。若 `open-a1z-t/GALAXEA-A1Z` 目录为空，运行：
> ```bash
> cd open-a1z-t && git submodule update --init && cd ..
> ```

```bash
pip install -e open-a1z-t
pip install -e open-a1z-t/GALAXEA-A1Z
pip install -e .
```

### 3. 配置 CAN 总线

```bash
bash open-a1z-t/scripts/setup_can.sh
```

---

## 项目结构

```
a1z_data_collection/
├── cfg/
│   └── two_master_slave.yaml       # 示例配置
├── collectors/
│   ├── arm_collector.py            # 30Hz 遥操作 + 关节记录线程
│   └── camera_collector.py         # 30Hz 相机记录线程
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
│   └── record_data.py              # 主入口
└── open-a1z-t/                     # git submodule
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
  start_episode: null                # 起始编号，null 表示自动续接已有数据

arm_cfg:
  collection_type: "two_master_slave"  # 采集模式，见下表
  leader_port_left: "/dev/ttyACM0"    # 左主臂串口
  leader_port_right: "/dev/ttyACM1"   # 右主臂串口（单臂模式不需要）
  follower_can_left: "can0"           # 左从臂 CAN 通道
  follower_can_right: "can1"          # 右从臂 CAN 通道（单臂模式不需要）
  use_velocity: false                  # 是否保存关节速度
  use_effort: false                    # 是否保存关节力矩（开发中，暂未实现）

camera_cfg:
  camera_names: ["cam_high", "cam_right_wrist", "cam_left_wrist"]
  topics:                             # 各相机对应的 USB 设备号或路径
    cam_high: 0                       # 留空则启动时弹出交互式标定窗口
    cam_right_wrist: 1
    cam_left_wrist: 2

collector_cfg:
  arm_freq: 30           # 关节采集频率（Hz）
  camera_freq: 30        # 相机采集频率（Hz）
  cpu_affinity: null     # 关节采集线程绑定的 CPU 核号，null 表示不绑定
  realtime_priority: false  # 开启实时调度优先级，需要 root 权限
```

**采集模式（collection_type）：**

| 模式 | 说明 | state | action |
|------|------|-------|--------|
| `one_master` | 单臂，无从臂 | 主臂关节位置 (7,) | 同 state |
| `two_master` | 双臂，无从臂 | 双臂关节位置 (14,) | 同 state |
| `one_master_slave` | 单臂主从遥操作 | 从臂关节位置 (7,) | 主臂关节位置 (7,) |
| `two_master_slave` | 双臂主从遥操作 | 双臂从臂位置 (14,) | 双臂主臂位置 (14,) |

7 维 = [j1..j6 rad, gripper_norm]，gripper_norm 范围 0.0（张开）~ 1.0（闭合）。双臂为 14 维 = [left(7), right(7)]。

---

## 相机标定

三个相机型号相同，无法通过序列号区分。每次启动时有两种方式指定相机：

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
2. 按提示依次点击对应的相机预览
3. 按回车确认，完成后自动进入采集流程

---

## 录制数据

### 启动

```bash
cd a1z_data_collection
python -m scripts.record_data config_file=cfg/my_task.yaml
```

启动后程序自动完成：主臂初始化 → 从臂上电 → 相机初始化（或标定）→ 等待开始指令。

可以在命令行覆盖 yaml 中的单个参数：

```bash
python -m scripts.record_data config_file=cfg/my_task.yaml task_description="cup stacking" num_episodes=50
```

需要实时调度优先级时（提升关节采集精度）：

```bash
sudo python -m scripts.record_data config_file=cfg/my_task.yaml collector_cfg.realtime_priority=true
```

### 键盘操作

| 按键 | 说明 |
|------|------|
| `S` | 开始采集当前 episode |
| `E` | 结束并保存当前 episode |
| `R` | 丢弃当前 episode，重新采集 |
| `Q` | 退出程序 |

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
        ├── 按 S ──► 遥操作线程（30Hz）+ 相机线程（30Hz）同时启动
        │             从臂实时跟随主臂运动
        │
        ├── 按 E ──► 线程停止，保存 episode_N.hdf5
        ├── 按 R ──► 丢弃本次，重新等待 S
        └── 按 Q ──► 退出
```

---

## 数据格式

数据保存在 `dataset_dir/task_name/episode_N.hdf5`：

```
episode_N.hdf5
├── attrs["task"]                       # 任务描述字符串
├── arm/
│   ├── state               float32 [T, D]   # 从臂关节位置，D=7(单臂)/14(双臂)
│   ├── action              float32 [T, D]   # 主臂关节位置（经控制器处理后）
│   ├── action_raw          float32 [T, D]   # 主臂原始读取位置
│   ├── leader_timestamps   float64 [T]      # 主臂读取完成时刻
│   ├── follower_timestamps float64 [T]      # 从臂读取完成时刻
│   ├── velocity            float32 [T, D]   # 可选，use_velocity=true，控制器输出速度
│   └── velocity_raw        float32 [T, D]   # 可选，use_velocity=true，主臂原始速度
└── cameras/
    └── <cam_name>/
        ├── frames      vlen uint8            # JPEG 字节，每帧独立存储，30Hz
        └── timestamps  float64 [T]          # 帧读取完成时刻
```

所有时间戳基于同一单调时钟，可跨模态直接对齐。

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
