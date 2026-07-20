"""A1Z 机械臂 IK 控制器 — 移植自 vr_joy 已验证方案。

核心差异 vs a1z.robots.kinematics.Kinematics:
  - 使用 arm_link6 作为末端 frame（而非默认的 gripper_finger_rIght_link FIXED_JOINT）
  - 增量式 6-DOF IK（去 roll）+ roll 投影分离（j1-j5 做位置+无 roll 姿态，j6 做 roll）
  - 使用 pinocchio.LOCAL_WORLD_ALIGNED Jacobian（与误差计算坐标系一致）
  - 跨帧维持热启动状态 self._q，5 次外迭代收敛
"""

from __future__ import annotations

import numpy as np
import pinocchio

from robots.vr_filter import WeightedMovingFilter


class ArmIKBase:
    """工作空间映射 + 通用 IK 模板 — 移植自 vr_joy/vr_teleop/arms/base.py。

    设计原则（参考宇树 xr_teleoperate）:
        - 映射策略和 IK 算法在基类完成，子类只声明机型常量。
        - N_IK_DOFS 维阻尼最小二乘 IK（位置 + 去 roll 姿态，6-DOF 任务），
          剩余 N_TOTAL_DOFS - N_IK_DOFS 维 roll 投影到 EE 局部轴。
    """

    # ----- 子类必须覆盖 -----
    N_IK_DOFS: int = None        # IK 求解维度（= N_TOTAL_DOFS - 1）
    N_TOTAL_DOFS: int = None     # 总 DOF

    # ----- 共用算法参数（子类可覆盖）-----
    POSITION_SCALE = 1.0
    IK_DAMPING = 0.05            # JJT 阻尼系数
    IK_MAX_STEP = 0.5            # 单次迭代最大步长 (rad)
    N_IK_ITER = 5                # 每帧外迭代次数
    IK_CONVERGE_THRESH = 0.002   # 位置收敛阈值 (m)
    ORI_CONVERGE_THRESH = 0.02   # 姿态收敛阈值 (rad)
    ROLL_AXIS_LOCAL = 0          # 0=EE局部X轴, 1=Y, 2=Z
    FILTER_WEIGHTS = (0.1, 0.2, 0.3, 0.4)
    NULL_SPACE_GAIN: float = 0.0  # 非冗余臂(A1Z)为 0

    @property
    def q_ref(self) -> np.ndarray:
        """零空间参考姿态（子类可覆盖）。"""
        return np.zeros(self.N_IK_DOFS)

    def __init__(
        self,
        urdf_path: str,
        ee_frame: str,
        joint_names: list,
        arm_base_link: str | None = None,
    ):
        """
        Args:
            urdf_path: URDF 文件路径。
            ee_frame: 末端 frame 名（用于 IK 目标），如 "arm_link6"。
            joint_names: 该臂所有关节名列表（长度 = N_TOTAL_DOFS）。
            arm_base_link: 臂基座 frame 名。None 时回退到 oMf[1]。
        """
        assert self.N_IK_DOFS is not None and self.N_TOTAL_DOFS is not None, \
            "子类必须设置 N_IK_DOFS / N_TOTAL_DOFS"
        assert self.N_TOTAL_DOFS > self.N_IK_DOFS, \
            "N_TOTAL_DOFS 必须大于 N_IK_DOFS (至少 1 个映射 DOF)"
        assert len(joint_names) == self.N_TOTAL_DOFS

        self._model = pinocchio.buildModelFromUrdf(urdf_path)
        self._data = self._model.createData()
        self._ee_frame_id = self._model.getFrameId(ee_frame)
        self._q = np.zeros(self._model.nq)

        pinocchio.forwardKinematics(self._model, self._data, self._q)
        pinocchio.updateFramePlacements(self._model, self._data)

        if arm_base_link is None:
            self._arm_base_pos = self._data.oMf[1].translation.copy()
        else:
            fid = self._model.getFrameId(arm_base_link)
            self._arm_base_pos = self._data.oMf[fid].translation.copy()

        self._ee_zero_rot = self._data.oMf[self._ee_frame_id].rotation.copy()
        self._filter = WeightedMovingFilter(
            np.array(self.FILTER_WEIGHTS), data_size=self.N_TOTAL_DOFS
        )
        self._target_pos = None
        self.JOINT_NAMES = list(joint_names)

    # ----- 公开 API -----

    def calibrate(self):
        """重置滤波器。"""
        self._filter.reset()

    def sync_state(self, q: np.ndarray):
        """将 IK 热启动初值同步到当前构型，并重置滤波器。

        在重新锚定 / 恢复遥操作 / 回零后调用，避免 IK 从旧构型出发。

        Args:
            q: 机械臂当前 N_TOTAL_DOFS 维构型。
        """
        q = np.asarray(q, dtype=np.float64)
        self._q = np.zeros(self._model.nq)
        self._q[: len(q)] = q
        self._filter.reset()
        pinocchio.forwardKinematics(self._model, self._data, self._q)
        pinocchio.updateFramePlacements(self._model, self._data)

    def get_ee_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """返回当前 q 对应的末端位姿 (translation, rotation_matrix)。"""
        pinocchio.forwardKinematics(self._model, self._data, self._q)
        pinocchio.updateFramePlacements(self._model, self._data)
        M = self._data.oMf[self._ee_frame_id]
        return M.translation.copy(), M.rotation.copy()

    def set_ee_target(self, pos: np.ndarray, rot_mat: np.ndarray) -> np.ndarray:
        """设置 EE 目标位姿并求解 IK。

        Args:
            pos: 目标位置 [x, y, z]（机器人基坐标系）。
            rot_mat: 目标姿态 3x3 旋转矩阵（机器人基坐标系）。

        Returns:
            N_TOTAL_DOFS 维关节目标角度（已滤波）。
        """
        self._target_pos = np.asarray(pos, dtype=np.float64)
        q_target = self._compute_ik(self._target_pos, rot_mat)
        q_target = self._filter.next(q_target)
        self._q[: self.N_TOTAL_DOFS] = q_target
        return q_target

    def get_ee_error(self) -> float:
        """当前 EE 位姿与目标位姿的距离 (m)。"""
        if self._target_pos is None:
            return 0.0
        pinocchio.forwardKinematics(self._model, self._data, self._q)
        pinocchio.updateFramePlacements(self._model, self._data)
        return float(np.linalg.norm(
            self._target_pos - self._data.oMf[self._ee_frame_id].translation
        ))

    # ----- 内部 IK 求解 -----

    def _compute_ik(self, target_pos: np.ndarray, target_rot: np.ndarray) -> np.ndarray:
        """增量 IK（移植自 vr_joy 已验证方案）。

        前 N_IK_DOFS 维做 6-DOF 阻尼最小二乘 IK（位置 + 去掉 roll 的姿态），
        剩余维做 roll 直接投影到 EE 局部 ROLL 轴。

        以 self._q 为热启动初值，执行 N_IK_ITER 次外迭代。
        """
        n_ik = self.N_IK_DOFS          # 5 (j1-j5)
        n_mapped = self.N_TOTAL_DOFS - n_ik  # 1 (j6, roll)

        model, data = self._model, self._data
        fid = self._ee_frame_id
        q = self._q.copy()

        # j6 (roll DOF) 先清零，由最后投影重新计算
        for i in range(n_mapped):
            q[n_ik + i] = 0.0

        for _ in range(self.N_IK_ITER):
            pinocchio.forwardKinematics(model, data, q)
            pinocchio.updateFramePlacements(model, data)
            oMf = data.oMf[fid]
            ee_pos = oMf.translation
            ee_rot = oMf.rotation.copy()

            pos_err = target_pos - ee_pos
            R_err_world = target_rot @ ee_rot.T
            ori_err = pinocchio.log3(R_err_world)

            # 去掉 roll 分量：沿 EE 局部 ROLL 轴方向的姿态误差留给 j6 处理
            roll_dir = ee_rot[:, self.ROLL_AXIS_LOCAL]
            ori_err_no_roll = ori_err - np.dot(ori_err, roll_dir) * roll_dir

            if (np.linalg.norm(pos_err) < self.IK_CONVERGE_THRESH
                    and np.linalg.norm(ori_err_no_roll) < self.ORI_CONVERGE_THRESH):
                break

            # j1-j5: 求解 (位置 + 去 roll 姿态) 共 6-DOF 任务
            error = np.concatenate([pos_err, ori_err_no_roll])
            J_full = pinocchio.computeFrameJacobian(
                model, data, q, fid, pinocchio.LOCAL_WORLD_ALIGNED
            )
            J = J_full[:, :n_ik]  # 6×5 雅可比 (j1-j5)

            JJT = J @ J.T + self.IK_DAMPING ** 2 * np.eye(6)
            dq = J.T @ np.linalg.solve(JJT, error)

            dq_norm = np.linalg.norm(dq)
            if dq_norm > self.IK_MAX_STEP:
                dq = dq * (self.IK_MAX_STEP / dq_norm)

            q[:n_ik] += dq
            for i in range(n_ik):
                q[i] = np.clip(q[i], model.lowerPositionLimit[i], model.upperPositionLimit[i])

        # j6 (roll): 将剩余姿态误差投影到 EE 局部 ROLL 轴
        pinocchio.forwardKinematics(model, data, q)
        pinocchio.updateFramePlacements(model, data)
        ee_rot_curr = data.oMf[fid].rotation.copy()
        R_remaining = target_rot @ ee_rot_curr.T
        remaining = pinocchio.log3(R_remaining)
        roll_dir = ee_rot_curr[:, self.ROLL_AXIS_LOCAL]
        for i in range(n_mapped):
            j_idx = n_ik + i
            j_val = float(np.dot(remaining, roll_dir))
            q[j_idx] = np.clip(j_val, model.lowerPositionLimit[j_idx], model.upperPositionLimit[j_idx])

        return q[:self.N_TOTAL_DOFS].copy()


class A1ZArmIK(ArmIKBase):
    """A1Z 6-DOF 桌面机械臂：j1-j5 做 6-DOF IK（去掉 roll）+ j6 做 roll 投影。

    移植自 vr_joy 已验证方案：
    j1-j5 阻尼最小二乘求解位置+去 roll 姿态（6-DOF 任务），
    j6 将剩余姿态误差投影到 EE 局部 ROLL 轴直接映射。
    """

    N_IK_DOFS = 5
    N_TOTAL_DOFS = 6

    def __init__(self, urdf_path: str):
        super().__init__(
            urdf_path=urdf_path,
            ee_frame="arm_link6",
            joint_names=[f"arm_joint{i}" for i in range(1, 7)],
            arm_base_link=None,  # A1Z URDF root 即臂基座，沿用 oMf[1]
        )
