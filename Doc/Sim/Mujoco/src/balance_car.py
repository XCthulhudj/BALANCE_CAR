#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""balance_car.py —— 两轮自平衡车 MuJoCo 闭环仿真

这是 ``Doc/Sim/BalanceCar_InvertedPendulum.m``（由 motionCtrlTask.c 的 PID 逻辑还原）
的 MuJoCo 物理版本：

* 控制器与固件完全一致 —— 位置式 PID，速度环(外) + 直立环(内) + 转向环，
  增益、限幅、积分饱和处理均照搬，控制周期 2 ms (对应 TASK_FREQ_MOTIONCTRL 500 Hz)；
* 被控对象不再是手写的 cart-pole 微分方程，而是真正的多刚体动力学：
  地面接触、轮子滚动、转动惯量、摩擦锥全部交给 MuJoCo 解算；
* 传感器按真实车的方式读取：编码器量来自轮子角速度（打滑时与真实车速不一致，
  这点和实车一致），倾角来自车身俯仰角，陀螺 Z 轴在平面约束模型里恒为 0。

平面化说明
    真车有横滚/偏航自由度，需要额外的侧向平衡环才能站住。为了和 .m 参考模型
    （只有 x 与 pitch 两个状态）对齐，这里用 slide_x + slide_z + pitch 三个关节
    把车约束在 x-z 竖直平面内：横滚、偏航被约束锁定，因此转向环输出恒为 0
    （与 .m 中 gyro_z≈0 的假设一致），但转向环的代码路径仍然保留。

关于两套参数（重要）
    ``--preset firmware`` 是 .m 里的原始整定，原样复现，行为也和 .m 一致：
        倾角能收住，但速度会一直单方向发散（10 s 时 -17 m/s 左右）。
        原因是速度环的输出极性与直立环不匹配 —— 速度误差被翻译成了"朝当前
        行进方向倾斜"，于是重力持续供能，形成正反馈；速度环积分限幅只有
        0.01 deg，等于没有积分，兜不住这个偏差。
    ``--preset stable`` （默认）是重新整定的一套：把外环极性取反、压低速度环
        增益、给积分真正的权限、并大幅加大直立环的微分项。8 deg 的初始倾角
        没有过冲，2 m/s 阶跃跟踪误差约 5%。
    两套都在 ``--drive force``（等价 .m 推力模型）下标定；``--drive torque``
    走轮胎摩擦，会略软一点，需要时可在其基础上再压 Kp、加 Kd。

用法::

    python src/balance_car.py                    # 可视化，默认 stable 参数
    python src/balance_car.py --headless         # 无窗口，打印结果表
    python src/balance_car.py --preset firmware  # 复现 .m 原始参数（会跑飞）
    python src/balance_car.py --headless --csv log.csv --svg report.svg
    python src/balance_car.py --dump-xml car.xml # 导出 MJCF 方便在别处打开
    python src/balance_car.py --theta0 -12 --v-set 200   # 改初始倾角 / 目标车速
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from dataclasses import dataclass

import numpy as np

try:
    import mujoco
except ImportError:  # pragma: no cover
    sys.exit(
        "找不到 mujoco，请先激活虚拟环境后安装:\n"
        "    .venv\\Scripts\\python.exe -m pip install mujoco"
    )

# Windows 控制台默认是 GBK，中文会炸；这里强制用 UTF-8 输出。
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):  # pragma: no cover
        pass


# --------------------------------------------------------------------------- #
# 1. 物理参数（与 BalanceCar_InvertedPendulum.m 第 6~25 行一一对应）
# --------------------------------------------------------------------------- #
@dataclass
class CarParams:
    """整车物理参数 + 传感器标定。"""

    g: float = 9.81        # 重力加速度 m/s^2
    m: float = 0.60        # 车身质量 kg
    M: float = 0.20        # 轮子等效质量 kg（两轮合计）
    l: float = 0.07        # 车身质心到轮轴距离 m
    I: float = 0.006       # 车身绕轮轴的转动惯量 kg*m^2
    b: float = 0.04        # 平移阻尼 N/(m/s)
    c: float = 0.002       # 旋转阻尼 N*m/(rad/s)
    r: float = 0.03175     # 轮子半径 m
    Ts: float = 0.002      # 控制周期 s（500 Hz，对应 TASK_FREQ_MOTIONCTRL）
    Tend: float = 10.0     # 默认仿真时长 s
    theta0: float = -8.0   # 初始倾角 deg（.m 中为 deg2rad(-8)）
    enc_gain: float = 100.0  # 编码器增益：1 m/s -> 100 counts
    pwm_gain: float = 0.01   # 推力增益：1000 PWM -> 10 N

    @property
    def total_mass(self) -> float:
        return self.m + self.M


# --------------------------------------------------------------------------- #
# 2. 控制器参数（默认值取自 .m 第 27~43 行）
# --------------------------------------------------------------------------- #
@dataclass
class Gains:
    # 直立环（内环）：输入是"目标倾角 - 实际倾角"，单位 deg，输出 PWM
    kp_pitch: float = 200.0
    ki_pitch: float = 0.0
    kd_pitch: float = 10.0
    max_out_pitch: float = 3000.0
    max_iout_pitch: float = 500.0

    # 速度环（外环）：编码器量，输出作为直立环的目标倾角（单位 deg）
    kp_vel: float = 0.1
    ki_vel: float = 0.001
    kd_vel: float = 0.001
    max_out_vel: float = 10.0
    max_iout_vel: float = 0.01

    # 转向环：仅 P
    kp_turn: float = 1.0
    ki_turn: float = 0.0
    kd_turn: float = 0.0
    max_out_turn: float = 1.0
    max_iout_turn: float = 1.0

    # 设定值
    v_enc_set: float = 0.0
    wz_enc_set: float = 0.0

    # 外环极性：+1 表示 err = v_set - v_enc（与 .m 相同），-1 表示取反。
    # 实车上这一位等价于"编码器速度方向"或"速度环输出叠加方式"的正负，
    # 装车时如果发现一向就加速，就是这里反了。
    vel_err_sign: float = 1.0


# .m 第 29~39 行的原始整定，用于原样复现参考模型的行为
FIRMWARE = Gains()

# 重新整定的版本：能真正站住并跟踪设定车速
STABLE = Gains(
    kp_pitch=60.5, ki_pitch=0.0, kd_pitch=394.6,
    max_out_pitch=3000.0, max_iout_pitch=500.0,
    kp_vel=0.0169, ki_vel=0.00024, kd_vel=0.001,
    max_out_vel=8.6, max_iout_vel=7.40,
    vel_err_sign=-1.0,
)

PRESETS = {"firmware": FIRMWARE, "stable": STABLE}


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else (hi if x > hi else x)


class Pid:
    """位置式 PID，行为与 usr_pid_caculate() 一致。

    注意积分项不乘 Ts —— 原 C 代码与 .m 都是这么写的（.m 里也有注释提到这点），
    照搬可以让实车标定值原样迁移。
    """

    __slots__ = ("kp", "ki", "kd", "max_out", "max_iout", "err1", "err2", "iout", "out")

    def __init__(self, kp: float, ki: float, kd: float, max_out: float, max_iout: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.max_iout = max_iout
        self.reset()

    def reset(self) -> None:
        self.err1 = 0.0   # 当前误差
        self.err2 = 0.0   # 上一次误差
        self.iout = 0.0
        self.out = 0.0

    def update(self, err: float) -> float:
        self.err2 = self.err1
        self.err1 = err

        pout = self.kp * err
        if self.ki:
            self.iout = _clamp(self.iout + self.ki * err, -self.max_iout, self.max_iout)
        dout = self.kd * (self.err1 - self.err2)   # 已隐含 /Ts，与原代码一致

        self.out = _clamp(pout + self.iout + dout, -self.max_out, self.max_out)
        return self.out


# --------------------------------------------------------------------------- #
# 3. 传感器快照与执行器指令
# --------------------------------------------------------------------------- #
@dataclass
class Sensors:
    x: float            # 位移 m（真值，仅用于记录）
    xdot: float         # 车速 m/s（真值）
    pitch_deg: float    # 车身倾角 deg（车头方向为正）
    pitch_rate: float   # 俯仰角速度 deg/s
    wheel_l: float      # 左轮角速度 rad/s
    wheel_r: float      # 右轮角速度 rad/s
    v_enc: float        # 编码器测得的车速（counts）
    gyro_z: float       # 陀螺 Z 轴 rad/s（平面模型恒为 0）


@dataclass
class Command:
    pwm_l: float
    pwm_r: float
    thrust: float       # 期望合力 N
    vel_out: float
    pitch_out: float
    turn_out: float


class BalanceController:
    """三段串级 PID，逻辑与 motionCtrlTask.c 的 motionCtrl 任务一致。"""

    def __init__(self, g: Gains, p: CarParams):
        self.gains = g
        self.params = p
        self.pitch = Pid(g.kp_pitch, g.ki_pitch, g.kd_pitch, g.max_out_pitch, g.max_iout_pitch)
        self.vel = Pid(g.kp_vel, g.ki_vel, g.kd_vel, g.max_out_vel, g.max_iout_vel)
        self.turn = Pid(g.kp_turn, g.ki_turn, g.kd_turn, g.max_out_turn, g.max_iout_turn)

    def reset(self) -> None:
        for pid in (self.pitch, self.vel, self.turn):
            pid.reset()

    def update(self, s: Sensors) -> Command:
        g = self.gains

        # --- 外环：速度环，输出 = 目标倾角(deg) ---
        # 极性由 gains.vel_err_sign 决定，见文件头"关于两套参数"
        vel_out = self.vel.update(g.vel_err_sign * (g.v_enc_set - s.v_enc))

        # --- 内环：直立环，输入 = 目标倾角 - 实际倾角 ---
        pitch_out = self.pitch.update(vel_out - s.pitch_deg)

        # --- 转向环：只剩 P ---
        turn_out = self.turn.update(g.wz_enc_set - s.gyro_z)

        # --- 输出合成（左右电机镜像安装，故 pwm2 = -pwm1）---
        pwm_l = pitch_out + turn_out
        pwm_r = -pitch_out - turn_out

        # .m 中 force = pwm_cn1 * pwm_gain，即通道 1 的 PWM 数值直接换算成合力
        thrust = pwm_l * self.params.pwm_gain

        return Command(pwm_l=pwm_l, pwm_r=pwm_r, thrust=thrust,
                       vel_out=vel_out, pitch_out=pitch_out, turn_out=turn_out)


# --------------------------------------------------------------------------- #
# 4. MJCF 模型
# --------------------------------------------------------------------------- #
def build_xml(p: CarParams, mu: float = 1.6, dt: float = 0.0005) -> str:
    """生成两轮平衡车的 MJCF。

    坐标系约定：+x 向前、+z 向上、轮轴沿 y。
    MuJoCo 中绕 +y 的正向转动会把车顶压向 +x，即 pitch > 0 = 车头前倾，
    与 .m 里 theta 的正方向一致（.m 中 pitch_deg = -rad2deg(theta)）。
    """
    # 车身绕质心的转动惯量：I(绕轴) - m*l^2 ≈ 0.006 - 0.00294 ≈ 0.003
    i_com_y = max(p.I - p.m * p.l * p.l, 1e-5)
    wheel_half_w = 0.008
    wheel_mass = p.M / 2.0
    hub_r = p.r * 0.55

    return f"""
<mujoco model="balance_car">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{dt:.6f}" integrator="implicitfast" gravity="0 0 {-p.g}"
          iterations="80" ls_iterations="40"/>

  <visual>
    <global offwidth="1280" offheight="720"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.6 0.6 0.6" specular="0 0 0"/>
    <map znear="0.01" zfar="50" haze="0.1"/>
    <quality shadowsize="2048"/>
  </visual>

  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.22 0.24 0.28" rgb2="0.30 0.32 0.36"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0.05"/>
    <material name="body"   rgba="0.16 0.47 0.86 1"/>
    <material name="dark"   rgba="0.13 0.14 0.16 1"/>
    <material name="tire"   rgba="0.09 0.09 0.10 1"/>
    <material name="hub"    rgba="0.80 0.82 0.86 1"/>
  </asset>

  <worldbody>
    <light name="sun" pos="0 0 4" dir="0.2 0.3 -1" directional="true"
           castshadow="true" diffuse="0.8 0.8 0.8"/>
    <geom name="floor" type="plane" size="0 0 0.05" material="grid" priority="1"
          friction="{mu:g} 0.005 0.0001"/>
    <camera name="side" pos="-0.35 -1.55 0.45" xyaxes="1 0 0 0 0 1"/>

    <!-- 车轴：只有平面内两个平移自由度 -->
    <body name="base" pos="0 0 {p.r:.5f}">
      <joint name="slide_x" type="slide" axis="1 0 0" damping="{p.b:g}"/>
      <joint name="slide_z" type="slide" axis="0 0 1"/>
      <inertial pos="0 0 0" mass="0.01" diaginertia="1e-5 1e-5 1e-5"/>

      <body name="wheel_l" pos="0 0.031 0">
        <joint name="wheel_l" type="hinge" axis="0 1 0" armature="2e-05" damping="2e-04"/>
        <geom name="wheel_l_tire" type="cylinder" size="{p.r:.5f} {wheel_half_w}"
              euler="1.5707963 0 0" mass="{wheel_mass:.4f}" material="tire"
              friction="{mu:g} 0.005 0.0001"/>
        <geom name="wheel_l_hub" type="cylinder" size="{hub_r:.5f} {wheel_half_w + 0.0008:.5f}"
              euler="1.5707963 0 0" material="hub" mass="1e-6" contype="0" conaffinity="0"/>
      </body>

      <body name="wheel_r" pos="0 -0.031 0">
        <joint name="wheel_r" type="hinge" axis="0 1 0" armature="2e-05" damping="2e-04"/>
        <geom name="wheel_r_tire" type="cylinder" size="{p.r:.5f} {wheel_half_w}"
              euler="1.5707963 0 0" mass="{wheel_mass:.4f}" material="tire"
              friction="{mu:g} 0.005 0.0001"/>
        <geom name="wheel_r_hub" type="cylinder" size="{hub_r:.5f} {wheel_half_w + 0.0008:.5f}"
              euler="1.5707963 0 0" material="hub" mass="1e-6" contype="0" conaffinity="0"/>
      </body>

      <!-- 车身：绕车轴俯仰，质心在轴上方 l 处 -->
      <body name="chassis" pos="0 0 0">
        <joint name="pitch" type="hinge" axis="0 1 0" damping="{p.c:g}"/>
        <inertial pos="0 0 {p.l:.5f}" mass="{p.m:g}"
                  diaginertia="0.0015 {i_com_y:.6f} 0.003"/>
        <geom name="chassis_box" type="box" size="0.032 0.024 0.048" pos="0 0 0.088"
              material="body"/>
        <geom name="chassis_top" type="box" size="0.018 0.018 0.018" pos="0 0 0.150"
              material="dark"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <!-- 合力驱动：等价于 .m 中直接作用在车轴上的 force -->
    <motor name="thrust" joint="slide_x" gear="1" ctrllimited="true" ctrlrange="-80 80"/>
    <!-- 轮毂电机力矩驱动：两条通路按需选用（默认只启用 thrust） -->
    <motor name="torque_l" joint="wheel_l" gear="1" ctrllimited="true" ctrlrange="-6 6"/>
    <motor name="torque_r" joint="wheel_r" gear="1" ctrllimited="true" ctrlrange="-6 6"/>
  </actuator>
</mujoco>
""".strip()


class Ids:
    """把常用的关节/执行器 id 预先查好，避免循环里反复做字符串查找。"""

    def __init__(self, model: "mujoco.MjModel"):
        self.slide_x_q = model.jnt_qposadr[Ids._joint(model, "slide_x")]
        self.slide_x_v = model.jnt_dofadr[Ids._joint(model, "slide_x")]
        self.pitch_q = model.jnt_qposadr[Ids._joint(model, "pitch")]
        self.pitch_v = model.jnt_dofadr[Ids._joint(model, "pitch")]
        self.wheel_l_v = model.jnt_dofadr[Ids._joint(model, "wheel_l")]
        self.wheel_r_v = model.jnt_dofadr[Ids._joint(model, "wheel_r")]
        self.act_thrust = Ids._actuator(model, "thrust")
        self.act_l = Ids._actuator(model, "torque_l")
        self.act_r = Ids._actuator(model, "torque_r")

    @staticmethod
    def _joint(model, name: str) -> int:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise RuntimeError(f"模型里没有关节 {name!r}")
        return jid

    @staticmethod
    def _actuator(model, name: str) -> int:
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if aid < 0:
            raise RuntimeError(f"模型里没有执行器 {name!r}")
        return aid


# --------------------------------------------------------------------------- #
# 5. 仿真主体
# --------------------------------------------------------------------------- #
DRIVE_FORCE = "force"    # 合力直接作用在车轴（等价 .m 的推力模型，默认）
DRIVE_TORQUE = "torque"  # 轮毂电机力矩 + 轮胎摩擦（更接近真车，依赖地面摩擦）


class BalanceSim:
    def __init__(self, params: CarParams, gains: Gains, *, drive: str = DRIVE_FORCE,
                 mu: float = 1.6, ctrl_dt: float = 0.002, substeps: int = 4,
                 preset_name: str = "stable"):
        self.params = params
        self.ctrl_dt = ctrl_dt
        self.substeps = substeps
        self.drive = drive
        self.preset_name = preset_name
        self.xml = build_xml(params, mu=mu, dt=ctrl_dt / substeps)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.ids = Ids(self.model)
        self.ctrl = BalanceController(gains, params)
        self.log: dict[str, list[float]] = {}
        self.fallen = False
        self.fall_time = math.inf
        self.time = 0.0

    # --- 初始化 ---------------------------------------------------------- #
    def reset(self, theta0_deg: float | None = None) -> None:
        mujoco.mj_resetData(self.model, self.data)
        th0 = self.params.theta0 if theta0_deg is None else theta0_deg
        self.data.qpos[self.ids.slide_x_q] = 0.0
        self.data.qpos[self.ids.pitch_q] = math.radians(th0)
        mujoco.mj_forward(self.model, self.data)
        self.ctrl.reset()
        self.fallen = False
        self.fall_time = math.inf
        self.time = 0.0
        self.log = {
            "t": [], "x": [], "xdot": [], "pitch": [], "pitch_rate": [],
            "v_enc": [], "pwm_l": [], "pwm_r": [], "thrust": [],
            "wheel_l": [], "wheel_r": [], "vel_out": [], "ncon": [],
        }

    # --- 传感器 ---------------------------------------------------------- #
    def read_sensors(self) -> Sensors:
        p, d, ids = self.params, self.data, self.ids
        wl = float(d.qvel[ids.wheel_l_v])
        wr = float(d.qvel[ids.wheel_r_v])
        # 真实编码器读的是轮子转速而不是对地速度：打滑时两者不同
        v_enc = 0.5 * (wl + wr) * p.r * p.enc_gain
        return Sensors(
            x=float(d.qpos[ids.slide_x_q]),
            xdot=float(d.qvel[ids.slide_x_v]),
            pitch_deg=-math.degrees(float(d.qpos[ids.pitch_q])),   # 与 .m 同号约定
            pitch_rate=-math.degrees(float(d.qvel[ids.pitch_v])),
            wheel_l=wl,
            wheel_r=wr,
            v_enc=v_enc,
            gyro_z=0.0,   # 平面约束模型：偏航自由度被锁死
        )

    # --- 单步：算一次控制 + 走若干物理步 -------------------------------- #
    def advance(self) -> tuple[Sensors, Command]:
        s = self.read_sensors()
        cmd = self.ctrl.update(s)

        if self.drive == DRIVE_FORCE:
            self.data.ctrl[self.ids.act_thrust] = cmd.thrust
            self.data.ctrl[self.ids.act_l] = 0.0
            self.data.ctrl[self.ids.act_r] = 0.0
        else:
            # 每个轮子分担一半推力：T = F * r / 2；转向环做差速
            half = 0.5 * cmd.thrust * self.params.r
            turn = 0.5 * cmd.turn_out * self.params.pwm_gain * self.params.r
            self.data.ctrl[self.ids.act_thrust] = 0.0
            self.data.ctrl[self.ids.act_l] = half + turn
            self.data.ctrl[self.ids.act_r] = half - turn

        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)
        self.time += self.ctrl_dt

        self._record(s, cmd)
        if abs(s.pitch_deg) > 60.0:
            self.fallen = True
            self.fall_time = self.time
        return s, cmd

    def _record(self, s: Sensors, cmd: Command) -> None:
        log = self.log
        log["t"].append(self.time)
        log["x"].append(s.x)
        log["xdot"].append(s.xdot)
        log["pitch"].append(s.pitch_deg)
        log["pitch_rate"].append(s.pitch_rate)
        log["v_enc"].append(s.v_enc)
        log["pwm_l"].append(cmd.pwm_l)
        log["pwm_r"].append(cmd.pwm_r)
        log["thrust"].append(cmd.thrust)
        log["wheel_l"].append(s.wheel_l)
        log["wheel_r"].append(s.wheel_r)
        log["vel_out"].append(cmd.vel_out)
        log["ncon"].append(self.data.ncon)


# --------------------------------------------------------------------------- #
# 6. 运行循环
# --------------------------------------------------------------------------- #
def _follow_camera(viewer, sim: BalanceSim) -> None:
    """让自由相机跟着车走，否则车一跑出画面就看不见了。"""
    try:
        cam = viewer.cam
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = (float(sim.data.qpos[sim.ids.slide_x_q]), 0.0, 0.12)
        cam.distance = 1.9
        cam.azimuth = 90.0      # 从 -y 方向看过去，即侧面
        cam.elevation = -12.0
    except Exception:  # pragma: no cover - 不同版本字段差异
        pass


def simulate(sim: BalanceSim, duration: float, viewer=None, speed: float = 1.0,
             render_hz: float = 60.0) -> int:
    """推进仿真；返回实际跑了多少个控制周期。"""
    n_ctrl = max(1, int(round(duration / sim.ctrl_dt)))
    every = max(1, int(round(1.0 / (sim.ctrl_dt * render_hz))))
    wall0 = time.perf_counter()
    done = 0

    for k in range(n_ctrl):
        sim.advance()
        done += 1
        if sim.fallen:
            break
        if viewer is not None:
            if k % every == 0:
                _follow_camera(viewer, sim)
                viewer.sync()
            # 按仿真时间对齐墙钟，保证窗口里的观感是实时的
            lag = (wall0 + sim.time / max(speed, 1e-6)) - time.perf_counter()
            if lag > 0:
                time.sleep(min(lag, 0.05))
    return done


# --------------------------------------------------------------------------- #
# 7. 结果输出：控制台表格 / CSV / 无依赖 SVG 曲线
# --------------------------------------------------------------------------- #
def summarize(sim: BalanceSim) -> dict[str, float]:
    log = sim.log
    pitch = np.asarray(log["pitch"])
    if pitch.size == 0:
        return {}
    thrust = np.asarray(log["thrust"])
    # 末段 0.5 s 的平均值比"最后一个采样点"更能说明是否真的收敛
    tail = min(250, pitch.size)
    return {
        "steps": float(len(pitch)),
        "balanced": float(not sim.fallen),
        "fall_time": float(sim.fall_time),
        "pitch_max": float(np.max(np.abs(pitch))),
        "pitch_mean_last": float(np.mean(pitch[-tail:])),
        "pitch_absmax_last": float(np.max(np.abs(pitch[-tail:]))),
        "xdot_mean_last": float(np.mean(log["xdot"][-tail:])),
        "x_final": float(log["x"][-1]),
        "v_enc_final": float(log["v_enc"][-1]),
        "thrust_max": float(np.max(np.abs(thrust))),
        "pwm_max": float(np.max(np.abs(log["pwm_l"]))),
        "wheel_max": float(np.max(np.abs(log["wheel_l"]))),
    }


def print_report(sim: BalanceSim, stats: dict[str, float]) -> None:
    if not stats:
        print("没有任何采样点。")
        return
    p = sim.params
    g = sim.ctrl.gains
    ok = "平衡成功" if stats["balanced"] else f"翻车 @ {stats['fall_time']:.3f} s"
    print("=" * 62)
    print(f"{'平衡车 MuJoCo 仿真结果':^56}")
    print("=" * 62)
    print(f"参数预置      : {sim.preset_name}   (外环极性 {g.vel_err_sign:+.0f})")
    print(f"驱动模型      : {sim.drive}")
    print(f"控制周期      : {sim.ctrl_dt * 1000:.1f} ms   "
          f"(物理步长 {sim.ctrl_dt / sim.substeps * 1e6:.0f} us x {sim.substeps})")
    print(f"仿真步数      : {int(stats['steps'])}   ({sim.time:.3f} s)")
    print(f"状态          : {ok}")
    print(f"最大倾角      : {stats['pitch_max']:.3f} deg")
    print(f"末 0.5 s 倾角 : 均值 {stats['pitch_mean_last']:+.4f} deg, "
          f"峰值 {stats['pitch_absmax_last']:.4f} deg")
    print(f"末 0.5 s 车速 : {stats['xdot_mean_last']:+.4f} m/s   "
          f"(编码器 {stats['v_enc_final']:+.2f} counts)")
    print(f"终了位移      : {stats['x_final']:+.4f} m")
    print(f"峰值推力      : {stats['thrust_max']:.3f} N   (峰值 PWM {stats['pwm_max']:.1f})")
    print(f"峰值轮速      : {stats['wheel_max']:.2f} rad/s "
          f"({stats['wheel_max'] * p.r:.3f} m/s)")

    # 积分饱和提示：.m 原样的 max_iout_vel=0.01 deg 等于把积分废掉
    vel_i = sim.ctrl.vel.iout
    if abs(vel_i) >= sim.ctrl.vel.max_iout - 1e-9:
        print(f"提示          : 速度环积分已顶在上限 (Iout={vel_i:+.4f} deg)；"
              "若 max_iout_vel 很小，外环实际上没有积分作用")
    print("=" * 62)


def write_csv(sim: BalanceSim, path: str) -> None:
    log = sim.log
    keys = list(log.keys())
    with open(path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(keys)
        for row in zip(*(log[k] for k in keys)):
            writer.writerow(row)
    print(f"CSV 已写入 {path}  ({len(log['t'])} 行)")


def write_svg(sim: BalanceSim, path: str) -> None:
    """零依赖画图：三张纵向排列的折线（倾角 / 编码器速度 / 位移）。"""
    log = sim.log
    t = np.asarray(log["t"])
    if t.size < 2:
        print("采样点太少，跳过 SVG。")
        return

    panels = [
        ("Pitch (deg)", np.asarray(log["pitch"]), 0.0),
        ("v_enc (counts)", np.asarray(log["v_enc"]), 0.0),
        ("x (m)", np.asarray(log["x"]), None),
    ]
    colors = ("#e04a4a", "#3a7bd5", "#2fa563")
    width, left, right, panel_h, gap, top = 960.0, 92.0, 26.0, 180.0, 42.0, 56.0
    height = top + len(panels) * panel_h + (len(panels) - 1) * gap + 56
    plot_w = width - left - right
    t0, t1 = float(t[0]), float(t[-1])
    t1 = t1 if t1 > t0 else t0 + 1e-3
    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" height="{height:.0f}" '
        f'font-family="Consolas,Menlo,monospace" font-size="12">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        f'<text x="{left:.0f}" y="26" font-size="15" font-weight="bold">'
        f'balance_car MuJoCo / drive={sim.drive} / preset={sim.preset_name} / '
        f'{"balanced" if not sim.fallen else "FALLEN"}</text>',
    ]

    for idx, (title, data, ref) in enumerate(panels):
        color = colors[idx]
        y_off = top + idx * (panel_h + gap)
        lo, hi = float(np.min(data)), float(np.max(data))
        if ref is not None:
            lo, hi = min(lo, ref), max(hi, ref)
        if hi - lo < 1e-9:
            lo, hi = lo - 1.0, hi + 1.0
        pad = (hi - lo) * 0.12
        lo, hi = lo - pad, hi + pad

        def sy(v: float, _y=y_off, _lo=lo, _hi=hi) -> float:
            return _y + panel_h - (v - _lo) / (_hi - _lo) * panel_h

        def sx(v: float) -> float:
            return left + (v - t0) / (t1 - t0) * plot_w

        out.append(f'<rect x="{left:.0f}" y="{y_off:.0f}" width="{plot_w:.0f}" '
                   f'height="{panel_h:.0f}" fill="#fbfbfd" stroke="#d7d9e0"/>')
        out.append(f'<text x="{left:.0f}" y="{y_off - 10:.0f}" font-weight="bold" '
                   f'fill="#33363d">{title}</text>')

        for i in range(5):
            frac = i / 4.0
            yy = y_off + panel_h - frac * panel_h
            val = lo + frac * (hi - lo)
            out.append(f'<line x1="{left:.0f}" y1="{yy:.1f}" x2="{left + plot_w:.0f}" '
                       f'y2="{yy:.1f}" stroke="#eceef3"/>')
            out.append(f'<text x="{left - 8:.0f}" y="{yy + 4:.1f}" fill="#8a8f9c" '
                       f'font-size="10" text-anchor="end">{val:+.3g}</text>')

        if ref is not None:
            out.append(f'<line x1="{left:.0f}" y1="{sy(ref):.1f}" x2="{left + plot_w:.0f}" '
                       f'y2="{sy(ref):.1f}" stroke="#b9bcc6" stroke-dasharray="4 4"/>')

        pts = " ".join(f"{sx(float(tt)):.1f},{sy(float(vv)):.1f}" for tt, vv in zip(t, data))
        out.append(f'<polyline fill="none" stroke="{color}" stroke-width="1.6" points="{pts}"/>')
        out.append(f'<text x="{left + plot_w - 4:.0f}" y="{y_off + panel_h + 14:.0f}" '
                   f'text-anchor="end" fill="#8a8f9c" font-size="10">t = {t1:.2f} s</text>')

    out.append("</svg>")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(out))
    print(f"SVG 已写入 {path}")


# --------------------------------------------------------------------------- #
# 8. 命令行
# --------------------------------------------------------------------------- #
def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = CarParams()
    ap = argparse.ArgumentParser(
        description="MuJoCo 两轮平衡车闭环仿真（PID 结构照搬 BalanceCar_InvertedPendulum.m）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("--preset", choices=tuple(PRESETS), default="stable",
                    help="firmware=.m 原始整定(会跑飞)；stable=重调整定")
    ap.add_argument("--duration", type=float, default=p.Tend, help="仿真时长 s")
    ap.add_argument("--theta0", type=float, default=p.theta0, help="初始倾角 deg")
    ap.add_argument("--ctrl-dt", type=float, default=p.Ts, help="控制周期 s")
    ap.add_argument("--substeps", type=int, default=4, help="每个控制周期内的物理子步数")
    ap.add_argument("--drive", choices=(DRIVE_FORCE, DRIVE_TORQUE), default=DRIVE_FORCE,
                    help="force=合力作用在车轴(等价 .m)；torque=轮毂电机力矩+轮胎摩擦")
    ap.add_argument("--mu", type=float, default=1.6, help="地面/轮胎摩擦系数")
    ap.add_argument("--headless", action="store_true", help="不开窗口，纯后台跑")
    ap.add_argument("--speed", type=float, default=1.0, help="可视化播放倍速")
    ap.add_argument("--csv", metavar="FILE", help="导出逐拍数据")
    ap.add_argument("--svg", metavar="FILE", help="导出三宫格曲线图")
    ap.add_argument("--dump-xml", metavar="FILE", help="把生成的 MJCF 写到文件")

    # 以下增益默认取自 --preset，显式给出时覆盖
    ap.add_argument("--kp-pitch", type=float, help="直立环 Kp")
    ap.add_argument("--ki-pitch", type=float, help="直立环 Ki")
    ap.add_argument("--kd-pitch", type=float, help="直立环 Kd（每步误差差分量，未除 Ts）")
    ap.add_argument("--kp-vel", type=float, help="速度环 Kp")
    ap.add_argument("--ki-vel", type=float, help="速度环 Ki")
    ap.add_argument("--kd-vel", type=float, help="速度环 Kd")
    ap.add_argument("--kp-turn", type=float, help="转向环 Kp")
    ap.add_argument("--max-vel-out", type=float, help="速度环输出限幅 deg（= 最大目标倾角）")
    ap.add_argument("--max-iout-vel", type=float, help="速度环积分限幅 deg")
    ap.add_argument("--v-set", type=float, help="编码器速度设定值（100 counts = 1 m/s）")
    ap.add_argument("--vel-err-sign", type=float, choices=(1.0, -1.0),
                    help="外环极性，+1 同 .m，-1 取反")
    return ap.parse_args(argv)


def make_gains(args: argparse.Namespace) -> Gains:
    """预置 + 命令行覆盖。"""
    base = PRESETS[args.preset]
    overrides = {
        "kp_pitch": args.kp_pitch, "ki_pitch": args.ki_pitch, "kd_pitch": args.kd_pitch,
        "kp_vel": args.kp_vel, "ki_vel": args.ki_vel, "kd_vel": args.kd_vel,
        "kp_turn": args.kp_turn, "max_out_vel": args.max_vel_out,
        "max_iout_vel": args.max_iout_vel, "v_enc_set": args.v_set,
        "vel_err_sign": args.vel_err_sign,
    }
    fields = {f: getattr(base, f) for f in Gains.__dataclass_fields__}
    fields.update({k: v for k, v in overrides.items() if v is not None})
    return Gains(**fields)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    params = CarParams(Ts=args.ctrl_dt, Tend=args.duration, theta0=args.theta0)
    gains = make_gains(args)

    try:
        sim = BalanceSim(params, gains, drive=args.drive, mu=args.mu,
                         ctrl_dt=args.ctrl_dt, substeps=args.substeps,
                         preset_name=args.preset)
    except Exception as exc:  # MJCF 拼错时给出可读信息
        print(f"模型构建失败: {exc}", file=sys.stderr)
        return 2

    if args.dump_xml:
        with open(args.dump_xml, "w", encoding="utf-8") as fh:
            fh.write(sim.xml)
        print(f"MJCF 已写入 {args.dump_xml}")

    sim.reset(args.theta0)

    viewer = None
    if not args.headless:
        try:
            import mujoco.viewer as mv
            viewer = mv.launch_passive(sim.model, sim.data)
        except Exception as exc:
            print(f"[warn] 打开可视化窗口失败（{exc}），改为无窗口运行")
            viewer = None

    if viewer is not None:
        with viewer:
            try:
                simulate(sim, args.duration, viewer=viewer, speed=args.speed)
            except KeyboardInterrupt:
                print("\n[已中断]")
    else:
        try:
            simulate(sim, args.duration)
        except KeyboardInterrupt:
            print("\n[已中断]")

    if sim.fallen:
        print(f"车翻啦！t = {sim.fall_time:.3f} s, 倾角 {sim.log['pitch'][-1]:+.2f} deg")

    stats = summarize(sim)
    print_report(sim, stats)
    if args.csv:
        write_csv(sim, args.csv)
    if args.svg:
        write_svg(sim, args.svg)
    return 0 if stats.get("balanced") else 1


if __name__ == "__main__":
    raise SystemExit(main())
