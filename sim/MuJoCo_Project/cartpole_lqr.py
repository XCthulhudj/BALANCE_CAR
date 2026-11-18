# -*- coding: utf-8 -*-
"""
MuJoCo 倒立摆（CartPole）—— LQR 自动扶正
=========================================
用法（在 MuJoCo_Project 目录、已激活 .venv 的情况下）：
    python cartpole_lqr.py              # 弹 3D 窗口，实时看它自己站稳
    python cartpole_lqr.py --headless   # 不弹窗，跑 10 秒并打印指标（远程桌面/无显卡时用）
    python cartpole_lqr.py --push       # 边站稳边被随机推一下，看抗扰能力
    python cartpole_lqr.py --snapshot   # 离屏渲染存一张 cartpole.png（需要 pillow）

依赖：mujoco（自带的），numpy（mujoco 依赖会自动装）。
"""
import argparse
import time

import numpy as np
import mujoco

# ---------------------------------------------------------------------------
# 1. 模型：小车（slide 关节）+ 摆杆（hinge 关节）+ 一个电机推小车
# ---------------------------------------------------------------------------
XML = """
<mujoco model="cartpole">
  <compiler angle="radian"/>
  <option timestep="0.002" gravity="0 0 -9.81" integrator="Euler"/>

  <visual>
    <global offwidth="800" offheight="600"/>
    <headlight diffuse="0.7 0.7 0.7" ambient="0.35 0.35 0.35" specular="0.1 0.1 0.1"/>
    <rgba haze="0.15 0.25 0.35 1"/>
  </visual>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
    <geom name="floor" type="plane" size="3 3 0.1" rgba="0.55 0.60 0.66 1" friction="1 0.005 0.0001"/>

    <camera name="side" pos="1.5 -1.5 0.9" xyaxes="0.7071 0.7071 0 -0.131 0.131 0.981"/>

    <body name="cart" pos="0 0 0.15">
      <joint name="slider" type="slide" axis="1 0 0" range="-1.5 1.5" damping="0.05"/>
      <geom name="cart_geom" type="box" size="0.14 0.09 0.07" density="400"
            rgba="0.20 0.45 0.78 1"/>

      <body name="pole" pos="0 0 0.07">
        <joint name="hinge" type="hinge" axis="0 1 0" range="-3.0 3.0" damping="0.002"/>
        <geom name="pole_geom" type="capsule" fromto="0 0 0 0 0 0.55" size="0.018"
              density="500" rgba="0.85 0.35 0.25 1"/>
        <body name="tip" pos="0 0 0.55">
          <geom name="tip_geom" type="sphere" size="0.05" density="700" rgba="0.95 0.65 0.15 1"/>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor name="push" joint="slider" gear="1" ctrlrange="-40 40"/>
  </actuator>

  <sensor>
    <jointpos joint="hinge"/>
    <jointvel joint="hinge"/>
  </sensor>
</mujoco>
"""


def build():
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    return model, data


def linearize(model, data):
    """在竖直平衡点做数值离散线性化，得到 A(4x4)、B(4x1)。"""
    data.qpos[:] = [0.0, 0.0]      # 小车在原点，摆杆竖直
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)

    nv = model.nv
    nu = model.nu
    A = np.zeros((2 * nv, 2 * nv))
    B = np.zeros((2 * nv, nu))
    mujoco.mjd_transitionFD(model, data, 1e-6, False, A, B, None, None)
    return A, B


def lqr(A, B, q_diag, r):
    """求解离散代数 Riccati 方程（迭代法），返回增益 K。"""
    n, m = B.shape
    Q = np.diag(q_diag)
    R = np.eye(m) * r
    P = Q.copy()
    for _ in range(5000):
        BtP = B.T @ P
        S = R + BtP @ B
        K = np.linalg.solve(S, BtP @ A)
        P_new = A.T @ P @ A - A.T @ P @ B @ K + Q
        if np.max(np.abs(P_new - P)) < 1e-10:
            P = P_new
            break
        P = P_new
    K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
    return K


def state_of(data):
    """状态向量 [小车位置, 摆角, 小车速度, 摆角速度]"""
    return np.concatenate([data.qpos[:2], data.qvel[:2]])


def save_png(path, rgb):
    """不依赖 pillow 的极简 PNG 写出（8bit RGB）。"""
    import struct
    import zlib
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw, 6))
           + chunk(b"IEND", b""))
    with open(path, "wb") as f:
        f.write(png)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true", help="不弹窗，只跑并打印指标")
    ap.add_argument("--push", action="store_true", help="运行中随机推一下")
    ap.add_argument("--snapshot", action="store_true", help="离屏渲染存 PNG")
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args()

    model, data = build()
    A, B = linearize(model, data)

    # Q 权重：位置轻、角度重、速度中等；R 越小越激进
    K = lqr(A, B, q_diag=[5.0, 200.0, 1.0, 10.0], r=1.0)
    print("LQR 增益 K =", np.round(K, 3))

    dt = model.opt.timestep
    n_steps = int(args.seconds / dt)

    # 初始：摆杆偏离竖直 12°，给一点点初速度
    data.qpos[0] = 0.0
    data.qpos[1] = np.deg2rad(12.0)
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    viewer = None
    renderer = None
    if args.snapshot:
        renderer = mujoco.Renderer(model, 600, 800)
    elif not args.headless:
        from mujoco import viewer as mj_viewer
        viewer = mj_viewer.launch_passive(model, data)

    rng = np.random.default_rng(0)
    push_step = n_steps // 2 if args.push else -1
    max_angle = 0.0
    t_start = time.perf_counter()

    for i in range(n_steps):
        if i == push_step:                      # 相当于被撞一下
            data.qvel[0] += 1.5

        u = -K[0] @ state_of(data)
        data.ctrl[0] = float(np.clip(u, -40.0, 40.0))
        mujoco.mj_step(model, data)

        max_angle = max(max_angle, abs(float(data.qpos[1])))

        if viewer is not None:
            viewer.sync()
            time.sleep(dt)                       # 实时播放
        if renderer is not None and i == n_steps - 3:
            cam = mujoco.MjvCamera()
            mujoco.mjv_defaultCamera(cam)
            cam.lookat[:] = [0.0, 0.0, 0.55]
            cam.distance = 2.0
            cam.azimuth = 135.0
            cam.elevation = -12.0
            renderer.update_scene(data, camera=cam)
            save_png("cartpole.png", renderer.render())
            print("已保存 cartpole.png")

    elapsed = time.perf_counter() - t_start
    if viewer is not None:
        viewer.close()

    print(f"仿真 {args.seconds:g}s / {n_steps} 步完成，墙钟耗时 {elapsed:.2f}s")
    print(f"最终摆角 {np.rad2deg(data.qpos[1]):+.3f}°，"
          f"小车位置 {data.qpos[0]:+.3f} m")
    print(f"过程中最大摆角 {np.rad2deg(max_angle):.2f}°"
          f"（初始 12°，能收敛到 1° 内就说明稳住了）")


if __name__ == "__main__":
    main()
