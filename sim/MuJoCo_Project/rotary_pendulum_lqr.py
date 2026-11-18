# -*- coding: utf-8 -*-
"""
旋转倒立摆（Furuta 摆）—— 底座不动，靠水平臂原地旋转把摆杆扶正
================================================================
和 cart-pole 的区别：
    cart-pole  = 小车沿轨道平移来平衡（你看到"平移"的那版）
    Furuta 摆  = 底座固定，电机驱动一条水平臂绕竖直轴旋转，摆杆挂在臂端

用法（MuJoCo_Project 目录、激活 .venv 后）：
    python rotary_pendulum_lqr.py              # 3D 窗口，看臂转、杆立
    python rotary_pendulum_lqr.py --headless   # 不弹窗，打印指标
    python rotary_pendulum_lqr.py --snapshot   # 离屏存 rotary.png
    python rotary_pendulum_lqr.py --kick       # 中途踢一下，看抗扰
"""
import argparse
import time

import numpy as np
import mujoco

XML = """
<mujoco model="furuta">
  <compiler angle="radian"/>
  <option timestep="0.001" gravity="0 0 -9.81" integrator="Euler"/>

  <visual>
    <global offwidth="800" offheight="600"/>
    <headlight diffuse="0.7 0.7 0.7" ambient="0.35 0.35 0.35"/>
  </visual>

  <worldbody>
    <light pos="1 1 2" dir="-1 -1 -2" diffuse="0.8 0.8 0.8"/>
    <geom name="floor" type="plane" size="3 3 0.1" rgba="0.55 0.60 0.66 1"/>

    <body name="base" pos="0 0 0.10">
      <geom name="base_geom" type="cylinder" size="0.07 0.06" density="900"
            contype="0" conaffinity="0" rgba="0.30 0.32 0.36 1"/>

      <body name="arm" pos="0 0 0.06">
        <joint name="arm" type="hinge" axis="0 0 1" damping="0.02"/>
        <geom name="arm_geom" type="capsule" fromto="0 0 0 0.5 0 0" size="0.020"
              density="700" rgba="0.20 0.45 0.78 1"/>

        <body name="pole" pos="0.5 0 0">
          <joint name="pole" type="hinge" axis="1 0 0" damping="0.002"/>
          <geom name="pole_geom" type="capsule" fromto="0 0 0 0 0 0.42" size="0.015"
                density="600" rgba="0.85 0.35 0.25 1"/>
          <geom name="tip_geom" type="sphere" pos="0 0 0.42" size="0.035"
                density="900" rgba="0.95 0.65 0.15 1"/>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor name="motor" joint="arm" gear="1" ctrlrange="-3 3"/>
  </actuator>
</mujoco>
"""


def build():
    m = mujoco.MjModel.from_xml_string(XML)
    d = mujoco.MjData(m)
    return m, d


def lqr_gain(m, d, q_diag, r):
    """竖直平衡点线性化 + 迭代解 DARE。"""
    d.qpos[:] = 0.0
    d.qvel[:] = 0.0
    d.ctrl[:] = 0.0
    mujoco.mj_forward(m, d)

    nv, nu = m.nv, m.nu
    A = np.zeros((2 * nv, 2 * nv))
    B = np.zeros((2 * nv, nu))
    mujoco.mjd_transitionFD(m, d, 1e-6, False, A, B, None, None)

    Q = np.diag(q_diag)
    R = np.eye(nu) * r
    P = Q.copy()
    for _ in range(20000):
        BtP = B.T @ P
        K = np.linalg.solve(R + BtP @ B, BtP @ A)
        P_new = A.T @ P @ A - A.T @ P @ B @ K + Q
        if np.max(np.abs(P_new - P)) < 1e-12:
            P = P_new
            break
        P = P_new
    return np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)


def save_png(path, rgb):
    import struct
    import zlib
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n"
                + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 6))
                + chunk(b"IEND", b""))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--snapshot", action="store_true")
    ap.add_argument("--kick", action="store_true")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--tilt", type=float, default=20.0, help="初始摆杆偏角(度)")
    args = ap.parse_args()

    m, d = build()
    # 臂角度权重给小一点（臂可以随便转），摆杆角度权重大
    K = lqr_gain(m, d, q_diag=[1.0, 400.0, 0.5, 8.0], r=0.1)
    print("K =", np.round(K, 3))

    d.qpos[0] = 0.0
    d.qpos[1] = np.deg2rad(args.tilt)
    d.qvel[:] = 0.0
    mujoco.mj_forward(m, d)

    dt = m.opt.timestep
    n = int(args.seconds / dt)

    viewer = None
    renderer = None
    if args.snapshot:
        renderer = mujoco.Renderer(m, 600, 800)
    elif not args.headless:
        from mujoco import viewer as mjv
        viewer = mjv.launch_passive(m, d)

    max_tilt = 0.0
    t0 = time.perf_counter()
    for i in range(n):
        if args.kick and i == n // 2:
            d.qvel[1] += 2.0            # 相当于狠踢一脚摆杆

        x = np.concatenate([d.qpos[:2], d.qvel[:2]])
        d.ctrl[0] = float(np.clip(-K[0] @ x, -3.0, 3.0))
        mujoco.mj_step(m, d)
        max_tilt = max(max_tilt, abs(float(d.qpos[1])))

        if viewer is not None:
            viewer.sync()
            time.sleep(dt)
        if renderer is not None and i == int(n * 0.45):
            cam = mujoco.MjvCamera()
            mujoco.mjv_defaultCamera(cam)
            cam.lookat[:] = [0.15, 0.0, 0.32]
            cam.distance = 1.6
            cam.azimuth = 125.0
            cam.elevation = -20.0
            renderer.update_scene(d, camera=cam)
            save_png("rotary.png", renderer.render())
            print("已保存 rotary.png")

    if viewer is not None:
        viewer.close()

    print(f"{args.seconds:g}s / {n} 步，墙钟 {time.perf_counter()-t0:.2f}s")
    print(f"最终摆角 {np.rad2deg(d.qpos[1]):+.3f}°，臂累计转过 {np.rad2deg(d.qpos[0]):+.1f}°")
    print(f"最大摆角 {np.rad2deg(max_tilt):.2f}°（初始 {args.tilt:g}°）")


if __name__ == "__main__":
    main()
