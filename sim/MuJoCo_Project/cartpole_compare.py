# -*- coding: utf-8 -*-
"""
倒立摆 对照组：一个不控制（倒），一个 LQR 控制（立）
=====================================================
同一场景里放两个完全相同的 cart-pole：
    灰色（左）：不给任何控制力矩 —— 摆杆必然倒，演示"不稳定"
    蓝色（右）：LQR 反馈 —— 摆杆自动扶正并保持竖直，演示"可稳定"

用法（MuJoCo_Project 目录、激活 .venv 后）：
    python cartpole_compare.py              # 3D 窗口同时看两个
    python cartpole_compare.py --headless   # 只打印两个的指标
    python cartpole_compare.py --snapshot   # 离屏存 compare.png
    python cartpole_compare.py --only stable   # 只留被控的那个
    python cartpole_compare.py --only unstable # 只留不管的那个
"""
import argparse
import time

import numpy as np
import mujoco


def build_xml(n_carts=2):
    """两个小车沿 y 方向并排；0 号=不控制（灰），1 号=被控（蓝）。"""
    bodies = []
    for i in range(n_carts):
        y = 0.7 if i == 0 else -0.7
        dim = (0.45, 0.47, 0.50) if i == 0 else (0.20, 0.45, 0.78)
        pole = (0.45, 0.45, 0.45) if i == 0 else (0.85, 0.35, 0.25)
        tip = (0.55, 0.55, 0.55) if i == 0 else (0.95, 0.65, 0.15)
        bodies.append(f"""
    <body name="cart{i}" pos="0 {y} 0.15">
      <joint name="slider{i}" type="slide" axis="1 0 0" range="-1.5 1.5" damping="0.05"/>
      <geom name="cart_geom{i}" type="box" size="0.14 0.09 0.07" density="400"
            rgba="{dim[0]} {dim[1]} {dim[2]} 1"/>
      <body name="pole{i}" pos="0 0 0.07">
        <joint name="hinge{i}" type="hinge" axis="0 1 0" range="-3.0 3.0" damping="0.002"/>
        <geom name="pole_geom{i}" type="capsule" fromto="0 0 0 0 0 0.55" size="0.018"
              density="500" rgba="{pole[0]} {pole[1]} {pole[2]} 1"/>
        <body name="tip{i}" pos="0 0 0.55">
          <geom name="tip_geom{i}" type="sphere" size="0.05" density="700"
                rgba="{tip[0]} {tip[1]} {tip[2]} 1"/>
        </body>
      </body>
    </body>""")

    actuators = "\n".join(
        f'    <motor name="push{i}" joint="slider{i}" gear="1" ctrlrange="-40 40"/>'
        for i in range(n_carts))

    return f"""
<mujoco model="cartpole_compare">
  <compiler angle="radian"/>
  <option timestep="0.002" gravity="0 0 -9.81" integrator="Euler"/>
  <visual>
    <global offwidth="1000" offheight="560"/>
    <headlight diffuse="0.7 0.7 0.7" ambient="0.35 0.35 0.35"/>
  </visual>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
    <geom name="floor" type="plane" size="4 4 0.1" rgba="0.55 0.60 0.66 1"
          friction="1 0.005 0.0001"/>
    {"".join(bodies)}
  </worldbody>
  <actuator>
{actuators}
  </actuator>
</mujoco>
"""


def lqr_gain(A, B, q_diag, r):
    Q = np.diag(q_diag)
    R = np.eye(B.shape[1]) * r
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


def gain_for_cart(ctrl_idx, n_carts=2):
    """对两车模型做线性化，取出第 ctrl_idx 个小车的 4x4 块，算 LQR 增益。"""
    m = mujoco.MjModel.from_xml_string(build_xml(n_carts))
    d = mujoco.MjData(m)
    d.qpos[:] = 0.0
    d.qvel[:] = 0.0
    d.ctrl[:] = 0.0
    mujoco.mj_forward(m, d)

    A = np.zeros((2 * m.nv, 2 * m.nv))
    B = np.zeros((2 * m.nv, m.nu))
    mujoco.mjd_transitionFD(m, d, 1e-6, False, A, B, None, None)

    base = 2 * ctrl_idx
    idx_q = [base, base + 1]
    idx_v = [2 * m.nv // 2 + base, 2 * m.nv // 2 + base + 1]
    rows = idx_q + idx_v
    A4 = A[np.ix_(rows, rows)]
    B4 = B[np.ix_(rows, [ctrl_idx])]
    return lqr_gain(A4, B4, q_diag=[5.0, 200.0, 1.0, 10.0], r=1.0), rows


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
    ap.add_argument("--only", choices=["stable", "unstable"], default=None,
                    help="只保留其中一个小车")
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--tilt", type=float, default=8.0, help="两个摆杆的初始偏角(度)")
    args = ap.parse_args()

    n_carts = 1 if args.only else 2
    m = mujoco.MjModel.from_xml_string(build_xml(n_carts))
    d = mujoco.MjData(m)

    # 约定：n_carts=2 时 0 号不受控、1 号受控；只留一个时它就是"受控/不受控"那一个
    nv2 = m.nv // n_carts          # 每个小车占用的 dof 数（2）
    ctrl_idx = 0 if n_carts == 1 else 1
    controlled = (args.only != "unstable")

    if controlled:
        K, rows = gain_for_cart(ctrl_idx, n_carts)
        print("受控小车 增益 K =", np.round(K, 3))
    else:
        K, rows = None, None

    for i in range(n_carts):
        d.qpos[2 * i + 1] = np.deg2rad(args.tilt)   # 每个小车的摆杆初偏
    d.qvel[:] = 0.0
    mujoco.mj_forward(m, d)

    dt = m.opt.timestep
    n = int(args.seconds / dt)

    viewer = None
    renderer = None
    if args.snapshot:
        renderer = mujoco.Renderer(m, 560, 1000)
    elif not args.headless:
        from mujoco import viewer as mjv
        viewer = mjv.launch_passive(m, d)

    t0 = time.perf_counter()
    for step in range(n):
        for i in range(n_carts):
            if controlled and i == ctrl_idx:
                x = np.array([d.qpos[2 * i], d.qpos[2 * i + 1],
                              d.qvel[2 * i], d.qvel[2 * i + 1]])
                d.ctrl[i] = float(np.clip(-K[0] @ x, -40.0, 40.0))
            else:
                d.ctrl[i] = 0.0            # 不受控：一点力都不给
        mujoco.mj_step(m, d)

        if viewer is not None:
            viewer.sync()
            time.sleep(dt)
        if renderer is not None and step == int(n * 0.55):
            cam = mujoco.MjvCamera()
            mujoco.mjv_defaultCamera(cam)
            cam.lookat[:] = [0.0, 0.0, 0.45]
            cam.distance = 3.0
            cam.azimuth = 90.0
            cam.elevation = -12.0
            renderer.update_scene(d, camera=cam)
            save_png("compare.png", renderer.render())
            print("已保存 compare.png")

    if viewer is not None:
        viewer.close()

    print(f"{args.seconds:g}s / {n} 步，墙钟 {time.perf_counter()-t0:.2f}s")
    for i in range(n_carts):
        tag = "受控" if (controlled and i == ctrl_idx) else "不受控"
        print(f"  小车{i}（{tag}）最终摆角 {np.rad2deg(d.qpos[2*i+1]):+8.2f}°  "
              f"小车位置 {d.qpos[2*i]:+.3f} m")


if __name__ == "__main__":
    main()
