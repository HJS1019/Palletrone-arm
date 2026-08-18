#!/usr/bin/env python3
"""
Palletrone + 로봇팔 : 명령어로 조종하는 대화형 시뮬레이터

  터미널에 명령을 치면 그대로 반영된다. 시간 제한 없이 계속 떠 있는다.

명령어
  takeoff / t        이륙 -> 호버 (목표 고도까지 램프 상승)
  dob on | off       외란 관측기 켜고 끄기 (켤 때 관측기 상태 리셋)
  dist X Y Z         외란 인가 [N], world frame.  예:  dist 15 0 0
  dist off           외란 해제
  moment X Y Z       외란 모멘트 인가 [Nm], body frame
  goto X Y Z         목표 위치 이동 [m]
  status / s         현재 상태 한 줄 출력
  log <파일>          지금부터 CSV 기록 시작 (없으면 자동 이름)
  land / l           착륙 -> 접지 후 시뮬레이션 종료
  quit / q           즉시 종료

뷰어 단축키 (뷰어 창에 포커스가 있을 때)
  T 이륙   D DoB 토글   X +15N 외란 토글   L 착륙   Q 종료

실행
  python3 fly_sim.py --xml src/plant/xml/Palletrone.xml --a -0.15
"""
import argparse
import math
import os
import queue
import re
import sys
import threading
import time

import numpy as np
import mujoco

# ----------------------------------------------------------------- 상수
PHYS_HZ = 400.0
G = 9.81
K_THRUST = 0.02
ZETA = 0.02
R_Z = 0.075
INV_SQRT2 = 1.0 / math.sqrt(2.0)
MAX_T = 50.0
MAX_W = math.sqrt(MAX_T / K_THRUST)
SERVO_LIM = math.radians(90.0)
GROUND_Z = 0.115

KP_POS = np.array([28.0, 28.0, 20.0])
KI_POS = np.array([1.5, 1.5, 1.1])
KD_POS = np.array([6.0, 6.0, 10.0])
IP_MIN, IP_MAX, OP_MIN, OP_MAX = -5.0, 10.0, -200.0, 200.0

KP_ATT = np.array([2.0, 2.0, 2.0])
KI_ATT = np.array([0.3, 0.3, 0.3])
KD_ATT = np.array([1.30, 1.30, 1.30])
IA_MIN, IA_MAX, OA_MIN, OA_MAX = -1.0, 1.0, -5.0, 5.0

PSI = np.radians([45.0, 135.0, -135.0, -45.0])
SIGN = np.array([1.0, -1.0, 1.0, -1.0])


# ----------------------------------------------------------------- 유틸
def quat_to_rpy(q):
    w, x, y, z = q
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    return np.array([math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
                     math.asin(s),
                     math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))])


def R_wb(rpy):
    r, p, y = rpy
    sr, cr, sp, cp, sy, cy = (math.sin(r), math.cos(r), math.sin(p),
                              math.cos(p), math.sin(y), math.cos(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


class PID:
    def __init__(self, kp, ki, kd, imin, imax, omin, omax):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.imin, self.imax, self.omin, self.omax = imin, imax, omin, omax
        self.iacc = np.zeros(3)

    def __call__(self, ref, cur, dcur, dt):
        e = ref - cur
        self.iacc = np.clip(self.iacc + self.ki * e * dt, self.imin, self.imax)
        return np.clip(self.kp * e + self.iacc - self.kd * dcur, self.omin, self.omax)

    def reset(self):
        self.iacc[:] = 0.0


# ----------------------------------------------------------------- 할당기
class Allocator:
    def __init__(self, r, fix_mz=False):
        self.R = r
        self.L = math.sqrt(2.0) * r
        self.Pc = np.zeros(3)
        self.fix_mz = fix_mz      # calc_A1 의 Mz 행에 틸트 유발 항(-L sin) 추가

    def A1(self, C2):
        s, c = np.sin(C2), np.cos(C2)
        pcx, pcy, pcz = self.Pc
        h = self.L / math.sqrt(2.0)
        A = np.zeros((4, 4))
        A[0] = [INV_SQRT2 * (ZETA + R_Z - pcz) * s[0] + (h - pcy) * c[0],
                INV_SQRT2 * (-ZETA - R_Z + pcz) * s[1] + (h - pcy) * c[1],
                INV_SQRT2 * (-ZETA - R_Z + pcz) * s[2] + (-h - pcy) * c[2],
                INV_SQRT2 * (ZETA + R_Z - pcz) * s[3] + (-h - pcy) * c[3]]
        A[1] = [INV_SQRT2 * (-ZETA + R_Z - pcz) * s[0] + (-h + pcx) * c[0],
                INV_SQRT2 * (-ZETA + R_Z - pcz) * s[1] + (h + pcx) * c[1],
                INV_SQRT2 * (ZETA - R_Z + pcz) * s[2] + (h + pcx) * c[2],
                INV_SQRT2 * (ZETA - R_Z + pcz) * s[3] + (-h + pcx) * c[3]]
        if self.fix_mz:
            # M_z^(i) = sigma_i * zeta * T_i * cos(th) - L * T_i * sin(th)
            A[2] = [ZETA * c[0] - self.L * s[0], -ZETA * c[1] - self.L * s[1],
                    ZETA * c[2] - self.L * s[2], -ZETA * c[3] - self.L * s[3]]
        else:
            A[2] = [ZETA * c[0], -ZETA * c[1], ZETA * c[2], -ZETA * c[3]]
        A[3] = c
        return A

    def A2(self, C1, C2):
        s = np.sin(C2)
        pcx, pcy = self.Pc[0], self.Pc[1]
        f, L = C1, self.L
        A = np.zeros((4, 4))
        A[0] = [INV_SQRT2 * f[0], INV_SQRT2 * f[1], -INV_SQRT2 * f[2], -INV_SQRT2 * f[3]]
        A[1] = [-INV_SQRT2 * f[0], INV_SQRT2 * f[1], INV_SQRT2 * f[2], -INV_SQRT2 * f[3]]
        A[2] = [INV_SQRT2 * (pcx + pcy) * s[0] - L * f[0],
                INV_SQRT2 * (-pcx + pcy) * s[1] - L * f[1],
                INV_SQRT2 * (-pcx - pcy) * s[2] - L * f[2],
                INV_SQRT2 * (pcx - pcy) * s[3] - L * f[3]]
        A[3] = [-2 * L * f[0], 2 * L * f[1], -2 * L * f[2], 2 * L * f[3]]
        return A


def solve4(A, b):
    try:
        return np.linalg.solve(A, b)
    except np.linalg.LinAlgError:
        return np.linalg.lstsq(A, b, rcond=None)[0]


# ----------------------------------------------------------------- 관측기
class MomentumObserver:
    """1차 운동량 기반 외란 관측기 (first_wrench_observer.cpp 와 동일 형태)."""

    def __init__(self, mass, inertia, kf=5.0, km=10.0):
        self.m, self.I = mass, inertia
        self.kf, self.km = kf, km
        self.armed = False
        self.zf = np.zeros(3)
        self.zt = np.zeros(3)
        self.F_hat = np.zeros(3)
        self.M_hat = np.zeros(3)

    def reset(self):
        self.armed = False
        self.F_hat[:] = 0.0
        self.M_hat[:] = 0.0

    def update(self, v_com_w, omega_b, F_act_b, M_act_b, Rwb, dt):
        p = self.m * v_com_w
        h = self.I @ omega_b
        if not self.armed:
            self.zf, self.zt = p.copy(), h.copy()
            self.F_hat[:] = 0.0
            self.M_hat[:] = 0.0
            self.armed = True
            return self.F_hat, self.M_hat
        self.F_hat = self.kf * (p - self.zf)
        self.M_hat = self.km * (h - self.zt)
        u_f = Rwb @ F_act_b + np.array([0.0, 0.0, -self.m * G])
        u_t = M_act_b - np.cross(omega_b, h)
        self.zf += dt * (u_f + self.F_hat)
        self.zt += dt * (u_t + self.M_hat)
        return self.F_hat, self.M_hat


# ----------------------------------------------------------------- XML
def build_xml(src, a, m_arm, lc, base_mass, r_new, add_arm=True,
              cw_mass=0.0, cw_pos=-0.251):
    s = open(os.path.abspath(src), encoding="utf-8").read()
    s = re.sub(r'(<body name="base"\s+pos="[^"]*?)\s[\d.eE+-]+">',
               lambda m: f'{m.group(1)} {GROUND_Z}">', s, count=1)
    s = s.replace('mass="4.00"', f'mass="{base_mass:.4f}"', 1)
    s = s.replace("0.148492", f"{r_new:.6f}")
    if cw_mass and cw_mass > 0:
        cw = (f'\n      <body name="cw" pos="{cw_pos:.6f} 0 0">\n'
              f'        <inertial pos="0 0 0" mass="{cw_mass:.4f}" '
              f'diaginertia="0.001 0.001 0.001"/>\n'
              f'        <geom name="cw_vis" type="box" size="0.089 0.05 0.03" '
              f'rgba="0.9 0.5 0.1 1" contype="0" conaffinity="0"/>\n      </body>\n')
        i = s.rfind("    </body>\n  </worldbody>")
        s = s[:i] + cw + s[i:]
    if add_arm:
        arm = (f'\n      <body name="arm" pos="{a + lc:.6f} 0 0">\n'
               f'        <inertial pos="0 0 0" mass="{m_arm:.4f}" '
               f'diaginertia="{m_arm*0.0025:.6f} {m_arm*lc**2/12:.6f} {m_arm*lc**2/12:.6f}"/>\n'
               f'        <geom name="arm_vis" type="box" size="0.05 0.04 0.04" '
               f'rgba="0.1 0.6 1 1" contype="0" conaffinity="0"/>\n      </body>\n')
        i = s.rfind("    </body>\n  </worldbody>")
        s = s[:i] + arm + s[i:]
    if "<light" not in s:
        scene = '''
  <visual>
    <headlight diffuse="0.85 0.85 0.85" ambient="0.45 0.45 0.45" specular="0.1 0.1 0.1"/>
    <global azimuth="130" elevation="-20"/>
    <map znear="0.02"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.60 0.72 0.88" rgb2="0.92 0.95 0.99"
             width="512" height="3072"/>
    <texture type="2d" name="_grid" builtin="checker" mark="edge" rgb1="0.80 0.80 0.82"
             rgb2="0.65 0.65 0.68" markrgb="0.95 0.95 0.95" width="300" height="300"/>
    <material name="_grid" texture="_grid" texuniform="true" texrepeat="4 4" reflectance="0.05"/>
  </asset>
  <worldbody>
    <light pos="2 2 6" dir="-0.3 -0.3 -1" directional="true" diffuse="0.7 0.7 0.7"/>
    <light pos="-3 -2 4" dir="0.5 0.4 -1" directional="true" diffuse="0.35 0.35 0.4"/>
    <geom name="_floor" size="0 0 0.05" type="plane" material="_grid"/>
  </worldbody>
'''
        s = s.replace("<compiler", scene + "\n  <compiler", 1)
        s = s.replace('mesh="body_mesh" rgba="0.1 0.1 0.1 1"',
                      'mesh="body_mesh" rgba="0.35 0.38 0.45 1"')
    out = os.path.join(os.path.dirname(os.path.abspath(src)), "_fly_sim_generated.xml")
    open(out, "w", encoding="utf-8").write(s)
    return out


# ----------------------------------------------------------------- 본체
def composite_inertia(model, data, bid, com_b):
    """base 서브트리 전체의 관성텐서를 base body frame, 합성 CoM 기준으로."""
    Rb = np.array(data.xmat[bid]).reshape(3, 3)
    base_pos = np.array(data.xpos[bid])
    com_w = base_pos + Rb @ com_b
    I = np.zeros((3, 3))
    # base 서브트리에 속한 body 만
    def in_subtree(b):
        while b != -1:
            if b == bid:
                return True
            b = model.body_parentid[b]
        return False
    for i in range(model.nbody):
        if model.body_mass[i] <= 0 or not in_subtree(i):
            continue
        Ri = np.array(data.ximat[i]).reshape(3, 3)          # 주축 -> world
        Ii = Ri @ np.diag(model.body_inertia[i]) @ Ri.T     # world frame
        d = np.array(data.xipos[i]) - com_w                 # CoM 에서의 변위
        I += Ii + model.body_mass[i] * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return Rb.T @ I @ Rb                                    # body frame 으로


def lowest_point_offset(model, data, bid):
    """현재 자세에서 가장 낮은 점이 지면에 닿도록 하는 base 원점 높이.
    바운딩 구(rbound) 대신 지오메트리 종류별 실제 최저점을 쓴다."""
    G = mujoco.mjtGeom
    base_z = float(data.xpos[bid][2])
    lo = base_z
    for g in range(model.ngeom):
        t = model.geom_type[g]
        if t == G.mjGEOM_PLANE:
            continue
        p = np.array(data.geom_xpos[g])
        R = np.array(data.geom_xmat[g]).reshape(3, 3)
        sz = model.geom_size[g]
        if t == G.mjGEOM_SPHERE:
            z = p[2] - sz[0]
        elif t == G.mjGEOM_CAPSULE or t == G.mjGEOM_CYLINDER:
            half = R[:, 2] * sz[1]
            z = min(p[2] - half[2], p[2] + half[2]) - sz[0]
        elif t == G.mjGEOM_BOX:
            ext = np.abs(R) @ sz[:3]
            z = p[2] - ext[2]
        elif t == G.mjGEOM_MESH:
            did = model.geom_dataid[g]
            a = model.mesh_vertadr[did]
            n = model.mesh_vertnum[did]
            v = model.mesh_vert[a:a + n].reshape(-1, 3)
            z = float((p + v @ R.T)[:, 2].min())
        else:
            z = p[2] - model.geom_rbound[g]
        lo = min(lo, z)
    return base_z - lo + 0.005            # 5 mm 여유


class Sim:
    def __init__(self, args):
        self.args = args
        m_arm = 0.0 if args.no_arm else args.mass
        self.m_total = args.base_mass + m_arm + args.cw_mass
        self.x_c = (m_arm * (args.a + args.lc)
                    + args.cw_mass * args.cw_pos) / self.m_total
        self.R = args.arm_len / math.sqrt(2.0)

        if args.model:
            xml = os.path.abspath(args.model)
        else:
            xml = build_xml(args.xml, args.a, args.mass, args.lc,
                            args.base_mass, self.R, add_arm=not args.no_arm,
                            cw_mass=args.cw_mass, cw_pos=args.cw_pos)
        self.model = mujoco.MjModel.from_xml_path(xml)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = 1.0 / PHYS_HZ
        self.bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, b"base")

        sid = lambda n: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, n.encode())
        self.iq, self.ig = sid("body_quat"), sid("body_gyro")
        self.ip, self.iv = sid("base_pos"), sid("base_linvel")
        self.isv = [sid(f"servo{i}_angle") for i in range(1, 5)]
        self.adr, self.dim = self.model.sensor_adr, self.model.sensor_dim
        self.lo = np.array(self.model.actuator_ctrlrange[:, 0])
        self.hi = np.array(self.model.actuator_ctrlrange[:, 1])
        # 팔 관절이 있으면 keyframe 자세로 초기화하고 그 값을 계속 유지
        self.n_drone_act = 8
        self.arm_ctrl = None
        if self.model.nkey > 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        if self.model.nu > self.n_drone_act:
            self.arm_ctrl = np.array(self.model.key_ctrl[0][self.n_drone_act:]) \
                if self.model.nkey > 0 else np.zeros(self.model.nu - self.n_drone_act)
        mujoco.mj_forward(self.model, self.data)

        # --- 실제 모델 제원 (팔/변압기 포함) ---
        self.m_total = float(self.model.body_subtreemass[self.bid])
        com_w = np.array(self.data.subtree_com[self.bid]) - np.array(self.data.xpos[self.bid])
        Rb = np.array(self.data.xmat[self.bid]).reshape(3, 3)
        self.com_b = Rb.T @ com_w
        self.x_c = float(self.com_b[0])
        self.I_com = composite_inertia(self.model, self.data, self.bid, self.com_b)
        self.ground_z = lowest_point_offset(self.model, self.data, self.bid)

        self.data.qpos[2] = self.ground_z
        mujoco.mj_forward(self.model, self.data)

        self.pid_p = PID(KP_POS, KI_POS, KD_POS, IP_MIN, IP_MAX, OP_MIN, OP_MAX)
        self.pid_a = PID(KP_ATT, KI_ATT, KD_ATT, IA_MIN, IA_MAX, OA_MIN, OA_MAX)
        self.alloc = Allocator(self.R, fix_mz=args.fix_mz)
        # NOTE: allocator 의 r_z 상수가 이미 프로펠러 높이를 담고 있어
        #       CoM 의 z 성분까지 Pc_ 에 넣으면 모멘트 팔이 이중 계산되어 발산한다.
        #       x (필요시 y) 성분만 사용한다.
        self.alloc.Pc = np.array([self.com_b[0], self.com_b[1], 0.0])

        self.obs = MomentumObserver(self.m_total, self.I_com, args.kf, args.km)

        # 상태
        self.mode = "GROUND"
        self.dob_on = False
        self.dist_F = np.zeros(3)
        self.dist_M = np.zeros(3)
        self.target = np.array([0.0, 0.0, self.ground_z])
        self.ramp_from = self.ground_z
        self.ramp_to = self.ground_z
        self.ramp_t0 = 0.0
        self.T_cmd = np.zeros(4)
        self.th_cmd = np.zeros(4)
        self.armed = False
        self.log_rows = []
        self.log_path = None
        self.quit = False

        print(f"\n기체  총질량 {self.m_total:.3f} kg   (모델에서 산출)")
        print(f"      암 길이 {args.arm_len*100:.1f} cm (r = {self.R:.5f} m)")
        print(f"      합성 CoM = [{self.com_b[0]:+.4f} {self.com_b[1]:+.4f} {self.com_b[2]:+.4f}] m"
              f"   x_c = {self.x_c/self.R:.0%} of limit")
        print(f"      관성 diag = [{self.I_com[0,0]:.4f} {self.I_com[1,1]:.4f} {self.I_com[2,2]:.4f}]")
        print(f"      지상 시작 고도 {self.ground_z:.3f} m,  액추에이터 {self.model.nu} 개")
        print(f"      호버 추력 모터당 {self.m_total*G/4:.2f} N\n")

    # ---------------- 센서
    def get(self, i):
        return np.array(self.data.sensordata[self.adr[i]:self.adr[i] + self.dim[i]])

    # ---------------- 명령
    def cmd(self, line):
        p = line.strip().split()
        if not p:
            return
        c = p[0].lower()
        try:
            if c in ("takeoff", "t"):
                if self.mode in ("GROUND", "LAND"):
                    self.armed = True
                    self.pid_p.reset()
                    self.pid_a.reset()
                    self.mode = "TAKEOFF"
                    self.ramp_from = self.data.sensordata[self.adr[self.ip] + 2]
                    self.ramp_to = self.args.alt
                    self.ramp_t0 = self.data.time
                    print(f"[{self.data.time:7.2f}] 이륙 -> {self.args.alt} m")
                else:
                    print("  이미 비행 중")
            elif c == "dob":
                on = (len(p) < 2) or p[1].lower() in ("on", "1", "true")
                if on and self.mode != "HOVER":
                    print("  호버 상태에서만 켤 수 있음 (지상 접촉이 외란으로 잡힘)")
                else:
                    self.dob_on = on
                    self.obs.reset()
                    print(f"[{self.data.time:7.2f}] DoB {'ON' if on else 'off'}")
            elif c == "dist":
                if len(p) >= 2 and p[1].lower() in ("off", "0", "none"):
                    self.dist_F[:] = 0.0
                    print(f"[{self.data.time:7.2f}] 외란 해제")
                else:
                    v = [float(x) for x in p[1:4]]
                    self.dist_F[:] = v + [0.0] * (3 - len(v))
                    print(f"[{self.data.time:7.2f}] 외란 F = {self.dist_F} N")
            elif c == "moment":
                if len(p) >= 2 and p[1].lower() in ("off", "0", "none"):
                    self.dist_M[:] = 0.0
                    print(f"[{self.data.time:7.2f}] 외란 모멘트 해제")
                else:
                    v = [float(x) for x in p[1:4]]
                    self.dist_M[:] = v + [0.0] * (3 - len(v))
                    print(f"[{self.data.time:7.2f}] 외란 M = {self.dist_M} Nm")
            elif c == "goto":
                v = [float(x) for x in p[1:4]]
                v = v + [0.0] * (3 - len(v))
                self.target[0], self.target[1] = v[0], v[1]
                if len(p) >= 4:
                    self.args.alt = v[2]
                    self.ramp_from = self.ramp_to = v[2]
                print(f"[{self.data.time:7.2f}] 목표 {self.target}")
            elif c in ("land", "l"):
                if self.mode in ("HOVER", "TAKEOFF"):
                    self.mode = "LAND"
                    self.dob_on = False
                    self.obs.reset()
                    self.dist_F[:] = 0.0
                    self.dist_M[:] = 0.0
                    self.ramp_from = self.data.sensordata[self.adr[self.ip] + 2]
                    self.ramp_to = self.ground_z - 0.02
                    self.ramp_t0 = self.data.time
                    print(f"[{self.data.time:7.2f}] 착륙 시작")
                else:
                    print("  비행 중이 아님")
            elif c in ("status", "s"):
                self.status()
            elif c == "log":
                self.log_path = p[1] if len(p) > 1 else f"flight_{int(time.time())}.csv"
                self.log_rows = []
                print(f"[{self.data.time:7.2f}] 기록 시작 -> {self.log_path}")
            elif c in ("quit", "q", "exit"):
                self.quit = True
            elif c in ("help", "h", "?"):
                print(__doc__)
            else:
                print(f"  알 수 없는 명령: {c}   (help 입력)")
        except (ValueError, IndexError):
            print(f"  인자 오류: {line.strip()}   예) dist 15 0 0")

    def status(self):
        pos = self.get(self.ip)
        rpy = np.degrees(quat_to_rpy(self.get(self.iq)))
        F = self.data.actuator_force[0:4]
        print(f"[{self.data.time:7.2f}] {self.mode:8s} DoB={'ON ' if self.dob_on else 'off'} "
              f"pos=[{pos[0]:+.3f} {pos[1]:+.3f} {pos[2]:.3f}] "
              f"rpy=[{rpy[0]:+.1f} {rpy[1]:+.1f} {rpy[2]:+.1f}] "
              f"F=[{F[0]:5.2f} {F[1]:5.2f} {F[2]:5.2f} {F[3]:5.2f}] "
              f"dist={self.dist_F} F_hat=[{self.obs.F_hat[0]:+.2f} "
              f"{self.obs.F_hat[1]:+.2f} {self.obs.F_hat[2]:+.2f}]")

    # ---------------- 한 스텝
    def step(self):
        dt = 1.0 / PHYS_HZ
        t = self.data.time
        rpy = quat_to_rpy(self.get(self.iq))
        pos, vel, gyro = self.get(self.ip), self.get(self.iv), self.get(self.ig)
        srv = np.array([self.get(i)[0] for i in self.isv])
        Rwb = R_wb(rpy)

        # 고도 램프
        if self.mode in ("TAKEOFF", "LAND"):
            f = min(1.0, (t - self.ramp_t0) / self.args.ramp)
            z_ref = self.ramp_from + f * (self.ramp_to - self.ramp_from)
            if self.mode == "TAKEOFF" and f >= 1.0:
                self.mode = "HOVER"
                print(f"[{t:7.2f}] 호버 진입 (고도 {self.args.alt} m)")
            if self.mode == "LAND" and pos[2] < self.ground_z + 0.03 and abs(vel[2]) < 0.15:
                self.armed = False
                self.mode = "DONE"
                print(f"[{t:7.2f}] 접지. 시뮬레이션 종료")
        else:
            z_ref = self.args.alt if self.mode == "HOVER" else self.ground_z
        ref = np.array([self.target[0], self.target[1], z_ref])

        # 관측기
        F_act_b, M_act_b = self.applied_wrench()
        v_com = vel + Rwb @ np.cross(gyro, self.com_b)
        if self.dob_on:
            F_hat, M_hat = self.obs.update(v_com, gyro, F_act_b, M_act_b, Rwb, dt)
        else:
            F_hat, M_hat = np.zeros(3), np.zeros(3)

        # 제어
        if self.armed:
            u = self.pid_p(ref, pos, vel, dt)
            F_w = np.array([u[0], u[1], u[2] + self.m_total * G]) - F_hat
            F_b = Rwb.T @ F_w
            F_b[2] = max(0.0, F_b[2])
            M_b = self.pid_a(np.zeros(3), rpy, gyro, dt) - M_hat
            B1 = np.array([M_b[0], M_b[1], 0.0, F_b[2]])
            B2 = np.array([F_b[0], F_b[1], M_b[2], 0.0])
            C1r = solve4(self.alloc.A1(srv), B1)
            Sd = solve4(self.alloc.A2(C1r, srv), B2)
            C2 = np.clip(np.arcsin(np.clip(Sd, -1.0, 1.0)), -SERVO_LIM, SERVO_LIM)
            C1 = np.minimum(solve4(self.alloc.A1(C2), B1), MAX_T)
            w = np.clip(np.sqrt(np.maximum(0.0, C1 / K_THRUST)), 0.0, MAX_W)
            self.T_cmd, self.th_cmd = K_THRUST * w ** 2, C2
        else:
            self.T_cmd, self.th_cmd = np.zeros(4), np.zeros(4)

        # 외란
        self.data.xfrc_applied[:] = 0.0
        self.data.xfrc_applied[self.bid, 0:3] = self.dist_F
        self.data.xfrc_applied[self.bid, 3:6] = Rwb @ self.dist_M

        n = self.n_drone_act
        cmd = np.clip(np.concatenate([self.T_cmd, self.th_cmd]), self.lo[:n], self.hi[:n])
        self.data.ctrl[:n] = cmd
        if self.arm_ctrl is not None:
            self.data.ctrl[n:] = self.arm_ctrl
        mujoco.mj_step(self.model, self.data)

        if self.log_path is not None:
            self.log_rows.append([t, *pos, *np.degrees(rpy),
                                  *self.data.actuator_force[0:4], *np.degrees(srv),
                                  *F_hat, *M_hat, *self.dist_F,
                                  1.0 if self.dob_on else 0.0])

    def applied_wrench(self):
        F, M = np.zeros(3), np.zeros(3)
        for i in range(4):
            c, s = math.cos(PSI[i]), math.sin(PSI[i])
            Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            ct, st = math.cos(self.th_cmd[i]), math.sin(self.th_cmd[i])
            Rx = np.array([[1, 0, 0], [0, ct, -st], [0, st, ct]])
            Rm = Rz @ Rx
            f = Rm @ np.array([0.0, 0.0, self.T_cmd[i]])
            tq = Rm @ np.array([0.0, 0.0, SIGN[i] * ZETA * self.T_cmd[i]])
            p = np.array([self.R * np.sign(c), self.R * np.sign(s), 0.070]) - self.com_b
            F += f
            M += np.cross(p, f) + tq
        return F, M

    def save_log(self):
        if self.log_path and self.log_rows:
            hdr = ("t,x,y,z,roll_deg,pitch_deg,yaw_deg,F1,F2,F3,F4,"
                   "servo1,servo2,servo3,servo4,"
                   "Fhat_x,Fhat_y,Fhat_z,Mhat_x,Mhat_y,Mhat_z,"
                   "dist_x,dist_y,dist_z,dob_on")
            np.savetxt(self.log_path, np.array(self.log_rows), delimiter=",",
                       header=hdr, comments="", fmt="%.6f")
            print(f"[저장] {self.log_path}  ({len(self.log_rows)} rows)")


# ----------------------------------------------------------------- 입력 스레드
def stdin_thread(q):
    for line in sys.stdin:
        q.put(line)
    q.put("quit")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--xml", default=None, help="Palletrone.xml (모델을 즉석 생성할 때)")
    p.add_argument("--model", default=None,
                   help="이미 만들어둔 XML 경로 (build_arm_drone.py 결과 등)")
    p.add_argument("--a", type=float, default=-0.15, help="장착점 x 오프셋 [m]")
    p.add_argument("--mass", type=float, default=2.0, help="팔 질량 [kg]")
    p.add_argument("--lc", type=float, default=0.42, help="장착점->팔 CoM [m]")
    p.add_argument("--base-mass", type=float, default=4.0)
    p.add_argument("--arm-len", type=float, default=0.23, help="원점->프로펠러 암 길이 [m]")
    p.add_argument("--no-arm", action="store_true")
    p.add_argument("--cw-mass", type=float, default=0.0, help="카운터웨이트(변압기) 질량 [kg]")
    p.add_argument("--cw-pos", type=float, default=-0.251, help="카운터웨이트 x 위치 [m]")
    p.add_argument("--alt", type=float, default=1.0, help="호버 고도 [m]")
    p.add_argument("--ramp", type=float, default=4.0, help="이착륙 램프 시간 [s]")
    p.add_argument("--kf", type=float, default=5.0, help="DoB 힘 게인")
    p.add_argument("--km", type=float, default=10.0, help="DoB 모멘트 게인")
    p.add_argument("--fix-mz", action="store_true",
                   help="allocator calc_A1 의 Mz 행에 틸트 유발 요모멘트 항 추가")
    p.add_argument("--no-viewer", action="store_true")
    p.add_argument("--script", type=str, default=None,
                   help="'8:takeoff;20:dob on;30:dist 15 0 0;50:land' 형태로 자동 실행")
    args = p.parse_args()

    sim = Sim(args)

    sched = []
    if args.script:
        for item in args.script.split(";"):
            tt, cc = item.split(":", 1)
            sched.append((float(tt), cc.strip()))
        sched.sort()

    q = queue.Queue()
    if not args.script:
        threading.Thread(target=stdin_thread, args=(q,), daemon=True).start()
        print("명령 입력 준비됨.  takeoff / dob on / dist 15 0 0 / land / quit  (help 로 전체 목록)\n")

    def pump():
        while not q.empty():
            sim.cmd(q.get())
        while sched and sim.data.time >= sched[0][0]:
            sim.cmd(sched.pop(0)[1])

    def key_cb(k):
        m = {ord('T'): "takeoff", ord('L'): "land", ord('Q'): "quit",
             ord('D'): f"dob {'off' if sim.dob_on else 'on'}",
             ord('X'): "dist off" if abs(sim.dist_F[0]) > 1e-6 else "dist 15 0 0"}
        if k in m:
            sim.cmd(m[k])

    try:
        if args.no_viewer:
            while not sim.quit and sim.mode != "DONE":
                pump()
                sim.step()
        else:
            from mujoco import viewer as mjv
            with mjv.launch_passive(sim.model, sim.data, key_callback=key_cb) as v:
                t0 = time.perf_counter()
                n = 0
                while v.is_running() and not sim.quit and sim.mode != "DONE":
                    pump()
                    sim.step()
                    n += 1
                    if n % 8 == 0:
                        v.sync()
                    lag = sim.data.time - (time.perf_counter() - t0)
                    if lag > 0:
                        time.sleep(lag)
    except KeyboardInterrupt:
        print("\n중단")
    finally:
        sim.status()
        sim.save_log()


if __name__ == "__main__":
    main()
