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
  arm J1 J2 J3 J4    팔 관절 목표각 [deg]. 호버 중에만. 예:  arm 45 0 0 0
  tcp X Y Z [T]      TCP 목표 위치 [m, 기체 body frame]. DLS-IK 로 풀어 quintic 궤적 생성
                     예:  tcp 0.55 0 -0.15      (T 생략시 3초)
  tcpnow             현재 TCP 위치 출력
  hold               짐벌 ON — 지금 TCP 를 world 좌표에 고정하고 팔이 기체 움직임을 보상
  hold off           짐벌 해제
  orbit R T [평면]   기체를 원운동시킨다. R=반지름[m], T=주기[s], 평면=xy|xz|yz
                     예:  orbit 0.10 20 xy      (반지름 10cm, 20초 주기)
  orbit off          원운동 정지
                     뷰어의 Control 패널 슬라이더로도 조작 가능 (호버 중에만 반영,
                     이륙/착륙 중에는 자동으로 고정된다)
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
        # 팔 관절: cmd = 현재 지령(레이트 제한 적용), target = 사용자가 준 목표
        self.arm_target = None if self.arm_ctrl is None else self.arm_ctrl.copy()
        self.arm_written = None      # 직전에 우리가 ctrl 에 쓴 값 (뷰어 조작 감지용)
        self.arm_traj = None         # (t0, T, q_start, q_goal) quintic 궤적
        self.hold_w = None           # world frame 에 고정할 TCP 위치 (짐벌 모드)
        self.hold_warned = 0.0
        self.pert = None             # 뷰어의 MjvPerturb (마우스 외란)
        self.hold_bad_since = None   # 짐벌 오차가 기준을 넘기 시작한 시각
        self.orbit = None            # (t0, R, T, plane, center) 기체 원운동
        self.arm_qadr = []
        if self.arm_ctrl is not None:
            for i in range(self.n_drone_act, self.model.nu):
                j = self.model.actuator_trnid[i, 0]
                self.arm_qadr.append(self.model.jnt_qposadr[j])
        self.ik_data = mujoco.MjData(self.model) if self.arm_qadr else None
        self.tcp_sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, b"arm_tcp")
        self.mount_b = None
        self.reach_min = self.reach_max = None
        if self.arm_qadr:
            mb = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, b"arm_base_link")
            self.mount_b = np.array(self.model.body_pos[mb])
            self.reach_min, self.reach_max = self.compute_reach()
            print(f"      팔 장착점 {np.round(self.mount_b,3)} m, "
                  f"TCP 도달 반경 {self.reach_min:.3f} ~ {self.reach_max:.3f} m")

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
                    self.orbit = None
                    self.hold_w = None
                    self.dist_F[:] = 0.0
                    self.dist_M[:] = 0.0
                    # DoB 는 접지 직전까지 유지한다. 여기서 끄면 팔 이동으로 생긴
                    # CoM 불일치 보상이 갑자기 사라져 기체가 크게 기운다.
                    self.ramp_from = self.data.sensordata[self.adr[self.ip] + 2]
                    self.ramp_to = self.ground_z - 0.02
                    self.ramp_t0 = self.data.time
                    print(f"[{self.data.time:7.2f}] 착륙 시작")
                else:
                    print("  비행 중이 아님")
            elif c == "arm":
                if self.arm_ctrl is None:
                    print("  이 모델은 팔 관절이 고정됨 (--rigid 로 만든 모델)")
                elif self.mode != "HOVER":
                    print("  호버 상태에서만 팔을 움직일 수 있음")
                else:
                    v = [float(x) for x in p[1:1 + len(self.arm_ctrl)]]
                    v = v + list(np.degrees(self.arm_target[len(v):]))
                    lo = np.array(self.model.actuator_ctrlrange[self.n_drone_act:, 0])
                    hi = np.array(self.model.actuator_ctrlrange[self.n_drone_act:, 1])
                    lo[1] = max(lo[1], math.radians(self.args.elbow_min))
                    self.arm_target = np.clip(np.radians(v), lo, hi)
                    print(f"[{self.data.time:7.2f}] 팔 목표 "
                          f"{np.round(np.degrees(self.arm_target), 1)} deg "
                          f"(최대 {self.args.arm_rate:.0f} deg/s)")
            elif c == "tcp":
                if self.arm_ctrl is None:
                    print("  이 모델은 팔 관절이 고정됨 (--rigid)")
                elif self.mode != "HOVER":
                    print("  호버 상태에서만 사용 가능")
                else:
                    goal = np.array([float(x) for x in p[1:4]])
                    T = float(p[4]) if len(p) > 4 else self.args.ik_duration
                    q_goal, err = self.solve_ik(goal, self.args.ik_lambda,
                                                self.args.ik_max_qdot)
                    q_now = np.array([self.data.qpos[a] for a in self.arm_qadr])
                    self.arm_traj = (self.data.time, T, q_now, q_goal)
                    self.arm_target = q_goal.copy()
                    reach = self.tcp_body(q_goal)
                    print(f"[{self.data.time:7.2f}] TCP 목표 {np.round(goal,3)} m (body frame)")
                    print(f"          IK 해 {np.round(np.degrees(q_goal),1)} deg, "
                          f"도달점 {np.round(reach,3)}, 오차 {err*1000:.1f} mm, {T:.1f}s 궤적")
                    if err > 0.01:
                        print("          !! 도달 불가 (최소자승 근사해)")
            elif c == "hold":
                if self.arm_ctrl is None:
                    print("  이 모델은 팔 관절이 고정됨 (--rigid)")
                elif len(p) > 1 and p[1].lower() in ("off", "0", "stop"):
                    self.hold_w = None
                    self.arm_target = self.arm_ctrl.copy()
                    print(f"[{self.data.time:7.2f}] 짐벌 해제")
                elif self.mode != "HOVER":
                    print("  호버 상태에서만 사용 가능")
                else:
                    # 팔꿈치가 하한 아래면 먼저 올려놓는다 (뒤집힘 방지)
                    lo_j, _ = self.arm_limits(3)
                    if self.arm_ctrl[1] < lo_j[1]:
                        print(f"          joint2 {math.degrees(self.arm_ctrl[1]):.1f} deg "
                              f"-> {self.args.elbow_min:.1f} deg 로 보정 (팔꿈치 방향 고정)")
                        self.arm_ctrl[1] = lo_j[1]
                        self.arm_target[1] = lo_j[1]
                    self.hold_w = self.tcp_world()
                    self.hold_bad_since = None
                    self.arm_traj = None
                    sv = self.ik_sigma_min()
                    print(f"[{self.data.time:7.2f}] 짐벌 ON — TCP 를 world "
                          f"{np.round(self.hold_w,4)} 에 고정")
                    print(f"          현재 자코비안 sigma_min = {sv:.4f} m/rad", end="")
                    if sv < 0.03:
                        print("   !! 특이점 근처. 팔을 굽히세요 (예: arm 45 50 0 0)")
                    else:
                        print(f"   (관절 1 deg/s 당 TCP {sv*math.pi/180*1000:.2f} mm/s)")
            elif c == "tcpnow":
                if self.arm_qadr:
                    print(f"[{self.data.time:7.2f}] TCP (body) = "
                          f"{np.round(self.tcp_body(), 4)} m")
            elif c == "orbit":
                if len(p) > 1 and p[1].lower() in ("off", "0", "stop"):
                    self.orbit = None
                    print(f"[{self.data.time:7.2f}] 원운동 정지")
                elif self.mode != "HOVER":
                    print("  호버 상태에서만 사용 가능")
                else:
                    R = float(p[1]) if len(p) > 1 else 0.10
                    T = float(p[2]) if len(p) > 2 else 20.0
                    pl = p[3].lower() if len(p) > 3 else "xy"
                    pos0 = self.get(self.ip).copy()
                    self.orbit = (self.data.time, R, T, pl, pos0)
                    v = 2 * math.pi * R / T
                    print(f"[{self.data.time:7.2f}] 원운동 시작 — 반지름 {R*100:.0f} cm, "
                          f"주기 {T:.1f} s, 평면 {pl}")
                    print(f"          접선속도 {v:.3f} m/s ({1/T:.3f} Hz)"
                          + (f", 짐벌 ON 상태" if self.hold_w is not None else ""))
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
              f"{self.obs.F_hat[1]:+.2f} {self.obs.F_hat[2]:+.2f}]"
              + (f" arm={np.round(np.degrees([self.data.qpos[a] for a in self.arm_qadr]),1)}"
                 f" x_c={self.x_c:+.4f}" if self.arm_qadr else "")
              + (f" hold_err={np.linalg.norm(self.tcp_world()-self.hold_w)*1000:.1f}mm"
                 if self.hold_w is not None else ""))

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
            if self.mode == "LAND" and self.dob_on and pos[2] < self.ground_z + 0.20:
                self.dob_on = False
                self.obs.reset()
                print(f"[{t:7.2f}] 접지 임박 — DoB 해제")
            if self.mode == "LAND" and pos[2] < self.ground_z + 0.03 and abs(vel[2]) < 0.15:
                self.armed = False
                self.mode = "DONE"
                print(f"[{t:7.2f}] 접지. 시뮬레이션 종료")
        else:
            z_ref = self.args.alt if self.mode == "HOVER" else self.ground_z
        if self.orbit is not None and self.mode == "HOVER":
            t0, R, T, pl, c0 = self.orbit
            th = 2 * math.pi * (t - t0) / T
            dx, dy, dz = 0.0, 0.0, 0.0
            if pl == "xy":
                dx, dy = R * math.sin(th), R * (1 - math.cos(th))
            elif pl == "xz":
                dx, dz = R * math.sin(th), R * (1 - math.cos(th))
            elif pl == "yz":
                dy, dz = R * math.sin(th), R * (1 - math.cos(th))
            self.target[0], self.target[1] = c0[0] + dx, c0[1] + dy
            z_ref = c0[2] + dz
        ref = np.array([self.target[0], self.target[1], z_ref])

        # 무게중심 추종 (팔이 움직이면 CoM 이 실시간으로 변한다)
        if self.args.pc_track and self.arm_ctrl is not None:
            cw = np.array(self.data.subtree_com[self.bid]) - np.array(self.data.xpos[self.bid])
            self.com_b = Rwb.T @ cw
            self.x_c = float(self.com_b[0])
            self.alloc.Pc = np.array([self.com_b[0], self.com_b[1], 0.0])

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
        if self.arm_ctrl is not None and self.arm_target is not None:
            # 뷰어 Control 슬라이더로 값이 바뀌었는지 확인
            cur = np.array(self.data.ctrl[n:])
            if self.arm_written is not None and not np.allclose(cur, self.arm_written, atol=1e-9):
                if self.mode == "HOVER":
                    lo_a = np.array(self.model.actuator_ctrlrange[n:, 0])
                    hi_a = np.array(self.model.actuator_ctrlrange[n:, 1])
                    lo_a[1] = max(lo_a[1], math.radians(self.args.elbow_min))
                    self.arm_target = np.clip(cur, lo_a, hi_a)
                    self.arm_traj = None
                # 호버가 아니면 무시 -> 아래에서 원래 값으로 덮어써 고정 유지
            if self.hold_w is not None and self.mode == "HOVER":
                # 오차는 '실측 TCP' 로, 적분은 '지령각' 으로 한다.
                # (실측각으로 적분하면 서보 처짐이 되먹여져 팔이 서서히 무너진다)
                goal_b_raw = Rwb.T @ (self.hold_w - pos)
                goal_b, sat = self.clamp_reach(goal_b_raw)
                e = goal_b - Rwb.T @ (self.tcp_world() - pos)
                q_act = np.array([self.data.qpos[a] for a in self.arm_qadr])
                jids = [self.model.actuator_trnid[self.n_drone_act + i, 0] for i in range(3)]
                lo_j, hi_j = self.arm_limits(3)
                self.tcp_body(q_act)                       # ik_data 를 실측 자세로 갱신
                jacp = np.zeros((3, self.model.nv))
                mujoco.mj_jacSite(self.model, self.ik_data, jacp, None, self.tcp_sid)
                J = jacp[:, [self.model.jnt_dofadr[j] for j in jids]]
                lam = self.args.ik_lambda
                v_des = self.args.hold_gain * e            # [m/s]
                qdot = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(3), v_des)
                rate = math.radians(self.args.hold_rate)
                qdot = np.clip(qdot, -rate, rate)
                self.arm_ctrl[:3] = np.clip(self.arm_ctrl[:3] + qdot * dt, lo_j, hi_j)
                self.arm_target[:3] = self.arm_ctrl[:3]
                # --- 안전장치: 실제 오차(클램프 전 목표 기준) 감시 ---
                err_true = float(np.linalg.norm(
                    goal_b_raw - Rwb.T @ (self.tcp_world() - pos)))
                if err_true > self.args.hold_tol:
                    if self.hold_bad_since is None:
                        self.hold_bad_since = self.data.time
                    held = self.data.time - self.hold_bad_since
                    if self.data.time - self.hold_warned > 3.0:
                        self.hold_warned = self.data.time
                        print(f"[{self.data.time:7.2f}] 짐벌 오차 {err_true*1000:.0f} mm "
                              f"({held:.1f}/{self.args.hold_timeout:.0f} s"
                              f"{', 도달한계 포화' if sat else ''})")
                    if held >= self.args.hold_timeout:
                        self.hold_w = None
                        self.hold_bad_since = None
                        self.arm_target = self.arm_ctrl.copy()
                        print(f"[{self.data.time:7.2f}] ** 짐벌 자동 해제 ** "
                              f"오차 {err_true*1000:.0f} mm 가 "
                              f"{self.args.hold_timeout:.0f} 초 이상 지속됨")
                else:
                    self.hold_bad_since = None
            elif self.arm_traj is not None:
                t0, T, q0, q1 = self.arm_traj
                u = min(1.0, max(0.0, (self.data.time - t0) / T))
                sca = 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5      # quintic
                self.arm_ctrl = q0 + (q1 - q0) * sca
                if u >= 1.0:
                    self.arm_traj = None
            else:
                step = math.radians(self.args.arm_rate) * dt
                e = self.arm_target - self.arm_ctrl
                self.arm_ctrl += np.clip(e, -step, step)

        cmd = np.clip(np.concatenate([self.T_cmd, self.th_cmd]), self.lo[:n], self.hi[:n])
        self.data.ctrl[:n] = cmd
        if self.arm_ctrl is not None:
            self.data.ctrl[n:] = self.arm_ctrl
            self.arm_written = self.arm_ctrl.copy()
        mujoco.mj_step(self.model, self.data)

        if self.log_path is not None:
            arm_q = [math.degrees(self.data.qpos[a]) for a in self.arm_qadr] \
                if self.arm_qadr else [0.0, 0.0, 0.0, 0.0]
            self.log_rows.append([t, *pos, *np.degrees(rpy),
                                  *self.data.actuator_force[0:4], *np.degrees(srv),
                                  *F_hat, *M_hat, *self.dist_F,
                                  1.0 if self.dob_on else 0.0,
                                  *arm_q, self.x_c])

    def tcp_body(self, q=None):
        """현재(또는 주어진) 관절각에서 TCP 위치를 base body frame 으로 반환."""
        d = self.ik_data
        d.qpos[:] = 0.0
        d.qpos[3] = 1.0                       # base 를 원점/무회전에 둔다 -> world = body
        if q is None:
            q = [self.data.qpos[a] for a in self.arm_qadr]
        for a, v in zip(self.arm_qadr, q):
            d.qpos[a] = v
        mujoco.mj_forward(self.model, d)
        return np.array(d.site_xpos[self.tcp_sid])

    def compute_reach(self, ns=13):
        """장착점에서 TCP 까지 도달 가능한 반경 [최소, 최대]."""
        lo, hi = self.arm_limits(3)
        R = []
        for a in np.linspace(lo[0], hi[0], ns):
            for b in np.linspace(lo[1], hi[1], ns):
                for c in np.linspace(lo[2], hi[2], 7):
                    R.append(np.linalg.norm(self.tcp_body([a, b, c]) - self.mount_b))
        return float(np.min(R)), float(np.max(R))

    def clamp_reach(self, goal_b, margin=0.02):
        """목표 TCP 를 도달 가능한 껍질 안으로 투영. 관절 한계에 박히는 것을 예방."""
        v = goal_b - self.mount_b
        r = float(np.linalg.norm(v))
        lo = self.reach_min + margin
        hi = self.reach_max - margin
        if r < 1e-9:
            return goal_b, False
        if r > hi:
            return self.mount_b + v / r * hi, True
        if r < lo:
            return self.mount_b + v / r * lo, True
        return goal_b, False

    def arm_limits(self, k=3):
        """팔 관절 [하한, 상한]. joint2 는 elbow_min 으로 하한을 올려
        팔꿈치 뒤집힘과 완전 신전(특이점)을 동시에 막는다."""
        jids = [self.model.actuator_trnid[self.n_drone_act + i, 0] for i in range(k)]
        lo = np.array([self.model.jnt_range[j][0] if self.model.jnt_limited[j] else -np.pi
                       for j in jids])
        hi = np.array([self.model.jnt_range[j][1] if self.model.jnt_limited[j] else np.pi
                       for j in jids])
        if k >= 2:
            lo[1] = max(lo[1], math.radians(self.args.elbow_min))
        return lo, hi

    def tcp_world(self):
        """실제 시뮬레이션 상태에서의 TCP world 좌표."""
        return np.array(self.data.site_xpos[self.tcp_sid])

    def ik_sigma_min(self, q=None):
        """TCP 위치 자코비안(팔 3관절)의 최소 특이값 [m/rad]."""
        d = self.ik_data
        d.qpos[:] = 0.0
        d.qpos[3] = 1.0
        q = q if q is not None else [self.data.qpos[a] for a in self.arm_qadr]
        for a, v in zip(self.arm_qadr, q):
            d.qpos[a] = v
        mujoco.mj_forward(self.model, d)
        jacp = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, d, jacp, None, self.tcp_sid)
        jids = [self.model.actuator_trnid[self.n_drone_act + i, 0] for i in range(3)]
        J = jacp[:, [self.model.jnt_dofadr[j] for j in jids]]
        return float(np.linalg.svd(J, compute_uv=False).min())

    def solve_ik(self, goal_b, lam=0.05, max_dq=0.5, iters=400, tol=1e-4):
        """위치 전용 DLS-IK.  dq = J^T (JJ^T + lam^2 I)^-1 * e   (레포와 동일 형태)
        오차가 줄어들 때만 스텝을 받아들이는 backtracking 을 추가했다.
        반환: (관절각 배열, 최종 오차 [m])"""
        d = self.ik_data
        n = len(self.arm_qadr)
        jids = [self.model.actuator_trnid[self.n_drone_act + i, 0] for i in range(n)]
        lo, hi = self.arm_limits(n)
        cols = [self.model.jnt_dofadr[j] for j in jids]
        jacp = np.zeros((3, self.model.nv))

        def fk_err(q):
            d.qpos[:] = 0.0
            d.qpos[3] = 1.0
            for a, v in zip(self.arm_qadr, q):
                d.qpos[a] = v
            mujoco.mj_forward(self.model, d)
            e = goal_b - np.array(d.site_xpos[self.tcp_sid])
            return e, float(np.linalg.norm(e))

        # 시작점 여러 개에서 풀고 가장 좋은 해를 고른다 (지역해 회피)
        q_now = np.array([self.data.qpos[a] for a in self.arm_qadr])
        starts = [q_now, np.clip(np.zeros(n), lo, hi),
                  np.clip((lo + hi) / 2.0, lo, hi),
                  np.clip(q_now + np.array([0.0, 0.6, 0.0, 0.0][:n]), lo, hi)]
        best_q, best_e = q_now.copy(), fk_err(q_now)[1]

        for q0 in starts:
            q = np.clip(np.array(q0, dtype=float), lo, hi)
            e, err = fk_err(q)
            for _ in range(iters):
                if err < tol:
                    break
                mujoco.mj_jacSite(self.model, d, jacp, None, self.tcp_sid)
                J = jacp[:, cols]
                dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(3), e)
                dq = np.clip(dq, -max_dq, max_dq)
                step = 1.0
                improved = False
                for _ in range(12):                  # backtracking
                    q_try = np.clip(q + dq * step, lo, hi)
                    e_try, err_try = fk_err(q_try)
                    if err_try < err:
                        q, e, err = q_try, e_try, err_try
                        improved = True
                        break
                    step *= 0.5
                if not improved:
                    break
            if err < best_e:
                best_q, best_e = q.copy(), err
        fk_err(best_q)
        return best_q, best_e

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
                   "dist_x,dist_y,dist_z,dob_on,"
                   "arm_j1,arm_j2,arm_j3,arm_j4,x_c")
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
    p.add_argument("--arm-rate", type=float, default=30.0,
                   help="팔 관절 최대 각속도 [deg/s]")
    p.add_argument("--ik-duration", type=float, default=3.0, help="IK 궤적 시간 [s]")
    p.add_argument("--ik-lambda", type=float, default=0.05, help="DLS 감쇠계수")
    p.add_argument("--ik-max-qdot", type=float, default=0.5, help="IK 반복당 관절속도 클램프")
    p.add_argument("--elbow-min", type=float, default=1.0,
                   help="joint2 하한 [deg]. 팔꿈치 뒤집힘과 완전 신전(특이점)을 막는다")
    p.add_argument("--hold-tol", type=float, default=0.05,
                   help="짐벌 허용 오차 [m]. 이를 넘는 상태가 지속되면 자동 해제")
    p.add_argument("--hold-timeout", type=float, default=10.0,
                   help="짐벌 자동 해제까지의 지속 시간 [s]. 0 이면 해제하지 않음")
    p.add_argument("--hold-gain", type=float, default=6.0,
                   help="짐벌 보상 게인 [1/s]. v_des = K * (TCP 오차)")
    p.add_argument("--hold-rate", type=float, default=180.0,
                   help="짐벌 모드 관절 최대 각속도 [deg/s]")
    p.add_argument("--pc-track", action="store_true",
                   help="allocator 의 Pc_ 를 실제 CoM 으로 매 스텝 갱신")
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
                sim.pert = v.perturb
                print("  [뷰어] 기체를 더블클릭해 선택한 뒤 "
                      "Ctrl+우드래그 = 힘, Ctrl+좌드래그 = 토크")
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
