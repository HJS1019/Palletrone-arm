#!/usr/bin/env python3
"""
Palletrone + Forest Cone Harvesting Robot 결합 XML 생성기

  - 팔은 드론 '아래쪽' 에 장착
  - 팔 자세는 joint1 = 90 도 (앞쪽 +x 로 수평하게 뻗은 최악 자세)
  - 갠트리(carriage_x / carriage_y)는 지상 실험 장비이므로 제외, base_link 부터 가져옴
  - 관절은 그대로 살려두고 position 액추에이터로 자세를 유지시킴
    (움직임 명령은 나중에. 지금은 목표각을 고정해 자세만 잡아둠)

사용법
  python3 build_arm_drone.py \
      --drone ~/PPP_sim/src/plant/xml/Palletrone.xml \
      --arm   ~/cone_harvester_sim_ws/src/cone_harvester_sim/models/Forest_Cone_Harvesting_Robot.xml \
      --a -0.014 --cw-mass 0.610 --cw-pos -0.251 --base-mass 3.3 --arm-len 0.23

결과
  드론 XML 과 같은 폴더에 Palletrone_with_arm.xml 생성 (STL 은 arm_meshes/ 로 복사)
"""
import argparse
import math
import os
import re
import shutil
import xml.etree.ElementTree as ET


def indent(elem, level=0):
    pad = "\n" + "  " * level
    if len(elem):
        if not (elem.text or "").strip():
            elem.text = pad + "  "
        for c in elem:
            indent(c, level + 1)
        if not (c.tail or "").strip():
            c.tail = pad
    if level and not (elem.tail or "").strip():
        elem.tail = pad


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--drone", required=True, help="Palletrone.xml 경로")
    p.add_argument("--arm", required=True, help="Forest_Cone_Harvesting_Robot.xml 경로")
    p.add_argument("--a", type=float, default=-0.014,
                   help="팔 장착점 x 오프셋 [m] (드론 CoM 기준)")
    p.add_argument("--mount-z", type=float, default=-0.125,
                   help="팔 장착점 z [m]. 기본은 본체 바닥면")
    p.add_argument("--base-mass", type=float, default=3.3, help="드론 본체 질량 [kg]")
    p.add_argument("--arm-len", type=float, default=0.23,
                   help="원점->프로펠러 암 길이 [m]")
    p.add_argument("--cw-mass", type=float, default=0.610, help="변압기 질량 [kg]")
    p.add_argument("--cw-pos", type=float, default=-0.251, help="변압기 x 위치 [m]")
    p.add_argument("--cw-z", type=float, default=0.165,
                   help="변압기 z 위치 [m]. 기본 0.165 = 본체 윗면(0.135)+박스 반높이(0.03)")
    p.add_argument("--j1", type=float, default=90.0, help="joint1 목표각 [deg]")
    p.add_argument("--j2", type=float, default=0.0)
    p.add_argument("--j3", type=float, default=0.0)
    p.add_argument("--j4", type=float, default=0.0)
    p.add_argument("--kp", type=float, default=80.0, help="팔 관절 위치 게인")
    p.add_argument("--top-height", type=float, default=0.404,
                   help="착륙 상태에서 지면~드론 윗면 높이 [m] (실측 0.404). "
                        "0 이면 다리를 만들지 않는다")
    p.add_argument("--gear-span", type=float, default=0.25,
                   help="다리 배치 반경 [m] (본체 반치수 0.34 이내)")
    p.add_argument("--rigid", action="store_true",
                   help="팔 관절을 제거하고 지정 자세로 굳힌 강체로 만든다 "
                        "(비행 중 서보 브레이크 상태에 해당)")
    p.add_argument("--out", default=None)
    args = p.parse_args()

    drone_path = os.path.abspath(args.drone)
    arm_path = os.path.abspath(args.arm)
    out_dir = os.path.dirname(drone_path)
    out_path = args.out or os.path.join(out_dir, "Palletrone_with_arm.xml")

    # ---------- 팔 STL 복사 ----------
    arm_dir = os.path.dirname(arm_path)
    arm_tree = ET.parse(arm_path)
    arm_root = arm_tree.getroot()
    comp = arm_root.find("compiler")
    meshdir = comp.get("meshdir", "") if comp is not None else ""
    src_mesh_dir = os.path.join(arm_dir, meshdir)

    dst_mesh_dir = os.path.join(out_dir, "arm_meshes")
    os.makedirs(dst_mesh_dir, exist_ok=True)

    arm_meshes = []
    for mesh in arm_root.findall("./asset/mesh"):
        fn = mesh.get("file")
        src = os.path.join(src_mesh_dir, fn)
        if os.path.exists(src):
            shutil.copy2(src, os.path.join(dst_mesh_dir, os.path.basename(fn)))
        else:
            print(f"  [경고] STL 없음: {src}")
        arm_meshes.append((mesh.get("name"), os.path.basename(fn), mesh.get("scale")))

    # ---------- base_link 서브트리 추출 ----------
    base_link = None
    for b in arm_root.iter("body"):
        if b.get("name") == "base_link":
            base_link = b
            break
    if base_link is None:
        raise SystemExit("팔 XML 에서 base_link 를 못 찾음")

    # ---------- 드론 XML 로드 ----------
    txt = open(drone_path, encoding="utf-8").read()
    txt = txt.replace('mass="4.00"', f'mass="{args.base_mass:.4f}"', 1)
    r_new = args.arm_len / math.sqrt(2.0)
    txt = txt.replace("0.148492", f"{r_new:.6f}")
    txt = re.sub(r'(<body name="base"\s+pos="[^"]*?)\s[\d.eE+-]+">',
                 lambda m: f'{m.group(1)} 0.115">', txt, count=1)
    root = ET.fromstring(txt)

    # 팔 메쉬는 이름 충돌을 피해 arm_ 접두사, 경로는 arm_meshes/
    asset = root.find("asset")
    for name, fn, scale in arm_meshes:
        e = ET.SubElement(asset, "mesh")
        e.set("name", f"arm_{name}")
        e.set("file", f"arm_meshes/{fn}")
        if scale:
            e.set("scale", scale)

    # 팔 서브트리의 mesh 참조와 이름에 접두사
    for b in base_link.iter("body"):
        b.set("name", "arm_" + b.get("name"))
    for g in base_link.iter("geom"):
        if g.get("mesh"):
            g.set("mesh", "arm_" + g.get("mesh"))
        if g.get("name"):
            g.set("name", "arm_" + g.get("name"))
        # 팔은 지금 단계에서 충돌 비활성 (드론 본체와의 자기충돌 방지)
        g.set("contype", "0")
        g.set("conaffinity", "0")
        g.attrib.pop("class", None)
        if g.get("type") is None:
            g.set("type", "mesh")
        if g.get("rgba") is None:
            g.set("rgba", "0.75 0.75 0.78 1")
    if args.rigid:
        # 관절축(z) 둘레로 목표각만큼 회전시킨 것을 body 의 quat 에 흡수시킨다
        qmap = {"joint1": math.radians(args.j1), "joint2": math.radians(args.j2),
                "joint3": math.radians(args.j3), "joint4": math.radians(args.j4)}
        for b in base_link.iter("body"):
            js = b.findall("joint")
            if not js:
                continue
            ang = qmap.get(js[0].get("name"), 0.0)
            for j in js:
                b.remove(j)
            q0 = [float(x) for x in (b.get("quat") or "1 0 0 0").split()]
            qz = [math.cos(ang / 2), 0.0, 0.0, math.sin(ang / 2)]   # z 축 회전
            w1, x1, y1, z1 = q0
            w2, x2, y2, z2 = qz
            b.set("quat", " ".join(f"{v:.10f}" for v in (
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)))
    else:
        for j in base_link.iter("joint"):
            j.set("name", "arm_" + j.get("name"))
            j.set("damping", "0.3")
            j.set("armature", "0.01")
    for s in base_link.iter("site"):
        s.set("name", "arm_" + s.get("name"))

    # 장착 위치로 이동 (원래 quat 은 그대로 두어 home 자세가 아래를 향하게 유지)
    base_link.set("pos", f"{args.a:.6f} 0 {args.mount_z:.6f}")

    drone_body = None
    for b in root.iter("body"):
        if b.get("name") == "base":
            drone_body = b
            break
    drone_body.append(base_link)

    # ---------- 랜딩기어 ----------
    if args.top_height and args.top_height > 0:
        BODY_TOP = 0.135          # base 원점 -> 본체 윗면
        BODY_BOT = -0.115         # base 원점 -> 본체 아랫면(충돌박스)
        z_land = args.top_height - BODY_TOP        # 착륙 시 base 원점 높이
        leg_len = (-BODY_BOT) + (-(BODY_BOT)) * 0  # placeholder
        leg_len = z_land + BODY_BOT                # 본체 아랫면에서 지면까지
        sp = args.gear_span
        for sx in (1, -1):
            for sy in (1, -1):
                lg = ET.SubElement(drone_body, "geom")
                lg.set("name", f"gear_{'p' if sx>0 else 'm'}{'p' if sy>0 else 'm'}")
                lg.set("type", "capsule")
                lg.set("fromto", f"{sx*sp:.4f} {sy*sp:.4f} {BODY_BOT:.4f} "
                                 f"{sx*sp:.4f} {sy*sp:.4f} {-z_land:.4f}")
                lg.set("size", "0.012")
                lg.set("rgba", "0.25 0.25 0.28 1")
                lg.set("contype", "1"); lg.set("conaffinity", "1")
                lg.set("condim", "6"); lg.set("friction", "5.0 0.1 0.1")
                lg.set("solref", "-2000 -40")
                lg.set("solimp", "0.99 0.999 0.0001")
        print(f"  [다리] 길이 {leg_len*100:.1f} cm, 착륙 시 base 원점 z = {z_land:.3f} m")

    # ---------- 변압기 ----------
    if args.cw_mass > 0:
        cw = ET.SubElement(drone_body, "body")
        cw.set("name", "transformer")
        cw.set("pos", f"{args.cw_pos:.6f} 0 {args.cw_z:.6f}")
        ine = ET.SubElement(cw, "inertial")
        ine.set("pos", "0 0 0")
        ine.set("mass", f"{args.cw_mass:.4f}")
        ine.set("diaginertia", "0.002 0.002 0.002")
        gg = ET.SubElement(cw, "geom")
        gg.set("name", "transformer_vis")
        gg.set("type", "box")
        gg.set("size", "0.089 0.05 0.03")
        gg.set("rgba", "0.9 0.55 0.1 1")
        gg.set("contype", "0")
        gg.set("conaffinity", "0")

    # ---------- 팔 관절 position 액추에이터 (자세 유지용) ----------
    act = root.find("actuator")
    for jn, lim in ([] if args.rigid else [("arm_joint1", (0.0, 1.5708)), ("arm_joint2", (-1.5708, 1.5708)),
                    ("arm_joint3", (-1.5708, 1.5708)), ("arm_joint4", (-3.15, 3.15))]):
        e = ET.SubElement(act, "position")
        e.set("name", f"act_{jn}")
        e.set("joint", jn)
        e.set("kp", f"{args.kp}")
        e.set("dampratio", "1")
        e.set("ctrlrange", f"{lim[0]} {lim[1]}")

    # ---------- 조명 / 바닥 (단독 실행용) ----------
    scene = ET.fromstring('''<mujoco>
      <visual>
        <headlight diffuse="0.85 0.85 0.85" ambient="0.45 0.45 0.45" specular="0.1 0.1 0.1"/>
        <global azimuth="130" elevation="-20"/>
        <map znear="0.02"/>
      </visual>
    </mujoco>''')
    root.insert(0, scene.find("visual"))
    tex = ET.SubElement(asset, "texture")
    tex.set("type", "skybox"); tex.set("builtin", "gradient")
    tex.set("rgb1", "0.60 0.72 0.88"); tex.set("rgb2", "0.92 0.95 0.99")
    tex.set("width", "512"); tex.set("height", "3072")
    t2 = ET.SubElement(asset, "texture")
    t2.set("type", "2d"); t2.set("name", "_grid"); t2.set("builtin", "checker")
    t2.set("mark", "edge"); t2.set("rgb1", "0.80 0.80 0.82"); t2.set("rgb2", "0.65 0.65 0.68")
    t2.set("markrgb", "0.95 0.95 0.95"); t2.set("width", "300"); t2.set("height", "300")
    mat = ET.SubElement(asset, "material")
    mat.set("name", "_grid"); mat.set("texture", "_grid"); mat.set("texuniform", "true")
    mat.set("texrepeat", "4 4"); mat.set("reflectance", "0.05")
    wb = root.find("worldbody")
    lt = ET.Element("light"); lt.set("pos", "2 2 6"); lt.set("dir", "-0.3 -0.3 -1")
    lt.set("directional", "true"); lt.set("diffuse", "0.7 0.7 0.7")
    wb.insert(0, lt)
    fl = ET.Element("geom"); fl.set("name", "_floor"); fl.set("size", "0 0 0.05")
    fl.set("type", "plane"); fl.set("material", "_grid")
    wb.insert(1, fl)
    for g in root.iter("geom"):
        if g.get("name") == "base_vis":
            g.set("rgba", "0.35 0.38 0.45 1")

    # ---------- keyframe: 팔 자세 ----------
    q = [math.radians(x) for x in (args.j1, args.j2, args.j3, args.j4)]
    old = root.find("keyframe")
    if old is not None:
        root.remove(old)
    kf = ET.SubElement(root, "keyframe")
    key = ET.SubElement(kf, "key")
    key.set("name", "arm_rigid" if args.rigid else "arm_forward")
    if args.rigid:
        key.set("qpos", "0 0 0.115 1 0 0 0 0 0 0 0")
        key.set("ctrl", "0 0 0 0 0 0 0 0")
    else:
        key.set("qpos", "0 0 0.115 1 0 0 0 " + " ".join("0" for _ in range(4))
                + " " + " ".join(f"{v:.6f}" for v in q))
        key.set("ctrl", "0 0 0 0 0 0 0 0 " + " ".join(f"{v:.6f}" for v in q))

    indent(root)
    ET.ElementTree(root).write(out_path, encoding="utf-8", xml_declaration=False)
    print(f"[생성] {out_path}")
    print(f"[STL ] {dst_mesh_dir}")

    # ---------- 검증 ----------
    try:
        import mujoco
        import numpy as np
        m = mujoco.MjModel.from_xml_path(out_path)
        d = mujoco.MjData(m)
        mujoco.mj_resetDataKeyframe(m, d, 0)
        mujoco.mj_forward(m, d)
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b"base")
        com = d.subtree_com[bid] - d.xpos[bid]
        print(f"\n[검증] 총질량 {m.body_subtreemass[bid]:.4f} kg")
        print(f"       합성 CoM (base 원점 기준) = {np.round(com, 4)} m")
        r = args.arm_len / math.sqrt(2.0)
        print(f"       x_c = {com[0]:+.4f} m  /  기하 한계 {r:.4f} m  "
              f"= {abs(com[0])/r:.1%}")
        W = m.body_subtreemass[bid] * 9.81
        Tf = (W / 2 + W * com[0] / (2 * r)) / 2
        Tr = (W / 2 - W * com[0] / (2 * r)) / 2
        print(f"       호버 모터당 {W/4:.2f} N -> 앞 {Tf:.2f} N ({Tf/(W/4):.0%}), "
              f"뒤 {Tr:.2f} N ({Tr/(W/4):.0%})")
        print(f"       판정: {'OK' if min(Tf, Tr) > 0.2 * W / 4 else '여유 부족'}")
    except Exception as e:
        print(f"[검증 실패] {e}")


if __name__ == "__main__":
    main()
