#!/usr/bin/env python3
"""
run_arm_test.py 가 --csv 로 남긴 로그를 그림으로 그린다.

레이아웃
    +---------------------+----------+
    |  자세 roll/pitch/yaw |    F1    |
    |                     +----------+
    +---------------------+    F2    |
    |  위치 x / y / z      +----------+
    |                     |    F3    |
    |                     +----------+
    |                     |    F4    |
    +---------------------+----------+

사용법
    python3 plot_result.py to_m10.csv
    python3 plot_result.py to_m10.csv --title "a = -10 cm" --out fig.png
    python3 plot_result.py to_m0.csv to_m10.csv to_m20.csv     # 여러 개를 한 장씩
"""
import argparse
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

C_ATT = {"roll": "#d62728", "pitch": "#2ca02c", "yaw": "#1f77b4"}
C_POS = {"x": "#d62728", "y": "#2ca02c", "z": "#1f77b4"}
C_F = ["#e45756", "#4c78a8", "#54a24b", "#f58518"]
F_LABEL = ["F1 (front)", "F2 (rear)", "F3 (rear)", "F4 (front)"]


def draw(csv, title=None, out=None, hover=None, dpi=130,
         ylim_att=None, ylim_pos=None, ylim_f=None, att_rad=False,
         events=None):
    d = np.loadtxt(csv, delimiter=",", skiprows=1)
    t = d[:, 0]
    x, y, z = d[:, 1], d[:, 2], d[:, 3]
    roll, pitch, yaw = d[:, 4], d[:, 5], d[:, 6]
    if att_rad:
        roll, pitch, yaw = np.radians(roll), np.radians(pitch), np.radians(yaw)
    F = d[:, 7:11]

    fig = plt.figure(figsize=(15, 8.5))
    gs = GridSpec(4, 2, figure=fig, width_ratios=[1.75, 1],
                  hspace=0.45, wspace=0.22)

    # ---------------- 왼쪽 위: 자세 ----------------
    ax_att = fig.add_subplot(gs[0:2, 0])
    ax_att.plot(t, roll,  color=C_ATT["roll"],  lw=2.25, ls="-",  label="roll")
    ax_att.plot(t, pitch, color=C_ATT["pitch"], lw=2.25, ls="-.", label="pitch")
    ax_att.plot(t, yaw,   color=C_ATT["yaw"],   lw=2.25, ls=":",  label="yaw")
    ax_att.axhline(0, color="0.6", lw=0.7, ls="--")
    ax_att.set_ylabel("attitude [rad]" if att_rad else "attitude [deg]")
    ax_att.set_title("Attitude", fontsize=11, loc="left", fontweight="bold")
    ax_att.legend(ncol=3, fontsize=9, loc="upper right", framealpha=0.9)
    ax_att.grid(alpha=0.3)
    if ylim_att:
        ax_att.set_ylim(*ylim_att)

    # ---------------- 왼쪽 아래: 위치 ----------------
    ax_pos = fig.add_subplot(gs[2:4, 0], sharex=ax_att)
    ax_pos.plot(t, x, color=C_POS["x"], lw=2.25, ls="-",  label="x")
    ax_pos.plot(t, y, color=C_POS["y"], lw=2.25, ls="-.", label="y")
    ax_pos.plot(t, z, color=C_POS["z"], lw=2.25, ls=":",  label="z")
    ax_pos.axhline(0, color="0.6", lw=0.7, ls="--")
    ax_pos.set_ylabel("position [m]")
    ax_pos.set_xlabel("time [s]")
    ax_pos.set_title("Position", fontsize=11, loc="left", fontweight="bold")
    ax_pos.legend(ncol=3, fontsize=9, loc="lower right", framealpha=0.9)
    ax_pos.grid(alpha=0.3)
    if ylim_pos:
        ax_pos.set_ylim(*ylim_pos)

    # ---------------- 오른쪽: 모터 추력 4단 ----------------
    if ylim_f:
        flo, fhi = ylim_f
    else:
        fmin, fmax = F.min(), F.max()
        pad = max(0.5, 0.08 * (fmax - fmin))
        flo, fhi = fmin - pad, fmax + pad
    axes_f = []
    for i in range(4):
        ax = fig.add_subplot(gs[i, 1], sharex=ax_att)
        ax.plot(t, F[:, i], color=C_F[i], lw=1.4)
        ax.set_ylim(flo, fhi)
        ax.set_ylabel("N", fontsize=9)
        ax.grid(alpha=0.3)
        ax.text(0.015, 0.86, F_LABEL[i], transform=ax.transAxes,
                fontsize=9.5, fontweight="bold", va="top", color=C_F[i])
        if hover is not None:
            ax.axhline(hover, color="0.45", lw=0.8, ls=":")
        ax.axhline(0, color="0.75", lw=0.7)
        if i < 3:
            plt.setp(ax.get_xticklabels(), visible=False)
        axes_f.append(ax)
    axes_f[0].set_title("Motor thrust", fontsize=11, loc="left", fontweight="bold")
    axes_f[-1].set_xlabel("time [s]")

    ax_att.set_xlim(t[0], t[-1])
    if events:
        for a_ in [ax_att, ax_pos] + axes_f:
            for et, ec, els in events:
                a_.axvline(et, color=ec, lw=1.0, ls=els, alpha=0.8)
    if title:
        fig.suptitle(title, fontsize=13, fontweight="bold", y=0.975)
        fig.subplots_adjust(top=0.92)

    out = out or os.path.splitext(csv)[0] + ".png"
    fig.savefig(out, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    print(f"[저장] {out}")
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument("csv", nargs="+")
    p.add_argument("--title", default=None)
    p.add_argument("--out", default=None)
    p.add_argument("--hover", type=float, default=None,
                   help="호버 기준선 [N]. 예: 총질량 6kg -> 14.72")
    p.add_argument("--dpi", type=int, default=130)
    p.add_argument("--ylim-att", nargs=2, type=float, metavar=("LO", "HI"),
                   help="자세 y축 고정 [deg], 예: --ylim-att -1 1")
    p.add_argument("--ylim-pos", nargs=2, type=float, metavar=("LO", "HI"),
                   help="위치 y축 고정 [m]")
    p.add_argument("--ylim-f", nargs=2, type=float, metavar=("LO", "HI"),
                   help="추력 y축 고정 [N], 예: --ylim-f 0 30")
    p.add_argument("--att-rad", action="store_true", help="자세를 rad 로 표시")
    a = p.parse_args()
    for c in a.csv:
        t = a.title if (a.title and len(a.csv) == 1) else os.path.basename(c)
        o = a.out if (a.out and len(a.csv) == 1) else None
        draw(c, title=t, out=o, hover=a.hover, dpi=a.dpi,
             ylim_att=a.ylim_att, ylim_pos=a.ylim_pos, ylim_f=a.ylim_f,
             att_rad=a.att_rad)


if __name__ == "__main__":
    main()
