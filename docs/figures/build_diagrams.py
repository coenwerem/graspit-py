"""Render the documentation's mathematical illustrations with Matplotlib."""

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
from matplotlib import gridspec
from matplotlib.patches import Circle, Polygon, Arc
import numpy as np
from scipy.spatial import ConvexHull


_CMU_BOLD = Path("/usr/share/fonts/truetype/cmu/cmunsx.ttf")
_CMU_REG = Path("/usr/share/fonts/truetype/cmu/cmunss.ttf")
if _CMU_BOLD.exists():
    fm.fontManager.addfont(str(_CMU_BOLD))
if _CMU_REG.exists():
    fm.fontManager.addfont(str(_CMU_REG))

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["CMU Sans Serif", "DejaVu Sans", "Arial"],
    "font.size": 13,
    "axes.labelsize": 18,
    "axes.titlesize": 20,
    "axes.titleweight": "bold",
    "axes.labelweight": "normal",
    "axes.spines.top": True,
    "axes.spines.right": True,
    "axes.linewidth": 1,
    "legend.fontsize": 12,
    "xtick.labelsize": 14,
    "ytick.labelsize": 14,
    "figure.dpi": 600,
    "grid.alpha": 0.12,
    "grid.linewidth": 0.3,
    "text.usetex": False,
    "axes.unicode_minus": False,
})
plt.rcParams.update({"mathtext.fontset": "cm", "pdf.fonttype": 42, "svg.fonttype": "path"})

BLUE, RED, GREEN, GRAY = "#0d47a1", "#bf360c", "#2e7d32", "#9e9e9e"
REGION = "#E5E8FB"
OUTPUT = Path(__file__).resolve().parent


def direct_label(ax, text, xy_point, xy_text, color, fontsize=13):
    """Place a direct label with the house figure kit's black leader line."""
    ax.annotate(text, xy=xy_point, xytext=xy_text, color=color,
                fontsize=fontsize, ha="center", va="center",
                arrowprops={"arrowstyle": "-", "color": "black", "lw": 0.6,
                            "shrinkA": 1, "shrinkB": 1},
                zorder=5, annotation_clip=False)


def save(fig, name):
    for extension in ("png", "pdf", "svg"):
        fig.savefig(OUTPUT / f"{name}.{extension}", dpi=600, bbox_inches="tight",
                    facecolor="white", pad_inches=0.02)
    plt.close(fig)


def wrench_capacity():
    """A two-dimensional analogy, not a measured grasp or a 6D projection."""
    vertices = np.array([[-1.0, -0.8], [0.5, -1.1], [1.8, -0.3],
                         [1.4, 1.0], [0.2, 1.3], [-1.2, 0.4]])
    hull = ConvexHull(vertices)
    A, b = hull.equations[:, :2], hull.equations[:, 2]
    distances = -b / np.linalg.norm(A, axis=1)
    epsilon = distances.min()
    edge = np.argmin(distances)
    tangent = epsilon * A[edge] / np.linalg.norm(A[edge])
    ray = np.array([1.0, 0.35])
    denominators = A @ ray
    capacity = np.min(-b[denominators > 0] / denominators[denominators > 0])
    limit = capacity * ray

    fig = plt.figure(figsize=(7.2, 5.0))
    gs = gridspec.GridSpec(1, 1, figure=fig, left=0.12, right=0.98, bottom=0.13, top=0.97)
    ax = fig.add_subplot(gs[0])
    ax.add_patch(Polygon(vertices[hull.vertices], facecolor=REGION,
                         edgecolor=BLUE, linewidth=2, zorder=1))
    ax.add_patch(Circle((0, 0), epsilon, facecolor="white", edgecolor=GREEN,
                        linestyle="--", linewidth=1.8, zorder=2))
    ax.axhline(0, color=GRAY, lw=0.7, zorder=0)
    ax.axvline(0, color=GRAY, lw=0.7, zorder=0)
    ax.plot(0, 0, "o", color="black", ms=4, zorder=5)
    ax.text(-0.06, -0.13, "0", ha="right")
    ax.annotate("", xy=tangent, xytext=(0, 0),
                arrowprops={"arrowstyle": "<->", "color": GREEN, "lw": 1.6}, zorder=4)
    ax.text(*(tangent * 0.58 + [-0.04, 0.14]), r"$\varepsilon$", color=GREEN, fontsize=20)
    ax.annotate("", xy=limit, xytext=(0, 0),
                arrowprops={"arrowstyle": "->", "color": RED, "lw": 2}, zorder=4)
    ax.plot(*ray, "o", color=RED, ms=4, zorder=5)
    ax.plot(*limit, "o", markerfacecolor="white", markeredgecolor=RED, ms=7, zorder=5)
    direct_label(ax, r"Task load $\ell$", ray, (0.95, -0.55), RED)
    direct_label(ax, r"Limit $Q\ell$", limit, (1.90, 1.12), RED)
    ax.text(-0.74, 0.91, r"Wrench set $W$", color=BLUE, ha="center")
    ax.set(xlim=(-1.55, 2.28), ylim=(-1.4, 1.55), aspect="equal")
    ax.set_xticks([]); ax.set_yticks([])
    ax.set_xlabel(r"Normalized wrench coordinate $w_1$")
    ax.set_ylabel(r"Normalized wrench coordinate $w_2$")
    save(fig, "wrench-capacity")


def approach_geometry():
    """Show standoff measured from the grasp center, with its palm offset."""
    fig = plt.figure(figsize=(7.2, 4.7))
    gs = gridspec.GridSpec(1, 1, figure=fig, left=0.04, right=0.98, bottom=0.05, top=0.96)
    ax = fig.add_subplot(gs[0])
    ax.add_patch(Circle((0, 0), 0.54, facecolor=REGION, edgecolor=BLUE, lw=1.8))
    ax.text(0, -0.10, "Object", ha="center", color=BLUE)
    ax.plot([0, -3, -3], [0, 0, -1], "o", color="black", ms=5)
    ax.text(0.05, 0.08, r"$c$", fontsize=18)
    ax.text(-3.06, 0.12, r"$c-sd$", ha="right", fontsize=17)
    ax.annotate("", xy=(0, 0.83), xytext=(-3, 0.83),
                arrowprops={"arrowstyle": "<->", "color": "black", "lw": 1.2})
    ax.text(-1.5, 0.96, r"Standoff $s$", ha="center", fontsize=16)
    for x in (-3, 0):
        ax.plot([x, x], [0.18, 0.88], ":", color=GRAY, lw=1)
    ax.annotate("", xy=(-0.65, 0), xytext=(-2.8, 0),
                arrowprops={"arrowstyle": "->", "color": RED, "lw": 2})
    ax.text(-2.05, 0.35, r"Approach $d$", ha="center", color=RED, fontsize=16)
    ax.annotate("", xy=(-3, 0), xytext=(-3, -1),
                arrowprops={"arrowstyle": "->", "color": BLUE, "lw": 2})
    ax.text(-2.85, -0.56, r"$Rg$", color=BLUE, fontsize=19)
    ax.annotate("", xy=(-2.5, -1), xytext=(-3, -1),
                arrowprops={"arrowstyle": "->", "color": GRAY, "lw": 1})
    ax.annotate("", xy=(-3, -0.60), xytext=(-3, -1),
                arrowprops={"arrowstyle": "->", "color": GRAY, "lw": 1})
    ax.text(-3.14, -1.14, "Palm frame", ha="center", fontsize=13)
    ax.text(-1.0, -1.39, r"$t=c-sd-Rg$", ha="center", fontsize=21)
    ax.add_patch(Arc((-1.45, 0), 0.23, 0.72, theta1=30, theta2=310, color=RED, lw=1))
    direct_label(ax, r"Roll about $d$", (-1.36, -0.19), (-0.9, -0.86), RED)
    ax.set(xlim=(-4.05, 0.9), ylim=(-1.72, 1.44), aspect="equal")
    ax.axis("off")
    save(fig, "approach-geometry")


if __name__ == "__main__":
    wrench_capacity()
    approach_geometry()
