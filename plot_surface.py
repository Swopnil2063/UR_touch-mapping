import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RESULT_FILE = "contact_results.csv"
OUTPUT_IMAGE = "touch_map_3d.png"

rows = []
with open(RESULT_FILE, newline="") as f:
    for r in csv.DictReader(f):
        rows.append({
            "gx": float(r["grid_x"]),
            "gy": float(r["grid_y"]),
            "ax": float(r["actual_x"]),
            "ay": float(r["actual_y"]),
            "z": float(r["contact_z"]),
        })

xs = sorted(set(r["gx"] for r in rows))
ys = sorted(set(r["gy"] for r in rows))

Z = np.full((len(ys), len(xs)), np.nan)
for r in rows:
    i = ys.index(r["gy"])
    j = xs.index(r["gx"])
    Z[i, j] = r["z"]

X, Y = np.meshgrid(xs, ys)

fig = plt.figure(figsize=(13, 6))

ax1 = fig.add_subplot(121, projection="3d")
ax1.plot_surface(X, Y, Z, cmap="viridis", edgecolor="k", linewidth=0.4, alpha=0.9)
ax1.set_xlabel("x (m)")
ax1.set_ylabel("y (m)")
ax1.set_zlabel("height (m)")
ax1.set_title("reconstructed surface (4x4)")
ax1.view_init(elev=35, azim=-120)

ax2 = fig.add_subplot(122, projection="3d")
for r in rows:
    ax2.bar3d(r["gx"] - 0.045, r["gy"] - 0.045, 0,
              0.09, 0.09, max(r["z"], 0.001),
              shade=True, alpha=0.85)
ax2.set_xlabel("x (m)")
ax2.set_ylabel("y (m)")
ax2.set_zlabel("height (m)")
ax2.set_title("voxel view")
ax2.view_init(elev=30, azim=-120)

plt.tight_layout()
plt.savefig(OUTPUT_IMAGE, dpi=130)
print(f"saved to {OUTPUT_IMAGE}")
