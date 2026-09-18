import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LOG_FILE = "touch_map_log_2.csv"
OUT_FILE = "touch_map_2.png"

# how tall a vertical range to show, in meters, so scale stays honest
# and comparable between runs (real-world proportions, not auto-zoomed)
VERTICAL_RANGE = 0.15

xs, ys, zs = [], [], []
with open(LOG_FILE, newline="") as f:
    for row in csv.DictReader(f):
        if row["contacted"] != "1":
            continue
        xs.append(float(row["grid_x"]))
        ys.append(float(row["grid_y"]))
        zs.append(float(row["contact_z"]))

print(f"loaded {len(zs)} contact points")

x_range = max(xs) - min(xs)
y_range = max(ys) - min(ys)
z_center = (max(zs) + min(zs)) / 2
z_lo = z_center - VERTICAL_RANGE / 2
z_hi = z_center + VERTICAL_RANGE / 2

fig = plt.figure(figsize=(14, 6))

ax1 = fig.add_subplot(121, projection="3d")
p1 = ax1.scatter(xs, ys, zs, c=zs, cmap="viridis", s=120)
ax1.set_xlabel("x (m)")
ax1.set_ylabel("y (m)")
ax1.set_zlabel("contact height (m)")
ax1.set_title(f"Contact points ({len(zs)})")
ax1.set_zlim(z_lo, z_hi)
ax1.set_box_aspect((x_range, y_range, VERTICAL_RANGE))
fig.colorbar(p1, ax=ax1, label="height", shrink=0.6)

unique_x = sorted(set(xs))
unique_y = sorted(set(ys))
expected = len(unique_x) * len(unique_y)

if len(zs) == expected and expected > 0:
    height_at = {}
    for x, y, z in zip(xs, ys, zs):
        height_at[(x, y)] = z

    grid_z = np.array([[height_at[(x, y)] for x in unique_x] for y in unique_y])
    grid_x, grid_y = np.meshgrid(unique_x, unique_y)

    ax2 = fig.add_subplot(122, projection="3d")
    surf = ax2.plot_surface(grid_x, grid_y, grid_z, cmap="viridis",
                             edgecolor="k", linewidth=0.3, antialiased=True)
    ax2.set_xlabel("x (m)")
    ax2.set_ylabel("y (m)")
    ax2.set_zlabel("contact height (m)")
    ax2.set_title("Reconstructed surface (true scale)")
    ax2.set_zlim(z_lo, z_hi)
    ax2.set_box_aspect((x_range, y_range, VERTICAL_RANGE))
    fig.colorbar(surf, ax=ax2, label="height", shrink=0.6)
else:
    print(f"WARNING: {len(zs)} contacts but expected {expected}")

plt.tight_layout()
plt.savefig(OUT_FILE, dpi=150)
print(f"wrote {OUT_FILE}")
