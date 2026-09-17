import csv
import matplotlib
matplotlib.use("Agg")          # save to file instead of opening a window
import matplotlib.pyplot as plt

LOG_FILE = "touch_map_log.csv"
OUT_FILE = "touch_map.png"

xs, ys, zs = [], [], []
with open(LOG_FILE, newline="") as f:
    for row in csv.DictReader(f):
        if row["contacted"] != "1":
            continue
        xs.append(float(row["grid_x"]))
        ys.append(float(row["grid_y"]))
        zs.append(float(row["contact_z"]))

fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection="3d")
p = ax.scatter(xs, ys, zs, c=zs, cmap="viridis", s=120)
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_zlabel("contact height (m)")
ax.set_title(f"Touch map - {len(zs)} contact points")
fig.colorbar(p, label="height")
plt.savefig(OUT_FILE, dpi=150)
print(f"wrote {OUT_FILE} ({len(zs)} points)")
