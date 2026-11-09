import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import numpy as np
from pathlib import Path
import time

base = Path(__file__).resolve().parent.parent
wall_map_path   = base / "src/wall_map.csv"
particles_path  = base / "src/particles.csv"
pot_field_path  = base / "src/pot_field.csv"
path_path       = base / "src/path.csv"
gold_path       = base / "src/gold_locations.csv"
goal_path       = base / "src/goal.csv"
attractive_path = base / "src/attractive.csv"
# subgoals_path   = base / "subgoals.csv"

mini_cell_length = 0.08
scale = 10


def load_wall_map():
    """Load wall_map.csv. Returns a 2D array, or None if not ready/valid yet."""
    try:
        data = np.loadtxt(wall_map_path, delimiter=",")
    except Exception:
        return None
    if data.ndim != 2:
        return None
    return data


# ---- initial wait for wall map (maze size/layout is read from the file itself) ----
wall_map = None
while wall_map is None:
    wall_map = load_wall_map()
    if wall_map is None:
        print("Waiting for wall_map.csv to be populated...")
        time.sleep(1.0)

rows, cols = wall_map.shape
last_mtime_wall_map = wall_map_path.stat().st_mtime

plt.ion()
fig = plt.figure(figsize=(10, 9))
ax = fig.add_subplot(111)

last_mtime_particles = 0
last_mtime_pot_field = 0
cached_pf_data = None


def load_pot_field():
    """Load pot_field.csv and return quiver data."""
    try:
        data = np.loadtxt(pot_field_path, delimiter=",", skiprows=1)
    except Exception:
        return None
    if data.ndim == 1:
        data = data[np.newaxis, :]

    # columns: row, col, gx0, gy0
    r_idx = data[:, 0].astype(int)
    c_idx = data[:, 1].astype(int)
    gx0, gy0 = data[:, 2], data[:, 3]

    total_mag = np.hypot(gx0, gy0)

    mask = total_mag > 1e-6
    return {
        "c": c_idx[mask],
        "r": r_idx[mask],
        "u": gx0[mask],
        "v": -gy0[mask],
        "mag": total_mag[mask],
    }


while plt.fignum_exists(fig.number):
    # ---- check wall map file (the maze can be a different size/layout between runs) ----
    try:
        wm_mtime = wall_map_path.stat().st_mtime
        if wm_mtime != last_mtime_wall_map:
            new_wall_map = load_wall_map()
            if new_wall_map is not None:
                wall_map = new_wall_map
                rows, cols = wall_map.shape
                last_mtime_wall_map = wm_mtime
    except FileNotFoundError:
        pass

    # ---- check particle file ----
    try:
        p_mtime = particles_path.stat().st_mtime
    except FileNotFoundError:
        plt.pause(0.5)
        continue

    # ---- check potential field file (reload only when it changes) ----
    try:
        pf_mtime = pot_field_path.stat().st_mtime
        if pf_mtime != last_mtime_pot_field:
            cached_pf_data = load_pot_field()
            last_mtime_pot_field = pf_mtime
    except FileNotFoundError:
        cached_pf_data = None

    if p_mtime == last_mtime_particles:
        plt.pause(0.3)
        continue
    last_mtime_particles = p_mtime

    try:
        pdata = np.loadtxt(particles_path, delimiter=",", skiprows=1)
    except Exception:
        plt.pause(0.2)
        continue
    if pdata.ndim == 1:
        pdata = pdata[np.newaxis, :]

    mini_cols   = pdata[:, 0] / mini_cell_length
    mini_rows   = pdata[:, 1] / mini_cell_length
    weights     = pdata[:, 3]
    w_norm      = (weights - weights.min()) / (weights.max() - weights.min() + 1e-9)
    sizes       = 5 + w_norm * 30

    ax.cla()
    ax.imshow(wall_map, cmap="binary", interpolation="nearest", zorder=0)

    # grid lines at big-cell boundaries (uses the *current* wall_map's rows/cols,
    # so this automatically adapts to any maze size, e.g. 3x3, 6x6, ...)
    for x in range(0, cols + 1, scale):
        ax.axvline(x - 0.5, color="red", linewidth=0.5, alpha=0.4, zorder=1)
    for y in range(0, rows + 1, scale):
        ax.axhline(y - 0.5, color="red", linewidth=0.5, alpha=0.4, zorder=1)

    # ---- potential field arrows ----
    if cached_pf_data is not None:
        pf = cached_pf_data

        # Normalize arrow length to 0.45 of a mini-cell so they fit neatly;
        # keep direction, discard magnitude
        mag = pf["mag"]
        u_hat = pf["u"] / (mag + 1e-9) * 0.75
        v_hat = pf["v"] / (mag + 1e-9) * 0.75

        ax.quiver(
            pf["c"], pf["r"],       # origin: col=x, row=y in image coords
            u_hat, v_hat,
            color="red",
            angles="xy",
            scale_units="xy",
            scale=1,                # 1 unit = 1 mini-cell pixel
            width=0.003,            # thin shaft
            headwidth=3,
            headlength=4,
            headaxislength=3.5,
            alpha=0.75,
            zorder=2,
        )

    # ---- planned path (mini-cell indices: col=x, row=y) ----
    if path_path.exists():
        try:
            path_data = np.loadtxt(path_path, delimiter=",", skiprows=1)

            if path_data.ndim == 1:
                path_data = path_data[np.newaxis, :]

            path_rows = path_data[:, 0]
            path_cols = path_data[:, 1]

            # Dijkstra outputs BIG CELL coordinates.
            # Place marker at centre of each big cell.
            path_x = path_cols * scale + scale / 2 - 0.5
            path_y = path_rows * scale + scale / 2 - 0.5

            ax.plot(
                path_x,
                path_y,
                color="pink",
                linewidth=2,
                marker="o",
                markersize=4,
                label="path",
                zorder=4,
            )

        except Exception as e:
            print(f"Failed to load path.csv: {e}")
    
    # ---- attractive vector toward next waypoint (world meters -> mini-cell pixels) ----
    if attractive_path.exists():
        try:
            attr_data = np.loadtxt(attractive_path, delimiter=",", skiprows=1)
            if attr_data.ndim == 1 and attr_data.size == 4:
                wx, wy, au, av = attr_data
                px = wx / mini_cell_length
                py = wy / mini_cell_length
                ax.quiver(
                    [px], [py],
                    [au], [av],
                    color="blue",
                    angles="xy",
                    scale_units="xy",
                    scale=0.2,        # unit vector -> ~5 mini-cells long
                    width=0.006,
                    headwidth=4,
                    headlength=5,
                    headaxislength=4.5,
                    label="attractive",
                    zorder=7,
                )
        except Exception as e:
            print(f"Failed to load attractive.csv: {e}")
        
    # ---- gold locations (big-cell coordinates: row, col) ----
    if gold_path.exists():
        try:
            gold_data = np.loadtxt(gold_path, delimiter=",", skiprows=1)
            if gold_data.ndim == 1:
                gold_data = gold_data[np.newaxis, :]
            if gold_data.size > 0:
                gold_rows = gold_data[:, 0]
                gold_cols = gold_data[:, 1]
                gold_x = gold_cols * scale + scale / 2 - 0.5
                gold_y = gold_rows * scale + scale / 2 - 0.5
                ax.scatter(
                    gold_x, gold_y,
                    s=160, c="gold", marker="*", edgecolors="black",
                    linewidths=1.0, label="gold", zorder=5,
                )
        except Exception as e:
            print(f"Failed to load gold_locations.csv: {e}")

    # ---- final goal (big-cell coordinates: row, col) ----
    if goal_path.exists():
        try:
            goal_data = np.loadtxt(goal_path, delimiter=",", skiprows=1)
            if goal_data.ndim == 1 and goal_data.size > 0:
                goal_row, goal_col = goal_data[0], goal_data[1]
                goal_x = goal_col * scale + scale / 2 - 0.5
                goal_y = goal_row * scale + scale / 2 - 0.5
                ax.scatter(
                    goal_x, goal_y,
                    s=220, c="lime", marker="X", edgecolors="black",
                    linewidths=1.5, label="goal", zorder=6,
                )
        except Exception as e:
            print(f"Failed to load goal.csv: {e}")

    # ---- subgoals (mini-cell indices: col=x, row=y) ----
    # if subgoals_path.exists():
    #     try:
    #         sg_data = np.loadtxt(subgoals_path, delimiter=",")
    #         if sg_data.ndim == 1:
    #             sg_data = sg_data[np.newaxis, :]
    #         ax.scatter(sg_data[:, 0], sg_data[:, 1],
    #                    s=140, c="black", marker="x", linewidths=2.5,
    #                    label="subgoals", zorder=5)
    #     except Exception:
    #         pass

    # ---- particles ----
    ax.scatter(mini_cols, mini_rows, s=sizes, c=w_norm,
               cmap="plasma", alpha=0.85, linewidths=0,
               vmin=0, vmax=1, zorder=3)

    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(f"Particles: {len(pdata)}  |  {time.strftime('%H:%M:%S')}")

    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(loc="upper right", fontsize=8)

    fig.canvas.draw()
    plt.pause(0.3)
