#!/usr/bin/python3
"""
3D Event Accumulation — visualise (x, y, t) event point clouds from a rosbag.

Modes:
  1) 3D scatter      — each event as a point in (x, y, t), coloured by polarity
  2) Voxel density   — 3D voxel grid, opacity = event count per voxel
  3) Time slices     — grid of 2D frames across equal time windows
"""
import glob
import os
import sys
import signal

import numpy as np
import rosbag

SCRIPT_DIR     = os.path.dirname(os.path.abspath(__file__))
RECORDINGS_DIR = os.path.join(SCRIPT_DIR, "recordings")
DEFAULT_TOPIC  = "/laser_event_processing/events_filtered"

# ---------------------------------------------------------------------------
#  DVXplorer Micro — fixed sensor constants
# ---------------------------------------------------------------------------
PIXEL_PITCH_MM  = 0.009          # 9 µm pixel pitch
SENSOR_W        = 640
SENSOR_H        = 480
CX_SENSOR       = SENSOR_W / 2   # 320 px  (principal point, full sensor)
CY_SENSOR       = SENSOR_H / 2   # 240 px
LASER_DEPTH_MM  = 60.0           # distance from camera to laser plane (mm)
PHYSICAL_Y_MM   = 50.0           # physical height of full sensor at laser plane (mm)
PX_TO_MM        = PHYSICAL_Y_MM / SENSOR_H   # 50 / 480 ≈ 0.1042 mm/px

# Crop offsets — must match CROP_X_MIN / CROP_Y_MIN in main.py
# (shift_coordinates=True shifts events to 0-origin, so we add these back)
CROP_X_MIN      = 220
CROP_Y_MIN      = 0

# ---------------------------------------------------------------------------
#  Surface reconstruction parameters
# ---------------------------------------------------------------------------
SURFACE_Y_BINS     = 100    # grid resolution along Y (height, mm)
SURFACE_Z_BINS     = 200    # grid resolution along Z (scan direction, mm)
SURFACE_MAD_THRESH = 3.0    # reject x values beyond N × MAD from median per cell

# ---------------------------------------------------------------------------
#  Shared helpers
# ---------------------------------------------------------------------------
def find_bags():
    return sorted(glob.glob(os.path.join(RECORDINGS_DIR, "*.bag")))


def pick_bag():
    bags = find_bags()
    if not bags:
        print(f"No .bag files found in {RECORDINGS_DIR}")
        return None
    print("\nAvailable recordings:")
    for i, b in enumerate(bags, 1):
        print(f"  {i}) {os.path.basename(b)}  ({os.path.getsize(b)/1e6:.1f} MB)")
    print(f"  {len(bags)+1}) Enter path manually")
    choice = input(f"\nSelect bag [1-{len(bags)+1}] (Enter = latest): ").strip()
    if choice == "":
        return os.path.abspath(max(bags, key=os.path.getmtime))
    if choice.isdigit():
        idx = int(choice)
        if idx == len(bags) + 1:
            p = input("Path: ").strip()
            return os.path.abspath(p) if os.path.exists(p) else None
        if 1 <= idx <= len(bags):
            return os.path.abspath(bags[idx - 1])
    print("Invalid."); return None


def pick_topic(bag_path):
    with rosbag.Bag(bag_path, "r") as bag:
        topics = sorted(bag.get_type_and_topic_info().topics.keys())
    if not topics:
        return DEFAULT_TOPIC
    print("\nTopics:")
    for i, t in enumerate(topics, 1):
        print(f"  {i}) {t}")
    c = input(f"Select topic (Enter = {DEFAULT_TOPIC}): ").strip()
    if c == "":
        return DEFAULT_TOPIC
    if c.isdigit() and 1 <= int(c) <= len(topics):
        return topics[int(c) - 1]
    return DEFAULT_TOPIC


def detect_msg_type(bag_path, topic):
    """Returns 'spatial' if EventSpatialArray, 'events' if EventArray."""
    with rosbag.Bag(bag_path, "r") as bag:
        info = bag.get_type_and_topic_info().topics
        if topic in info:
            return 'spatial' if 'EventSpatial' in info[topic].msg_type else 'events'
    return 'events'




def load_events(bag_path, topic, max_events=None):
    """
    Returns (x, y, third, polarity, axis_label) where:
      - For EventArray:        third = time in ms from first event, x/y in pixels
      - For EventSpatialArray: third = z in mm, x/y in mm (if focal length given)
    """
    msg_type = detect_msg_type(bag_path, topic)
    is_spatial = (msg_type == 'spatial')

    px_to_mm = PX_TO_MM

    xs, ys, thirds, pols = [], [], [], []
    total = 0

    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            if not msg.events:
                continue
            n = len(msg.events)
            if max_events is not None:
                n = min(n, max_events - total)

            ev = msg.events[:n]
            xs.append(np.fromiter((e.x for e in ev), np.uint16, n))
            ys.append(np.fromiter((e.y for e in ev), np.uint16, n))
            pols.append(np.fromiter((e.polarity for e in ev), bool, n))

            if is_spatial:
                thirds.append(np.fromiter((e.z for e in ev), np.float32, n))
            else:
                thirds.append(np.fromiter(
                    (e.ts.secs * 1_000_000 + e.ts.nsecs // 1_000 for e in ev),
                    np.int64, n))

            total += n
            if max_events and total >= max_events:
                break

    if not xs:
        return None

    x_px = np.concatenate(xs).astype(np.float32)
    y_px = np.concatenate(ys).astype(np.float32)
    pol  = np.concatenate(pols)

    # convert x/y pixels → mm
    # x_orig / y_orig restore the full-sensor coordinates (undo crop shift)
    # no centering — 0 px maps to 0 mm, full sensor maps to physical size
    if px_to_mm is not None:
        x = (x_px + CROP_X_MIN) * px_to_mm
        y = (y_px + CROP_Y_MIN) * px_to_mm
        xy_label = "mm"
    else:
        x, y     = x_px, y_px
        xy_label = "px"

    if is_spatial:
        z_m  = np.concatenate(thirds).astype(np.float32)
        third = z_m * 1000.0   # metres → mm
        axis_label = "Z (mm)"
        print(f"Loaded {len(x):,} spatial events  |  "
              f"z: {third.min():.2f} – {third.max():.2f} mm  |  xy in {xy_label}")
    else:
        t     = np.concatenate(thirds).astype(np.int64)
        third = (t - t[0]) / 1000.0
        axis_label = "Time (ms)"
        print(f"Loaded {len(x):,} events  |  duration: {third[-1]:.1f} ms  |  xy in {xy_label}")

    return x, y, third, pol, axis_label


def ask_int(prompt, default):
    s = input(f"{prompt} (Enter = {default}): ").strip()
    return int(s) if s.lstrip("-").isdigit() else default


# ---------------------------------------------------------------------------
#  Mode 1 — 3D scatter (x, y, t)
# ---------------------------------------------------------------------------
def mode_scatter(x, y, t_ms, pol, bag_name, topic, axis_label="Time (ms)"):
    import plotly.graph_objects as go

    MAX_SCATTER = 500_000
    if len(x) > MAX_SCATTER:
        print(f"Downsampling to {MAX_SCATTER:,} events for scatter plot...")
        idx = np.random.choice(len(x), MAX_SCATTER, replace=False)
        idx.sort()
        x, y, t_ms, pol = x[idx], y[idx], t_ms[idx], pol[idx]

    pos_mask = pol
    neg_mask = ~pol

    traces = []
    if pos_mask.any():
        traces.append(go.Scatter3d(
            x=x[pos_mask], y=t_ms[pos_mask], z=y[pos_mask],
            mode="markers",
            marker=dict(size=1, color="red", opacity=0.5),
            name="Positive",
        ))
    if neg_mask.any():
        traces.append(go.Scatter3d(
            x=x[neg_mask], y=t_ms[neg_mask], z=y[neg_mask],
            mode="markers",
            marker=dict(size=1, color="blue", opacity=0.5),
            name="Negative",
        ))

    fig = go.Figure(traces)
    fig.update_layout(
        title=f"3D Event Scatter — {bag_name}<br><sup>{topic}</sup>",
        scene=dict(
            xaxis_title="X (mm)",
            yaxis_title=axis_label,
            zaxis_title="Y (mm)",
            aspectmode="data" if "mm" in axis_label else "manual",
            aspectratio=None if "mm" in axis_label else dict(x=1, y=2, z=1),
        ),
        legend=dict(itemsizing="constant"),
    )
    out = os.path.join(RECORDINGS_DIR, "event_scatter_3d.html")
    fig.write_html(out)
    print(f"Saved: {out}")
    fig.show()


# ---------------------------------------------------------------------------
#  Mode 2 — Voxel density
# ---------------------------------------------------------------------------
def mode_voxel(x, y, t_ms, pol, bag_name, topic, axis_label="Time (ms)"):
    import plotly.graph_objects as go

    n_t   = ask_int("Number of time bins",  50)
    n_xy  = ask_int("Spatial bin size (px)", 2)

    pol_choice = input("Polarity [all / pos / neg] (Enter = all): ").strip().lower()
    if pol_choice == "pos":
        mask = pol
    elif pol_choice == "neg":
        mask = ~pol
    else:
        mask = np.ones(len(x), bool)

    xf, yf, tf = x[mask], y[mask], t_ms[mask]

    xb = xf // n_xy
    yb = yf // n_xy
    tb = (tf / tf.max() * (n_t - 1)).astype(np.int32)

    # accumulate into voxel grid
    nx = int(xb.max()) + 1
    ny = int(yb.max()) + 1

    counts = np.zeros((nx, ny, n_t), dtype=np.uint32)
    np.add.at(counts, (xb, yb, tb), 1)

    xi, yi, ti = np.nonzero(counts)
    vals = counts[xi, yi, ti].astype(float)

    # map voxel indices back to real coordinates
    xr = xi * n_xy + n_xy / 2
    yr = yi * n_xy + n_xy / 2
    tr = (ti / (n_t - 1)) * t_ms.max()

    fig = go.Figure(go.Scatter3d(
        x=xr, y=tr, z=yr,
        mode="markers",
        marker=dict(
            size=3,
            color=vals,
            colorscale="Inferno",
            colorbar=dict(title="Events/voxel"),
            opacity=0.6,
            cmin=vals.min(),
            cmax=np.percentile(vals, 98),
        ),
        text=[f"{int(v)} events" for v in vals],
        hovertemplate="x=%{x:.0f}  t=%{y:.2f}ms  y=%{z:.0f}<br>%{text}<extra></extra>",
    ))
    fig.update_layout(
        title=f"3D Voxel Density — {bag_name}<br><sup>{topic}  |  spatial bin={n_xy}px  t-bins={n_t}</sup>",
        scene=dict(
            xaxis_title="X (mm)",
            yaxis_title=axis_label,
            zaxis_title="Y (mm)",
            aspectmode="data" if "mm" in axis_label else "manual",
            aspectratio=None if "mm" in axis_label else dict(x=1, y=2, z=1),
        ),
    )
    out = os.path.join(RECORDINGS_DIR, "event_voxel_3d.html")
    fig.write_html(out)
    print(f"Saved: {out}")
    fig.show()


# ---------------------------------------------------------------------------
#  Mode 3 — Time slices (matplotlib grid)
# ---------------------------------------------------------------------------
def mode_slices(x, y, t_ms, pol, bag_name, topic):
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec

    n_slices = ask_int("Number of time slices", 12)
    pol_choice = input("Polarity [all / pos / neg] (Enter = all): ").strip().lower()
    if pol_choice == "pos":
        mask = pol
    elif pol_choice == "neg":
        mask = ~pol
    else:
        mask = np.ones(len(x), bool)

    xf, yf, tf = x[mask], y[mask], t_ms[mask]

    w = int(xf.max()) + 1
    h = int(yf.max()) + 1

    t_min, t_max = tf.min(), tf.max()
    edges = np.linspace(t_min, t_max, n_slices + 1)

    cols = min(4, n_slices)
    rows = (n_slices + cols - 1) // cols
    fig = plt.figure(figsize=(cols * 4, rows * 3))
    gs  = gridspec.GridSpec(rows, cols, figure=fig, hspace=0.4, wspace=0.3)

    for i in range(n_slices):
        t0, t1 = edges[i], edges[i + 1]
        in_slice = (tf >= t0) & (tf < t1)
        xs_s, ys_s = xf[in_slice], yf[in_slice]

        frame = np.zeros((h, w), dtype=np.uint32)
        if len(xs_s):
            valid = (xs_s < w) & (ys_s < h)
            np.add.at(frame, (ys_s[valid], xs_s[valid]), 1)

        ax = fig.add_subplot(gs[i // cols, i % cols])
        ax.imshow(frame, cmap="inferno", interpolation="nearest", aspect="auto",
                  origin="upper")
        ax.set_title(f"{t0:.1f} – {t1:.1f} ms\n({in_slice.sum():,} events)", fontsize=8)
        ax.axis("off")

    fig.suptitle(f"Time Slices — {bag_name}\n{topic}", fontsize=10)

    out = os.path.join(RECORDINGS_DIR, "event_slices.png")
    plt.savefig(out, dpi=150, bbox_inches="tight")
    print(f"Saved: {out}")
    plt.show()


# ---------------------------------------------------------------------------
#  Mode 4 — Surface reconstruction
# ---------------------------------------------------------------------------
def mode_surface(x, y, t_ms, pol, bag_name, topic, axis_label="Z (mm)"):
    import plotly.graph_objects as go

    z = t_ms  # scanning direction in mm (from spatial topic)

    # bin edges
    y_edges = np.linspace(y.min(), y.max(), SURFACE_Y_BINS + 1)
    z_edges = np.linspace(z.min(), z.max(), SURFACE_Z_BINS + 1)

    surface = np.full((SURFACE_Y_BINS, SURFACE_Z_BINS), np.nan)

    for iy in range(SURFACE_Y_BINS):
        y_mask = (y >= y_edges[iy]) & (y < y_edges[iy + 1])
        for iz in range(SURFACE_Z_BINS):
            z_mask = (z >= z_edges[iz]) & (z < z_edges[iz + 1])
            pts = x[y_mask & z_mask]
            if len(pts) < 2:
                continue

            # MAD outlier removal
            med  = np.median(pts)
            mad  = np.median(np.abs(pts - med))
            if mad > 0:
                pts = pts[np.abs(pts - med) <= SURFACE_MAD_THRESH * mad]
            if len(pts) == 0:
                continue

            surface[iy, iz] = np.median(pts)

    y_centers = (y_edges[:-1] + y_edges[1:]) / 2
    z_centers = (z_edges[:-1] + z_edges[1:]) / 2
    Z_grid, Y_grid = np.meshgrid(z_centers, y_centers)

    valid = np.sum(~np.isnan(surface))
    print(f"Surface grid: {SURFACE_Y_BINS}×{SURFACE_Z_BINS}  |  "
          f"{valid}/{SURFACE_Y_BINS*SURFACE_Z_BINS} cells filled  |  "
          f"x range: {np.nanmin(surface):.2f} – {np.nanmax(surface):.2f} mm")

    fig = go.Figure(go.Surface(
        x=Z_grid,
        y=Y_grid,
        z=surface,
        colorscale="Viridis",
        colorbar=dict(title="X (mm)<br>depth"),
        connectgaps=False,
    ))
    fig.update_layout(
        title=f"Surface Reconstruction — {bag_name}<br><sup>{topic}</sup>",
        scene=dict(
            xaxis_title="Z — scan direction (mm)",
            yaxis_title="Y — height (mm)",
            zaxis_title="X — depth (mm)",
            aspectmode="data",
        ),
    )
    out = os.path.join(RECORDINGS_DIR, "event_surface.html")
    fig.write_html(out)
    print(f"Saved: {out}")
    fig.show()


# ---------------------------------------------------------------------------
#  Main
# ---------------------------------------------------------------------------
MODES = {
    "1": ("3D scatter    — (x, y, t) point cloud coloured by polarity  [plotly]", mode_scatter),
    "2": ("Voxel density — 3D binned event density coloured by count   [plotly]", mode_voxel),
    "3": ("Time slices   — grid of 2D frames across time windows        [matplotlib]", mode_slices),
    "4": ("Surface       — 2D surface from median x per (y,z) cell      [plotly]", mode_surface),
}

def main():
    signal.signal(signal.SIGINT, signal.default_int_handler)

    print("=" * 60)
    print("   3D Event Accumulation")
    print("=" * 60)
    for k, (label, _) in MODES.items():
        print(f"  {k}) {label}")
    print("=" * 60)

    choice = input("\nSelect mode [1-4]: ").strip()
    if choice not in MODES:
        print(f"Invalid: '{choice}'"); return 1

    bag_path = pick_bag()
    if not bag_path:
        return 1
    topic = pick_topic(bag_path)

    max_str = input("Max events to load (Enter = all): ").strip()
    max_events = int(max_str) if max_str.isdigit() else None

    result = load_events(bag_path, topic, max_events)
    if result is None:
        print("No events found."); return 1

    x, y, t_ms, pol, axis_label = result
    bag_name = os.path.basename(bag_path)

    try:
        if choice == "3":
            MODES[choice][1](x, y, t_ms, pol, bag_name, topic)
        else:  # modes 1, 2, 4
            MODES[choice][1](x, y, t_ms, pol, bag_name, topic, axis_label)
    except KeyboardInterrupt:
        print("\nInterrupted.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
