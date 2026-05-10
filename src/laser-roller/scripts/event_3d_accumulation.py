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


def load_events(bag_path, topic, max_events=None):
    """
    Returns arrays: x, y, t_us (microseconds from first event), polarity.
    Reads at most max_events events.
    """
    xs, ys, ts, pols = [], [], [], []
    total = 0

    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            if not msg.events:
                continue
            n = len(msg.events)
            if max_events is not None:
                n = min(n, max_events - total)

            xs.append(np.fromiter((e.x for e in msg.events[:n]),        np.uint16, n))
            ys.append(np.fromiter((e.y for e in msg.events[:n]),        np.uint16, n))
            ts.append(np.fromiter(
                (e.ts.secs * 1_000_000 + e.ts.nsecs // 1_000 for e in msg.events[:n]),
                np.int64, n))
            pols.append(np.fromiter((e.polarity for e in msg.events[:n]), bool, n))

            total += n
            if max_events and total >= max_events:
                break

    if not xs:
        return None

    x   = np.concatenate(xs).astype(np.int32)
    y   = np.concatenate(ys).astype(np.int32)
    t   = np.concatenate(ts).astype(np.int64)
    pol = np.concatenate(pols)

    # normalise time to ms from first event
    t_ms = (t - t[0]) / 1000.0

    print(f"Loaded {len(x):,} events  |  duration: {t_ms[-1]:.1f} ms")
    return x, y, t_ms, pol


def ask_int(prompt, default):
    s = input(f"{prompt} (Enter = {default}): ").strip()
    return int(s) if s.lstrip("-").isdigit() else default


def ask_float(prompt, default):
    s = input(f"{prompt} (Enter = {default}): ").strip()
    try:
        return float(s)
    except ValueError:
        return default


# ---------------------------------------------------------------------------
#  Mode 1 — 3D scatter (x, y, t)
# ---------------------------------------------------------------------------
def mode_scatter(x, y, t_ms, pol, bag_name, topic):
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
            xaxis_title="X (px)",
            yaxis_title="Time (ms)",
            zaxis_title="Y (px)",
            aspectmode="manual",
            aspectratio=dict(x=1, y=2, z=1),
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
def mode_voxel(x, y, t_ms, pol, bag_name, topic):
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

    # normalise opacity
    vals_norm = (vals - vals.min()) / (vals.max() - vals.min() + 1e-9)

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
            xaxis_title="X (px)",
            yaxis_title="Time (ms)",
            zaxis_title="Y (px)",
            aspectmode="manual",
            aspectratio=dict(x=1, y=2, z=1),
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
#  Main
# ---------------------------------------------------------------------------
MODES = {
    "1": ("3D scatter    — (x, y, t) point cloud coloured by polarity  [plotly]", mode_scatter),
    "2": ("Voxel density — 3D binned event density coloured by count   [plotly]", mode_voxel),
    "3": ("Time slices   — grid of 2D frames across time windows        [matplotlib]", mode_slices),
}

def main():
    signal.signal(signal.SIGINT, signal.default_int_handler)

    print("=" * 60)
    print("   3D Event Accumulation")
    print("=" * 60)
    for k, (label, _) in MODES.items():
        print(f"  {k}) {label}")
    print("=" * 60)

    choice = input("\nSelect mode [1-3]: ").strip()
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

    x, y, t_ms, pol = result
    bag_name = os.path.basename(bag_path)

    try:
        MODES[choice][1](x, y, t_ms, pol, bag_name, topic)
    except KeyboardInterrupt:
        print("\nInterrupted.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
