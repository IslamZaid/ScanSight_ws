#!/usr/bin/python3
"""
Bag Tools — combined post-processing utility for laser-roller recordings.

  1) Inspect bag       — topics, message counts, frequencies
  2) Play events       — OpenCV real-time viewer
  3) Export to array   — save events as HDF5 or NPZ
  4) Height map        — 2D event-count visualisation
"""
import glob
import os
import sys
import signal
from datetime import datetime

import numpy as np
import rosbag
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm

# ---------------------------------------------------------------------------
#  Shared config
# ---------------------------------------------------------------------------
SCRIPT_DIR      = os.path.dirname(os.path.abspath(__file__))
RECORDINGS_DIR  = os.path.join(SCRIPT_DIR, "recordings")
DEFAULT_TOPIC   = "/laser_event_processing/events_filtered"

EVENT_DTYPE = np.dtype([
    ("t_ns",    np.uint64),
    ("x",       np.uint16),
    ("y",       np.uint16),
    ("polarity", np.bool_),
])

# ---------------------------------------------------------------------------
#  Shared helpers
# ---------------------------------------------------------------------------
def find_latest_bag(recordings_dir=RECORDINGS_DIR):
    bags = glob.glob(os.path.join(recordings_dir, "*.bag"))
    return max(bags, key=os.path.getmtime) if bags else None


def list_bags(recordings_dir=RECORDINGS_DIR):
    bags = sorted(glob.glob(os.path.join(recordings_dir, "*.bag")))
    return bags


def pick_bag():
    bags = list_bags()
    if not bags:
        print(f"No .bag files found in {RECORDINGS_DIR}")
        return None

    print("\nAvailable recordings:")
    for i, b in enumerate(bags, 1):
        size_mb = os.path.getsize(b) / 1e6
        print(f"  {i}) {os.path.basename(b)}  ({size_mb:.1f} MB)")
    print(f"  {len(bags)+1}) Enter path manually")

    choice = input(f"\nSelect bag [1-{len(bags)+1}] (Enter = latest): ").strip()
    if choice == "":
        return os.path.abspath(find_latest_bag())
    if not choice.isdigit():
        print("Invalid choice.")
        return None
    idx = int(choice)
    if idx == len(bags) + 1:
        path = input("Path: ").strip()
        return os.path.abspath(path) if os.path.exists(path) else None
    if 1 <= idx <= len(bags):
        return os.path.abspath(bags[idx - 1])
    print("Invalid choice.")
    return None


def pick_topic(bag_path):
    with rosbag.Bag(bag_path, "r") as bag:
        topics = sorted(bag.get_type_and_topic_info().topics.keys())
    if not topics:
        return DEFAULT_TOPIC
    print("\nTopics in bag:")
    for i, t in enumerate(topics, 1):
        print(f"  {i}) {t}")
    choice = input(f"Select topic [1-{len(topics)}] (Enter = {DEFAULT_TOPIC}): ").strip()
    if choice == "":
        return DEFAULT_TOPIC
    if choice.isdigit() and 1 <= int(choice) <= len(topics):
        return topics[int(choice) - 1]
    return DEFAULT_TOPIC


# ---------------------------------------------------------------------------
#  1) Inspect bag
# ---------------------------------------------------------------------------
def mode_inspect():
    bag_path = pick_bag()
    if not bag_path:
        return

    with rosbag.Bag(bag_path, "r") as bag:
        info   = bag.get_type_and_topic_info()
        topics = info.topics

        print(f"\nBag:  {bag_path}")
        print(f"Size: {os.path.getsize(bag_path) / 1e6:.2f} MB")
        try:
            t0 = datetime.fromtimestamp(bag.get_start_time()).strftime("%Y-%m-%d %H:%M:%S")
            t1 = datetime.fromtimestamp(bag.get_end_time()).strftime("%Y-%m-%d %H:%M:%S")
            dur = bag.get_end_time() - bag.get_start_time()
            print(f"Start:    {t0}")
            print(f"End:      {t1}")
            print(f"Duration: {dur:.3f} s")
        except rosbag.ROSBagException:
            print("(no messages)")

        if not topics:
            print("\nNo topics.")
            return

        print("\nTopics:")
        for name in sorted(topics):
            ti  = topics[name]
            hz  = f"{ti.frequency:.2f} Hz" if ti.frequency else "unknown"
            print(f"  {name}")
            print(f"    type:     {ti.msg_type}")
            print(f"    messages: {ti.message_count}")
            print(f"    freq:     {hz}")


# ---------------------------------------------------------------------------
#  2) Play events (OpenCV)
# ---------------------------------------------------------------------------
def mode_play():
    import cv2

    bag_path = pick_bag()
    if not bag_path:
        return
    topic = pick_topic(bag_path)

    print(f"\nPlaying '{topic}' — press Q to quit.")
    cv2.namedWindow("Event Camera", cv2.WINDOW_NORMAL)

    width, height = 0, 0
    frame = None
    msg_count = 0
    BATCH = 100

    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            w = int(msg.width) if msg.width > 0 else width
            h = int(msg.height) if msg.height > 0 else height
            if w != width or h != height:
                width, height = w, h
                frame = np.full((height, width, 3), 128, dtype=np.uint8)

            if frame is None:
                frame = np.full((height or 260, width or 346, 3), 128, dtype=np.uint8)

            if msg.events:
                xs   = np.fromiter((e.x for e in msg.events), dtype=np.uint16)
                ys   = np.fromiter((e.y for e in msg.events), dtype=np.uint16)
                pols = np.fromiter((e.polarity for e in msg.events), dtype=bool)
                valid = (xs < width) & (ys < height)
                xs, ys, pols = xs[valid], ys[valid], pols[valid]
                frame[ys[pols],  xs[pols]]  = 255
                frame[ys[~pols], xs[~pols]] = 0

            msg_count += 1
            if msg_count >= BATCH:
                cv2.imshow("Event Camera", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                frame.fill(128)
                msg_count = 0

    cv2.destroyAllWindows()
    print("Done.")


# ---------------------------------------------------------------------------
#  3) Export events to array (H5 / NPZ)
# ---------------------------------------------------------------------------
def stamp_to_ns(stamp):
    return int(stamp.secs) * 1_000_000_000 + int(stamp.nsecs)


def read_event_chunks(bag_path, topic, max_events=None):
    meta = {"topic": topic, "width": 0, "height": 0,
            "message_batches": 0, "total_events": 0,
            "source_bag": os.path.abspath(bag_path)}

    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            meta["message_batches"] += 1
            meta["width"]  = int(getattr(msg, "width",  meta["width"]))
            meta["height"] = int(getattr(msg, "height", meta["height"]))

            n = len(msg.events)
            arr = np.empty(n, dtype=EVENT_DTYPE)
            if n:
                arr["t_ns"]    = np.fromiter((stamp_to_ns(e.ts) for e in msg.events), np.uint64, n)
                arr["x"]       = np.fromiter((e.x for e in msg.events),               np.uint16, n)
                arr["y"]       = np.fromiter((e.y for e in msg.events),               np.uint16, n)
                arr["polarity"]= np.fromiter((e.polarity for e in msg.events),         np.bool_,  n)

            if max_events is not None:
                remaining = max_events - meta["total_events"]
                arr = arr[:remaining]

            meta["total_events"] += len(arr)
            if len(arr):
                yield arr, meta

            if max_events and meta["total_events"] >= max_events:
                break

    if meta["total_events"] == 0:
        yield np.empty(0, dtype=EVENT_DTYPE), meta


def mode_export():
    bag_path = pick_bag()
    if not bag_path:
        return
    topic = pick_topic(bag_path)

    fmt_choice = input("Format: [1] HDF5  [2] NPZ  (Enter = HDF5): ").strip()
    fmt = "npz" if fmt_choice == "2" else "h5"

    base = os.path.splitext(bag_path)[0]
    default_out = base + (".h5" if fmt == "h5" else ".npz")
    out_input = input(f"Output path (Enter = {default_out}): ").strip()
    output_path = os.path.abspath(out_input) if out_input else default_out

    max_str = input("Max events (Enter = all): ").strip()
    max_events = int(max_str) if max_str.isdigit() else None

    print(f"\nExporting '{topic}' → {output_path} ...")

    if fmt == "h5":
        try:
            import h5py
        except ImportError:
            print("h5py not installed: sudo apt install python3-h5py")
            return

        final_meta = None
        with h5py.File(output_path, "w") as f:
            g = f.create_group("events")
            ds = {k: g.create_dataset(k, shape=(0,), maxshape=(None,), dtype=dt, chunks=True)
                  for k, dt in [("t_ns","u8"),("x","u2"),("y","u2"),("polarity","?")]}

            for arr, meta in read_event_chunks(bag_path, topic, max_events):
                final_meta = meta
                for k in ds:
                    old = ds[k].shape[0]
                    ds[k].resize((old + len(arr),))
                    ds[k][old:] = arr[k]

            if final_meta:
                for k, v in final_meta.items():
                    g.attrs[k] = v
    else:
        chunks, final_meta = [], None
        for arr, meta in read_event_chunks(bag_path, topic, max_events):
            chunks.append(arr)
            final_meta = meta
        events = np.concatenate(chunks) if chunks else np.empty(0, dtype=EVENT_DTYPE)
        np.savez_compressed(output_path, events=events,
                            t_ns=events["t_ns"], x=events["x"],
                            y=events["y"], polarity=events["polarity"])

    if final_meta:
        print(f"Saved: {output_path}")
        print(f"Events: {final_meta['total_events']:,}  |  "
              f"Resolution: {final_meta['width']}x{final_meta['height']}")


# ---------------------------------------------------------------------------
#  4) Height map
# ---------------------------------------------------------------------------
def mode_heightmap():
    bag_path = pick_bag()
    if not bag_path:
        return
    topic = pick_topic(bag_path)

    start_str = input("Start event index (Enter = 0): ").strip()
    end_str   = input("End event index   (Enter = all): ").strip()
    pol_str   = input("Polarity [all / pos / neg] (Enter = all): ").strip().lower()

    start_event = int(start_str) if start_str.isdigit() else 0
    end_event   = int(end_str)   if end_str.lstrip("-").isdigit() else -1
    pol_filter  = pol_str if pol_str in ("pos", "neg") else None

    print(f"\nReading '{topic}' ...")

    # determine dimensions from first message
    width, height = 0, 0
    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            if msg.width > 0 and msg.height > 0:
                width, height = int(msg.width), int(msg.height)
                break

    if width == 0 or height == 0:
        print("Error: could not determine sensor dimensions from bag.")
        return

    print(f"Sensor: {width}x{height}")

    heightmap = np.zeros((height, width), dtype=np.uint64)
    global_idx = acc = 0

    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            if not msg.events:
                continue
            n = len(msg.events)
            b0, b1 = global_idx, global_idx + n
            global_idx += n

            if end_event != -1 and b0 >= end_event:
                break
            if b1 <= start_event:
                continue

            lo = max(0, start_event - b0)
            hi = n if end_event == -1 else min(n, end_event - b0)
            sl = msg.events[lo:hi]
            if not sl:
                continue

            ns = len(sl)
            xs   = np.fromiter((e.x for e in sl), np.uint16, ns)
            ys   = np.fromiter((e.y for e in sl), np.uint16, ns)

            if pol_filter is not None:
                pols = np.fromiter((e.polarity for e in sl), bool, ns)
                mask = pols if pol_filter == "pos" else ~pols
                xs, ys = xs[mask], ys[mask]

            valid = (xs < width) & (ys < height)
            xs, ys = xs[valid], ys[valid]
            np.add.at(heightmap, (ys, xs), 1)
            acc += len(xs)

    print(f"Accumulated {acc:,} events  |  max per pixel: {heightmap.max():,}  |  "
          f"active pixels: {np.count_nonzero(heightmap):,}")

    save_str = input("\nSave figure path (Enter = show only): ").strip()

    fig, axes = plt.subplots(1, 2, figsize=(16, 6))

    ax1 = axes[0]
    im1 = ax1.imshow(heightmap, cmap="inferno", interpolation="nearest", aspect="equal")
    ax1.set_title("Event Height Map (Linear)")
    ax1.set_xlabel("X"); ax1.set_ylabel("Y")
    fig.colorbar(im1, ax=ax1, label="Event count")

    ax2 = axes[1]
    hm_log = heightmap.astype(np.float64)
    hm_log[hm_log == 0] = np.nan
    im2 = ax2.imshow(hm_log, cmap="inferno", interpolation="nearest", aspect="equal",
                     norm=LogNorm(vmin=max(1, np.nanmin(hm_log)), vmax=np.nanmax(hm_log)))
    ax2.set_title("Event Height Map (Log Scale)")
    ax2.set_xlabel("X"); ax2.set_ylabel("Y")
    fig.colorbar(im2, ax=ax2, label="Event count (log)")

    fig.suptitle(f"{os.path.basename(bag_path)}  |  {acc:,} events  |  topic: {topic}",
                 fontsize=9, color="gray")
    plt.tight_layout()

    if save_str:
        plt.savefig(os.path.abspath(save_str), dpi=200, bbox_inches="tight")
        print(f"Saved: {save_str}")

    plt.show()


# ---------------------------------------------------------------------------
#  Main menu
# ---------------------------------------------------------------------------
MODES = {
    "1": ("Inspect bag       (topics, counts, frequencies)", mode_inspect),
    "2": ("Play events       (OpenCV viewer)",               mode_play),
    "3": ("Export to array   (HDF5 / NPZ)",                  mode_export),
    "4": ("Height map        (2D event-count visualisation)", mode_heightmap),
}

def main():
    signal.signal(signal.SIGINT, signal.default_int_handler)

    print("=" * 52)
    print("   Laser-Roller Bag Tools")
    print("=" * 52)
    for key, (label, _) in MODES.items():
        print(f"  {key}) {label}")
    print("=" * 52)

    choice = input("\nSelect mode [1-4]: ").strip()
    if choice not in MODES:
        print(f"Invalid choice: '{choice}'")
        return 1

    try:
        MODES[choice][1]()
    except KeyboardInterrupt:
        print("\nInterrupted.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
