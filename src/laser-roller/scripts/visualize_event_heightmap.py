#!/usr/bin/python3
"""
Accumulate all events from a rosbag into a 2D height map (event count per pixel)
and display it with matplotlib.

Usage:
    python3 visualize_event_heightmap.py                        # uses latest bag, events 100k-150k
    python3 visualize_event_heightmap.py --start 0 --end -1     # all events
    python3 visualize_event_heightmap.py --start 200000 --end 300000
    python3 visualize_event_heightmap.py --polarity pos          # only positive events
    python3 visualize_event_heightmap.py --save heightmap.png    # save to file
"""
import argparse
import glob
import os
import sys

import numpy as np
import rosbag
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm


EVENT_TOPIC = "/event_crop/events"


def default_recordings_dir():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(script_dir, "recordings")


def find_latest_bag(recordings_dir):
    bag_files = glob.glob(os.path.join(recordings_dir, "*.bag"))
    if not bag_files:
        return None
    return max(bag_files, key=os.path.getmtime)


def accumulate_heightmap(bag_path, topic=EVENT_TOPIC, polarity_filter=None,
                         start_event=0, end_event=10000000):
    """
    Read events from a rosbag and accumulate them into a height map.

    Args:
        bag_path: Path to the .bag file
        topic: ROS topic name for events
        polarity_filter: None (all events), 'pos' (only positive), 'neg' (only negative)
        start_event: Index of first event to include (default 100000)
        end_event: Index of last event to include, or -1 for all (default 150000)

    Returns:
        heightmap: 2D numpy array of event counts per pixel
        metadata: dict with bag info
    """
    if end_event == -1:
        range_label = f"events {start_event:,} to end"
    else:
        range_label = f"events {start_event:,} to {end_event:,}"
    print(f"Reading events from: {bag_path}")
    print(f"Event range: {range_label}")

    width, height = 0, 0
    global_event_idx = 0  # running count across all batches
    accumulated_events = 0
    total_batches = 0

    # First pass: determine dimensions
    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            if msg.width > 0 and msg.height > 0:
                width, height = msg.width, msg.height
                break

    if width == 0 or height == 0:
        print("Error: could not determine sensor dimensions from bag.")
        return None, None

    print(f"Sensor dimensions: {width} x {height}")

    # Accumulate all events
    heightmap = np.zeros((height, width), dtype=np.uint64)

    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            total_batches += 1
            if not msg.events:
                continue

            n = len(msg.events)
            batch_start = global_event_idx
            batch_end = global_event_idx + n
            global_event_idx += n

            # Skip batches entirely before the range
            if end_event != -1 and batch_start >= end_event:
                break
            if batch_end <= start_event:
                continue

            # Determine which events within this batch fall in [start_event, end_event)
            local_start = max(0, start_event - batch_start)
            local_end = n if end_event == -1 else min(n, end_event - batch_start)
            events_slice = msg.events[local_start:local_end]

            if not events_slice:
                continue

            ns = len(events_slice)
            xs = np.fromiter((e.x for e in events_slice), dtype=np.uint16, count=ns)
            ys = np.fromiter((e.y for e in events_slice), dtype=np.uint16, count=ns)

            # Apply polarity filter if requested
            if polarity_filter is not None:
                pols = np.fromiter((e.polarity for e in events_slice), dtype=bool, count=ns)
                if polarity_filter == "pos":
                    mask = pols
                else:
                    mask = ~pols
                xs = xs[mask]
                ys = ys[mask]

            # Filter within bounds
            valid = (xs < width) & (ys < height)
            xs = xs[valid]
            ys = ys[valid]

            # Accumulate counts
            np.add.at(heightmap, (ys, xs), 1)
            accumulated_events += len(xs)

            if total_batches % 2000 == 0:
                print(f"  Processed {total_batches} batches, {accumulated_events:,} events accumulated...")

    metadata = {
        "bag_path": bag_path,
        "width": width,
        "height": height,
        "total_events_in_bag": global_event_idx,
        "accumulated_events": accumulated_events,
        "event_range": f"{start_event}-{end_event}",
        "total_batches": total_batches,
        "polarity_filter": polarity_filter or "all",
        "max_count": int(heightmap.max()),
        "min_count": int(heightmap.min()),
        "nonzero_pixels": int(np.count_nonzero(heightmap)),
    }

    print(f"\nDone! Accumulated {accumulated_events:,} events ({range_label}) "
          f"out of {global_event_idx:,} total.")
    print(f"Max events per pixel: {metadata['max_count']:,}")
    print(f"Active pixels: {metadata['nonzero_pixels']:,} / {width * height:,} "
          f"({100.0 * metadata['nonzero_pixels'] / (width * height):.1f}%)")

    return heightmap, metadata


def show_heightmap(heightmap, metadata, save_path=None, log_scale=False):
    """Display the height map using matplotlib."""
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))

    polarity_label = metadata["polarity_filter"]
    title_suffix = f" ({polarity_label} polarity)" if polarity_label != "all" else ""

    # --- Linear scale ---
    ax1 = axes[0]
    im1 = ax1.imshow(heightmap, cmap="inferno", interpolation="nearest", aspect="equal")
    ax1.set_title(f"Event Height Map (Linear){title_suffix}", fontsize=13, fontweight="bold")
    ax1.set_xlabel("X (pixels)")
    ax1.set_ylabel("Y (pixels)")
    cbar1 = fig.colorbar(im1, ax=ax1, fraction=0.046, pad=0.04)
    cbar1.set_label("Event count")

    # --- Log scale ---
    ax2 = axes[1]
    # Avoid log(0): set zeros to NaN for display
    hm_log = heightmap.astype(np.float64)
    hm_log[hm_log == 0] = np.nan
    im2 = ax2.imshow(
        hm_log,
        cmap="inferno",
        interpolation="nearest",
        aspect="equal",
        norm=LogNorm(vmin=max(1, np.nanmin(hm_log)), vmax=np.nanmax(hm_log)),
    )
    ax2.set_title(f"Event Height Map (Log Scale){title_suffix}", fontsize=13, fontweight="bold")
    ax2.set_xlabel("X (pixels)")
    ax2.set_ylabel("Y (pixels)")
    cbar2 = fig.colorbar(im2, ax=ax2, fraction=0.046, pad=0.04)
    cbar2.set_label("Event count (log)")

    bag_name = os.path.basename(metadata["bag_path"])
    fig.suptitle(f"Source: {bag_name}\nRange: {metadata['event_range']} | "
                 f"{metadata['accumulated_events']:,} events | "
                 f"Max: {metadata['max_count']:,}/px | "
                 f"Active: {metadata['nonzero_pixels']:,} px",
                 fontsize=10, color="gray")

    plt.tight_layout()

    if save_path:
        save_path = os.path.abspath(save_path)
        plt.savefig(save_path, dpi=200, bbox_inches="tight")
        print(f"\nSaved height map to: {save_path}")

    plt.show()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Accumulate events from a rosbag into a 2D height map and display it."
    )
    parser.add_argument(
        "bag",
        nargs="?",
        help="Path to a .bag file. If omitted, uses the newest bag in scripts/recordings.",
    )
    parser.add_argument(
        "--topic",
        default=EVENT_TOPIC,
        help=f"Event topic name (default: {EVENT_TOPIC})",
    )
    parser.add_argument(
        "--polarity",
        choices=["all", "pos", "neg"],
        default="all",
        help="Filter by polarity: 'all' (default), 'pos' (positive only), 'neg' (negative only)",
    )
    parser.add_argument(
        "--start",
        type=int,
        default=None,
        help="Start event index (overrides function default if provided)",
    )
    parser.add_argument(
        "--end",
        type=int,
        default=None,
        help="End event index (overrides function default if provided, use -1 for all)",
    )
    parser.add_argument(
        "--save",
        default=None,
        help="Save the figure to this path (e.g. heightmap.png)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    bag_path = args.bag

    if bag_path is None:
        recordings_dir = default_recordings_dir()
        bag_path = find_latest_bag(recordings_dir)
        if bag_path is None:
            print(f"Error: no .bag files found in {recordings_dir}")
            return 1

    bag_path = os.path.abspath(bag_path)
    if not os.path.exists(bag_path):
        print(f"Error: bag file not found: {bag_path}")
        return 1

    polarity_filter = None if args.polarity == "all" else args.polarity

    # Build kwargs — only override function defaults if CLI args were given
    kwargs = {}
    if args.start is not None:
        kwargs["start_event"] = args.start
    if args.end is not None:
        kwargs["end_event"] = args.end

    heightmap, metadata = accumulate_heightmap(
        bag_path, args.topic, polarity_filter, **kwargs
    )
    if heightmap is None:
        return 1

    show_heightmap(heightmap, metadata, save_path=args.save)
    return 0


if __name__ == "__main__":
    sys.exit(main())
