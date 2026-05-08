#!/usr/bin/python3
import argparse
import glob
import os
import sys

import numpy as np
import rosbag


EVENT_TOPIC = "/capture_node/events"
EVENT_DTYPE = np.dtype(
    [
        ("t_ns", np.uint64),
        ("x", np.uint16),
        ("y", np.uint16),
        ("polarity", np.bool_),
    ]
)


def default_recordings_dir():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(script_dir, "recordings")


def find_latest_bag(recordings_dir):
    bag_files = glob.glob(os.path.join(recordings_dir, "*.bag"))
    if not bag_files:
        return None

    return max(bag_files, key=os.path.getmtime)


def default_output_path(bag_path, output_format):
    base_path, _ = os.path.splitext(os.path.abspath(bag_path))
    extension = ".h5" if output_format == "h5" else ".npz"
    return base_path + extension


def infer_output_format(output_path, requested_format):
    if requested_format != "auto":
        return requested_format

    if output_path:
        extension = os.path.splitext(output_path)[1].lower()
        if extension in (".h5", ".hdf5"):
            return "h5"
        if extension == ".npz":
            return "npz"

    return "h5" if h5py_available() else "npz"


def h5py_available():
    try:
        import h5py  # noqa: F401
    except ImportError:
        return False

    return True


def stamp_to_ns(stamp):
    return int(stamp.secs) * 1000000000 + int(stamp.nsecs)


def event_batch_to_array(message):
    events = message.events
    event_count = len(events)
    event_array = np.empty(event_count, dtype=EVENT_DTYPE)

    if event_count == 0:
        return event_array

    event_array["t_ns"] = np.fromiter(
        (stamp_to_ns(event.ts) for event in events),
        dtype=np.uint64,
        count=event_count,
    )
    event_array["x"] = np.fromiter(
        (event.x for event in events),
        dtype=np.uint16,
        count=event_count,
    )
    event_array["y"] = np.fromiter(
        (event.y for event in events),
        dtype=np.uint16,
        count=event_count,
    )
    event_array["polarity"] = np.fromiter(
        (event.polarity for event in events),
        dtype=np.bool_,
        count=event_count,
    )

    return event_array


def read_event_chunks(
    bag_path,
    topic=EVENT_TOPIC,
    max_events=None,
    progress=False,
    progress_every=1000000,
):
    metadata = {
        "topic": topic,
        "width": 0,
        "height": 0,
        "message_batches": 0,
        "total_events": 0,
        "source_bag": os.path.abspath(bag_path),
    }
    next_progress = progress_every

    with rosbag.Bag(bag_path, "r") as bag:
        for _, message, _ in bag.read_messages(topics=[topic]):
            metadata["message_batches"] += 1
            metadata["width"] = int(getattr(message, "width", metadata["width"]))
            metadata["height"] = int(getattr(message, "height", metadata["height"]))

            event_array = event_batch_to_array(message)
            if max_events is not None:
                remaining = max_events - metadata["total_events"]
                if remaining <= 0:
                    break
                event_array = event_array[:remaining]

            metadata["total_events"] += len(event_array)
            if progress and metadata["total_events"] >= next_progress:
                print(
                    "Read "
                    f"{metadata['total_events']} events from "
                    f"{metadata['message_batches']} message batch(es)...",
                    flush=True,
                )
                while next_progress <= metadata["total_events"]:
                    next_progress += progress_every

            if len(event_array) > 0:
                yield event_array, metadata

            if max_events is not None and metadata["total_events"] >= max_events:
                break

    if metadata["total_events"] == 0:
        yield np.empty(0, dtype=EVENT_DTYPE), metadata


def read_events_as_array(bag_path, topic=EVENT_TOPIC, max_events=None, progress=False):
    chunks = []
    final_metadata = None

    for event_array, metadata in read_event_chunks(
        bag_path,
        topic,
        max_events,
        progress=progress,
    ):
        if len(event_array) > 0:
            chunks.append(event_array)
        final_metadata = dict(metadata)

    if chunks:
        events = np.concatenate(chunks)
    else:
        events = np.empty(0, dtype=EVENT_DTYPE)

    if final_metadata is None:
        final_metadata = {
            "topic": topic,
            "width": 0,
            "height": 0,
            "message_batches": 0,
            "total_events": 0,
            "source_bag": os.path.abspath(bag_path),
        }

    return events, final_metadata


def write_npz(output_path, events, metadata, compressed=False):
    save_function = np.savez_compressed if compressed else np.savez
    save_function(
        output_path,
        events=events,
        t_ns=events["t_ns"],
        x=events["x"],
        y=events["y"],
        polarity=events["polarity"],
        topic=np.array(metadata["topic"]),
        width=np.array(metadata["width"], dtype=np.uint32),
        height=np.array(metadata["height"], dtype=np.uint32),
        message_batches=np.array(metadata["message_batches"], dtype=np.uint32),
        total_events=np.array(metadata["total_events"], dtype=np.uint64),
        source_bag=np.array(metadata["source_bag"]),
    )


def append_h5_dataset(dataset, values):
    old_size = dataset.shape[0]
    new_size = old_size + len(values)
    dataset.resize((new_size,))
    dataset[old_size:new_size] = values


def write_h5(output_path, bag_path, topic=EVENT_TOPIC, max_events=None, progress=False):
    try:
        import h5py
    except ImportError:
        print("Error: h5py is not installed, so HDF5 output is unavailable.")
        print("Install it with: sudo apt install python3-h5py")
        return 1

    final_metadata = None
    with h5py.File(output_path, "w") as h5_file:
        events_group = h5_file.create_group("events")
        t_dataset = events_group.create_dataset(
            "t_ns", shape=(0,), maxshape=(None,), dtype="u8", chunks=True
        )
        x_dataset = events_group.create_dataset(
            "x", shape=(0,), maxshape=(None,), dtype="u2", chunks=True
        )
        y_dataset = events_group.create_dataset(
            "y", shape=(0,), maxshape=(None,), dtype="u2", chunks=True
        )
        polarity_dataset = events_group.create_dataset(
            "polarity", shape=(0,), maxshape=(None,), dtype="?", chunks=True
        )

        for event_array, metadata in read_event_chunks(
            bag_path,
            topic,
            max_events,
            progress=progress,
        ):
            final_metadata = dict(metadata)
            if len(event_array) == 0:
                continue

            append_h5_dataset(t_dataset, event_array["t_ns"])
            append_h5_dataset(x_dataset, event_array["x"])
            append_h5_dataset(y_dataset, event_array["y"])
            append_h5_dataset(polarity_dataset, event_array["polarity"])

        if final_metadata is None:
            final_metadata = {
                "topic": topic,
                "width": 0,
                "height": 0,
                "message_batches": 0,
                "total_events": 0,
                "source_bag": os.path.abspath(bag_path),
            }

        events_group.attrs["topic"] = final_metadata["topic"]
        events_group.attrs["width"] = final_metadata["width"]
        events_group.attrs["height"] = final_metadata["height"]
        events_group.attrs["message_batches"] = final_metadata["message_batches"]
        events_group.attrs["total_events"] = final_metadata["total_events"]
        events_group.attrs["source_bag"] = final_metadata["source_bag"]

    print_conversion_summary(output_path, final_metadata)
    return 0


def print_conversion_summary(output_path, metadata):
    print(f"Saved: {output_path}")
    print(f"Topic: {metadata['topic']}")
    print(f"Resolution: {metadata['width']} x {metadata['height']}")
    print(f"Message batches: {metadata['message_batches']}")
    print(f"Events: {metadata['total_events']}")

    if metadata["total_events"] == 0:
        print("Warning: no events were found in this bag/topic.")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert a DV ROS EventArray rosbag topic into HDF5 or NumPy arrays."
    )
    parser.add_argument(
        "bag",
        nargs="?",
        help="Path to a .bag file. If omitted, the newest bag in scripts/recordings is used.",
    )
    parser.add_argument(
        "-t",
        "--topic",
        default=EVENT_TOPIC,
        help=f"Event topic to read. Default: {EVENT_TOPIC}",
    )
    parser.add_argument(
        "-o",
        "--output",
        help="Output file path. Use .h5/.hdf5 for HDF5 or .npz for NumPy arrays.",
    )
    parser.add_argument(
        "-f",
        "--format",
        choices=("auto", "h5", "npz"),
        default="auto",
        help="Output format. Default: auto.",
    )
    parser.add_argument(
        "--max-events",
        type=int,
        help="Optional limit for quick tests.",
    )
    parser.add_argument(
        "--compressed",
        action="store_true",
        help="Compress .npz output. Smaller file, but slower.",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Do not print progress while reading.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    bag_path = args.bag

    if bag_path is None:
        bag_path = find_latest_bag(default_recordings_dir())
        if bag_path is None:
            print(f"Error: no .bag files found in {default_recordings_dir()}")
            return 1

    bag_path = os.path.abspath(bag_path)
    if not os.path.exists(bag_path):
        print(f"Error: bag file not found: {bag_path}")
        return 1

    output_format = infer_output_format(args.output, args.format)
    output_path = args.output or default_output_path(bag_path, output_format)
    output_path = os.path.abspath(output_path)
    progress = not args.quiet

    if output_format == "h5":
        return write_h5(output_path, bag_path, args.topic, args.max_events, progress)

    events, metadata = read_events_as_array(
        bag_path,
        args.topic,
        args.max_events,
        progress=progress,
    )
    write_npz(output_path, events, metadata, compressed=args.compressed)
    print_conversion_summary(output_path, metadata)
    return 0


if __name__ == "__main__":
    sys.exit(main())
