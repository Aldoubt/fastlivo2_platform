#!/usr/bin/env python3
import argparse
import bisect
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def resolve_bag_uri(path):
    if path.is_file() and path.suffix == '.db3':
        return path.parent

    if path.is_dir():
        return path

    candidates = []
    for bags_dir in (Path('data/bags'), Path('ros2_ws/data/bags')):
        if bags_dir.exists():
            for metadata in sorted(bags_dir.glob('*/metadata.yaml')):
                candidates.append(metadata.parent)

    hint = ''
    if candidates:
        hint = '\nAvailable bag directories:\n' + '\n'.join(f'  {candidate}' for candidate in candidates)

    raise FileNotFoundError(
        f"Bag path '{path}' does not exist. Pass the rosbag2 directory, not just a guessed .db3 name."
        + hint
    )


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def nearest_delta(reference, samples):
    if not samples:
        return None

    if reference < samples[0] or reference > samples[-1]:
        return None

    index = bisect.bisect_left(samples, reference)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index] - reference)
    if index > 0:
        candidates.append(samples[index - 1] - reference)

    return min(candidates, key=abs)


def summarize(name, deltas):
    if not deltas:
        return f'{name}: no matched samples'

    abs_ms = sorted(abs(delta) * 1000.0 for delta in deltas)
    signed_ms = [delta * 1000.0 for delta in deltas]

    def percentile(percent):
        if len(abs_ms) == 1:
            return abs_ms[0]
        rank = (len(abs_ms) - 1) * percent / 100.0
        low = math.floor(rank)
        high = math.ceil(rank)
        if low == high:
            return abs_ms[int(rank)]
        return abs_ms[low] + (abs_ms[high] - abs_ms[low]) * (rank - low)

    mean_signed = sum(signed_ms) / len(signed_ms)
    mean_abs = sum(abs_ms) / len(abs_ms)

    return (
        f'{name}: count={len(deltas)}, '
        f'mean={mean_signed:+.3f} ms, '
        f'mean_abs={mean_abs:.3f} ms, '
        f'p95_abs={percentile(95):.3f} ms, '
        f'max_abs={abs_ms[-1]:.3f} ms'
    )


def read_header_stamps(bag_uri, topics):
    bag_uri = resolve_bag_uri(Path(bag_uri))
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_uri), storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }

    missing = [topic for topic in topics.values() if topic not in topic_types]
    if missing:
        available = '\n'.join(f'  {name}: {type_name}' for name, type_name in sorted(topic_types.items()))
        raise RuntimeError(
            'bag is missing required topics: '
            + ', '.join(missing)
            + '\nAvailable topics:\n'
            + available
        )

    message_types = {
        topic: get_message(topic_types[topic]) for topic in topics.values()
    }
    stamps = {key: [] for key in topics}

    while reader.has_next():
        topic, data, _ = reader.read_next()
        for key, expected_topic in topics.items():
            if topic != expected_topic:
                continue
            msg = deserialize_message(data, message_types[topic])
            stamps[key].append(stamp_to_sec(msg.header.stamp))
            break

    for values in stamps.values():
        values.sort()

    return stamps


def main():
    parser = argparse.ArgumentParser(
        description='Analyze LiDAR/IMU/image header timestamp alignment from a rosbag2.'
    )
    parser.add_argument(
        'bag',
        type=Path,
        help='rosbag2 directory, for example data/bags/livo_20260627_153000. '
             'A .db3 path inside a bag directory is also accepted.',
    )
    parser.add_argument('--lidar-topic', default='/livox/lidar')
    parser.add_argument('--imu-topic', default='/livox/imu')
    parser.add_argument('--image-topic', default='/left_camera/image')
    args = parser.parse_args()

    topics = {
        'lidar': args.lidar_topic,
        'imu': args.imu_topic,
        'image': args.image_topic,
    }

    stamps = read_header_stamps(args.bag, topics)

    print('Topic sample counts:')
    for key, topic in topics.items():
        values = stamps[key]
        if values:
            print(f'  {topic}: {len(values)} samples, {values[0]:.6f} -> {values[-1]:.6f}')
        else:
            print(f'  {topic}: 0 samples')

    img_lidar = []
    img_imu = []
    for img_stamp in stamps['image']:
        lidar_delta = nearest_delta(img_stamp, stamps['lidar'])
        imu_delta = nearest_delta(img_stamp, stamps['imu'])
        if lidar_delta is not None:
            img_lidar.append(img_stamp - (img_stamp + lidar_delta))
        if imu_delta is not None:
            img_imu.append(img_stamp - (img_stamp + imu_delta))

    print()
    print('Nearest-neighbor timestamp deltas using each image frame as reference:')
    print(summarize('img - nearest_lidar', img_lidar))
    print(summarize('img - nearest_imu', img_imu))


if __name__ == '__main__':
    main()
