"""Print raw /joint_states velocity samples from a rosbag2 directory."""

from __future__ import annotations

import argparse
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import JointState

from open_manipulator_sysid.bag_to_dataset import detect_bag_storage_id


def read_messages(input_bag: str):
    bag_dir = Path(input_bag)
    storage_id = detect_bag_storage_id(bag_dir)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )

    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(topic_types[topic])
        msg = deserialize_message(data, msg_type)
        yield topic, msg, timestamp


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Print raw /joint_states velocity from a rosbag2 bag.'
    )
    parser.add_argument('input', help='rosbag2 directory path')
    parser.add_argument(
        '--topic',
        default='/joint_states',
        help='topic to inspect (default: /joint_states)',
    )
    parser.add_argument(
        '--limit',
        type=int,
        default=1000,
        help='max messages to print (0 = all, default: 10)',
    )
    args = parser.parse_args()

    count = 0
    for topic, msg, timestamp in read_messages(args.input):
        if topic != args.topic:
            continue
        if not isinstance(msg, JointState):
            print(f'{topic} [{timestamp}]: not JointState ({type(msg).__name__})')
            continue

        t_sec = timestamp * 1e-9
        print(f'--- {topic}  t={t_sec:.9f}s  stamp={timestamp} ---')
        print(f'  name     = {list(msg.name)}')
        print(f'  position = {list(msg.position)}')
        print(f'  velocity = {list(msg.velocity)}')
        print(f'  effort   = {list(msg.effort)}')
        print()

        count += 1
        if args.limit and count >= args.limit:
            break

    if count == 0:
        print(f'No messages on topic {args.topic!r}.')


if __name__ == '__main__':
    main()