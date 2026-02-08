#!/usr/bin/env python3
import sys
import math
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

G = 9.80665  # standard gravity

def main():
    if len(sys.argv) != 3:
        print("Usage: python3 scale_livox_imu_g.py <input_bag> <output_bag>")
        sys.exit(1)

    input_bag = sys.argv[1]
    output_bag = sys.argv[2]

    # ---------- Reader ----------
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', '')
    )

    topics = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics}

    # ---------- Writer ----------
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=output_bag, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', '')
    )

    for topic, msg_type in type_map.items():
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=topic,
                type=msg_type,
                serialization_format='cdr'
            )
        )

    # ---------- Main loop ----------
    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        if topic == "/livox/imu":
            # scale linear acceleration
            msg.linear_acceleration.x *= G
            msg.linear_acceleration.y *= G
            msg.linear_acceleration.z *= G

        writer.write(
            topic,
            serialize_message(msg),
            timestamp
        )

    print(f"Finished. New bag written to: {output_bag}")

if __name__ == "__main__":
    main()
