#!/usr/bin/env python3
"""Extrai /goal_joint_states e /feedback de um bag ROS2 para CSVs (formato largo: uma coluna por junta)."""
import sys
import csv
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState


def read_joint_state_topic(bag_path: str, topic: str):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    type_by_topic = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in type_by_topic:
        return []

    rows = []
    while reader.has_next():
        name, data, t = reader.read_next()
        if name != topic:
            continue
        msg: JointState = deserialize_message(data, JointState)
        row = {"stamp_ns": t}
        for joint_name, pos in zip(msg.name, msg.position):
            row[joint_name] = pos
        rows.append(row)
    return rows


def write_csv(rows, out_path: Path):
    if not rows:
        print(f"Nenhuma mensagem encontrada, {out_path} nao sera criado.")
        return
    fieldnames = ["stamp_ns"]
    for row in rows:
        for k in row:
            if k not in fieldnames:
                fieldnames.append(k)
    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"Escrito {len(rows)} linhas em {out_path}")


def main():
    if len(sys.argv) < 2:
        print("Uso: bag_to_csv.py <caminho_do_bag> [dir_saida]")
        sys.exit(1)

    bag_path = sys.argv[1]
    out_dir = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(".")
    out_dir.mkdir(parents=True, exist_ok=True)

    write_csv(read_joint_state_topic(bag_path, "/goal_joint_states"), out_dir / "goal_joint_states.csv")
    write_csv(read_joint_state_topic(bag_path, "/feedback"), out_dir / "feedback.csv")


if __name__ == "__main__":
    main()
