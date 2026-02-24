import sqlite3
import os
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

def extract_raw_poses(bag_path, output_txt):
    # Find the database file
    db_files = [f for f in os.listdir(bag_path) if f.endswith('.db3')]
    if not db_files:
        raise FileNotFoundError(f"No .db3 files found in {bag_path}")
    db_path = os.path.join(bag_path, db_files[0])
    
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    # Get topic list and types
    cursor.execute("SELECT id, name, type FROM topics WHERE name = '/pose/raw'")
    topic_row = cursor.fetchone()
    if not topic_row:
        print("Topic /pose/raw not found in bag")
        return
    
    topic_id, topic_name, topic_type = topic_row
    msg_class = get_message(topic_type)
    
    cursor.execute("SELECT data, timestamp FROM messages WHERE topic_id = ? ORDER BY timestamp", (topic_id,))
    
    with open(output_txt, 'w') as f:
        f.write("# timestamp, x, y, z, qx, qy, qz, qw\n")
        count = 0
        for row in cursor.fetchall():
            msg = deserialize_message(row[0], msg_class)
            timestamp = row[1]
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            f.write(f"{timestamp}, {pos.x}, {pos.y}, {pos.z}, {ori.x}, {ori.y}, {ori.z}, {ori.w}\n")
            count += 1
    
    conn.close()
    print(f"Extracted {count} messages from /pose/raw to {output_txt}")

def main():
    bag_path = os.path.expanduser('~/colcon_ws/test_recording')
    output_file = 'raw_poses.txt'
    
    if os.path.exists(bag_path):
        extract_raw_poses(bag_path, output_file)
    else:
        print(f"Bag file path not found: {bag_path}")

if __name__ == '__main__':
    main()
