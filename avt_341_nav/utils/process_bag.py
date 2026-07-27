import sqlite3
import matplotlib.pyplot as plt
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import os
import argparse

def read_bag(bag_path):
    # Find the database file
    db_files = [f for f in os.listdir(bag_path) if f.endswith('.db3')]
    if not db_files:
        raise FileNotFoundError(f"No .db3 files found in {bag_path}")
    db_path = os.path.join(bag_path, db_files[0])
    
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    # Get topic list and types
    cursor.execute("SELECT id, name, type FROM topics")
    topics = {row[1]: {'id': row[0], 'type': row[2]} for row in cursor.fetchall()}
    
    def get_messages(topic_name):
        if topic_name not in topics:
            print(f"Topic {topic_name} not found in bag")
            return []
        
        topic_id = topics[topic_name]['id']
        topic_type = topics[topic_name]['type']
        msg_class = get_message(topic_type)
        
        cursor.execute("SELECT data FROM messages WHERE topic_id = ?", (topic_id,))
        messages = []
        for row in cursor.fetchall():
            msg = deserialize_message(row[0], msg_class)
            messages.append(msg)
        return messages

    data = {}
    data['pose_filtered'] = get_messages('/pose/filtered')
    data['pose_raw'] = get_messages('/pose/raw')
    data['mrzr2_odom'] = get_messages('/mrzr2/avt_341/odometry')
    data['mrzr4_odom'] = get_messages('/mrzr4/avt_341/odometry')
    
    conn.close()
    return data

def extract_xy(messages):
    x = []
    y = []
    for msg in messages:
        pos = msg.pose.pose.position
        if pos.x == 0.0 and pos.y == 0.0:
            continue
        x.append(pos.x)
        y.append(pos.y)
    return x, y

def main():
    parser = argparse.ArgumentParser(description='Process a ROS 2 bag and plot tracking results.')
    parser.add_argument('bag_path', type=str, nargs='?', default='~/colcon_ws/test_recording',
                        help='Path to the ROS 2 bag directory')
    args = parser.parse_args()

    bag_path = os.path.expanduser(args.bag_path)
    print(f"Reading bag from {bag_path}...")
    
    try:
        data = read_bag(bag_path)
    except Exception as e:
        print(f"Error reading bag: {e}")
        return
    
    mrzr2_x, mrzr2_y = extract_xy(data['mrzr2_odom'])
    mrzr4_x, mrzr4_y = extract_xy(data['mrzr4_odom'])
    filtered_x, filtered_y = extract_xy(data['pose_filtered'])
    raw_x, raw_y = extract_xy(data['pose_raw'])
    
    plt.figure(figsize=(12, 10))
    
    # Plot 1: Filtered
    plt.subplot(2, 1, 1)
    plt.plot(mrzr2_x, mrzr2_y, 'b-', label='MRZR2 Path', alpha=0.5)
    plt.plot(mrzr4_x, mrzr4_y, 'ro', label='MRZR4 Location', markersize=2)
    if filtered_x:
        v_min = -len(filtered_x) * 0.4  # Shift vmin to avoid too light colors
        plt.scatter(filtered_x, filtered_y, c=range(len(filtered_x)), cmap='Greens', s=15, 
                    label='/pose/filtered', alpha=0.8, vmin=v_min)
    plt.title('Filtered Pose Tracking (Visible Light -> Dark over time)')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    # Plot 2: Raw
    plt.subplot(2, 1, 2)
    plt.plot(mrzr2_x, mrzr2_y, 'b-', label='MRZR2 Path', alpha=0.5)
    plt.plot(mrzr4_x, mrzr4_y, 'ro', label='MRZR4 Location', markersize=2)
    if raw_x:
        v_min_raw = -len(raw_x) * 0.4  # Shift vmin to avoid too light colors
        plt.scatter(raw_x, raw_y, c=range(len(raw_x)), cmap='YlOrBr', s=15, 
                    label='/pose/raw', alpha=0.8, vmin=v_min_raw)
    plt.title('Raw Pose Tracking (Visible Light -> Dark over time)')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    plt.tight_layout()
    output_plot = 'tracking_results.png'
    plt.savefig(output_plot)
    print(f"Plots saved to {output_plot}")
    plt.show()

if __name__ == '__main__':
    main()
