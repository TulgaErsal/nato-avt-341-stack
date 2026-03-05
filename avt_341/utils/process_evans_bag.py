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
    data['mrzr2_odom'] = get_messages('/mrzr/avt_341/odometry')
    
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
    filtered_x, filtered_y = extract_xy(data['pose_filtered'])
    raw_x, raw_y = extract_xy(data['pose_raw'])
    
    # Compute shared axis limits across all data in both plots.
    all_x = mrzr2_x + filtered_x + raw_x
    all_y = mrzr2_y + filtered_y + raw_y
    x_min, x_max = min(all_x), max(all_x)
    y_min, y_max = min(all_y), max(all_y)
    x_pad = (x_max - x_min) * 0.05
    y_pad = (y_max - y_min) * 0.05
    shared_xlim = (x_min - x_pad, x_max + x_pad)
    shared_ylim = (y_min - y_pad, y_max + y_pad)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

    # Plot 1: Filtered
    ax1.plot(mrzr2_x, mrzr2_y, 'b-', label='MRZR2 Path', alpha=0.5)
    if filtered_x:
        v_min = -len(filtered_x) * 0.4  # Shift vmin to avoid too light colors
        ax1.scatter(filtered_x, filtered_y, c=range(len(filtered_x)), cmap='Greens', s=15,
                    label='/pose/filtered', alpha=0.8, vmin=v_min)
    ax1.set_title('Filtered Pose Tracking (Visible Light -> Dark over time)')
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.legend()
    ax1.grid(True)
    ax1.set_xlim(shared_xlim)
    ax1.set_ylim(shared_ylim)
    ax1.set_aspect('equal')

    # Plot 2: Raw
    ax2.plot(mrzr2_x, mrzr2_y, 'b-', label='MZRZ2 Path', alpha=0.5)
    if raw_x:
        v_min_raw = -len(raw_x) * 0.4  # Shift vmin to avoid too light colors
        ax2.scatter(raw_x, raw_y, c=range(len(raw_x)), cmap='YlOrBr', s=15,
                    label='/pose/raw', alpha=0.8, vmin=v_min_raw)
    ax2.set_title('Raw Pose Tracking (Visible Light -> Dark over time)')
    ax2.set_xlabel('X [m]')
    ax2.set_ylabel('Y [m]')
    ax2.legend()
    ax2.grid(True)
    ax2.set_xlim(shared_xlim)
    ax2.set_ylim(shared_ylim)
    ax2.set_aspect('equal')
    
    fig.tight_layout()
    output_plot = 'tracking_results.png'
    fig.savefig(output_plot)
    print(f"Plots saved to {output_plot}")
    plt.show()

if __name__ == '__main__':
    main()
