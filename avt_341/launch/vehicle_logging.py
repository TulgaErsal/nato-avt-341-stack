#!/usr/bin/env python3
#################################################################
# Used for common collection of rosbag data for V&V efforts.    #
#                                                               #
# @author   Evan Vandermate                                     #
# @email    evanderm@mtu.edu                                    #
# @date     02/25/2025                                          #
#################################################################
import os
import sys
import yaml
import shutil
import signal
import subprocess
import time
from datetime import datetime


# Parse args
if len(sys.argv) < 3 or any(s in sys.argv for s in ['-h','--help']):
    if os.environ['ROS_VERSION'] == '1':
        print("Usage: rosrun avt_341 vehicle_logging.py [config_file] [save_dir] [save_prefix=\"AVT_341_DATALOG\"]")
    else:
        print("Usage: ros2 run avt_341 vehicle_logging.py [config_file] [save_dir] [save_prefix=\"AVT_341_DATALOG\"]")
    sys.exit(0)
config_file = os.path.abspath(sys.argv[1])
save_dir = sys.argv[2]
save_prefix = sys.argv[3] if len(sys.argv) >= 4 else "AVT_341_DATALOG"

# Load logging config file
params = {}
try:
    with open(config_file, 'r') as f:    
        params = yaml.safe_load(f)
except FileNotFoundError:
    sys.exit(f"Unable to find config file [{config_file}]")

# Read topics to log
record_topics = set()
for t in params['log_topics']:
    if t['topic'] != '':
        record_topics.add(t['topic'])

# File naming constants
time_YYMMDD = datetime.now().strftime('%y%m%d')
time_HHMMSS = datetime.now().strftime('%H%M%S')
save_name = f"{time_YYMMDD}_{save_prefix}_{time_HHMMSS}"
save_path = os.path.join(save_dir,save_name)

if __name__ == "__main__":
    try:
        # Record rosbag using CLI
        command = []
        if os.environ['ROS_VERSION'] == '1':
            os.mkdir(save_path)
            command = ['rosbag', 'record', '-O', os.path.join(save_path,save_name), *record_topics]
        else:
            command = ['ros2', 'bag', 'record', '-o', save_path, *record_topics]
        process = subprocess.Popen(command, stdout=subprocess.PIPE)

        # Copy the logging config to the save path
        while not os.path.isdir(save_path):
            time.sleep(0.1) # Wait for save path to be created
        shutil.copy2(config_file, os.path.join(save_path,'logging_config.yaml'))

        # Wait for logging to complete
        process.wait()
    except KeyboardInterrupt:
        process.send_signal(signal.SIGINT)
        process.wait()
        sys.exit("LOGGING COMPLETE")