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
import re
from argparse import ArgumentParser


class BagConfigKeys:
    SYMBOLS = 'symbols'
    LOG_TOPICS = 'log_topics'
    TOPIC = 'topic'


class BagConfigLoader:

    def __init__(self, config_path):
        self.input_yaml = None
        self._config_yaml = None
        self._error = None

        try:
            with open(config_path, 'r') as f:
                self.input_yaml = yaml.safe_load(f)
        except FileNotFoundError:
            self._error = f"Unable to find config file [{args.config_file}]"
            return

        try:
            self._parse_yaml()
        except Exception as e:
            self._error = f"Error parsing config [{args.config_file}]: {str(e)}"
            return

    def in_error(self):
        return self._error is not None

    @property
    def error(self):
        return self._error

    def has_template(self):
        return BagConfigKeys.SYMBOLS in self.input_yaml

    def _parse_yaml(self):

        # Already parsed
        if self._config_yaml:
            return

        # Nothing to parse
        if not self.has_template():
            self._config_yaml = self.input_yaml
            return

        self._config_yaml = {BagConfigKeys.LOG_TOPICS: []}
        symbols = self.input_yaml[BagConfigKeys.SYMBOLS]

        for topic_i_dict in self.input_yaml[BagConfigKeys.LOG_TOPICS]:

            # Expand symbols in fields
            # - name: {vehicles}_x                  ->   name: [veh1_x, veh2_x, veh3_x, veh4_x]
            #   topic: /avt_341/{vehicles}/x        ->   topic: [/avt_341/veh1/x, /avt_341/veh2/x, /avt_341/veh3/x, /avt_341/veh4/x]
            for k, v in topic_i_dict.items():
                field_value = [v]
                if isinstance(v, str):
                    matches = [m for m in re.findall(r"{(.*?)}", v) if m in symbols]
                    for m in matches:
                        replace_value = symbols[m]
                        if type(replace_value) is list:
                            field_value = [v.replace(f"{{{m}}}", rv_i) for v in field_value for rv_i in replace_value]
                        else:
                            field_value = [v.replace(f"{{{m}}}", replace_value) for v in field_value]
                topic_i_dict[k] = field_value

            unique_expanded_lengths = set([len(v) for v in topic_i_dict.values() if len(v) > 1])

            if len(unique_expanded_lengths) > 1:
                raise ValueError("All fields after symbol expansion must have the same length" + os.linesep +
                                 os.linesep.join([f"{k}:{len(v)}" for k, v in topic_i_dict.items()]))

            expand_to_size = unique_expanded_lengths.pop() if unique_expanded_lengths else 1

            # Expand fields with only 1 element to size of expanded fields
            #   - name: [a,b,c]                 -->     [a,b,c]
            #     value: 1                      -->     [1,1,1]
            #     field: w                      -->     [w,w,w]
            #     topic: [x, y, z]              -->     [x,y,z]
            for k, v in topic_i_dict.items():
                if len(v) == 1:
                    topic_i_dict[k] = v * expand_to_size

            # Expand sub-lists into individual topic dictionaries
            for j in range(expand_to_size):
                expanded_fields = {k: v[j] for k, v in topic_i_dict.items()}
                self._config_yaml[BagConfigKeys.LOG_TOPICS].append(expanded_fields)

        # Check all resulting parsed topics unique
        name_list = [topic_i['name'] for topic_i in self._config_yaml[BagConfigKeys.LOG_TOPICS]]
        duplicated_names = [name for name in set(name_list) if name_list.count(name) > 1]
        if duplicated_names:
            raise ValueError(f"All topics should have unique names after parsing. Duplicated names: {duplicated_names}")


    @property
    def config_yaml(self):
        return self._config_yaml

    @property
    def record_topics(self):
        return [t[BagConfigKeys.TOPIC] for t in self._config_yaml[BagConfigKeys.LOG_TOPICS] if t[BagConfigKeys.TOPIC]] if self._config_yaml else []

def parse_args(is_ros1):
    parser = ArgumentParser(prog="rosrun avt_341 vehicle_logging.py" if is_ros1 else "ros2 run avt_341 vehicle_logging.py",
                            description="Records configured topics in a rosbag data structured.")

    parser.add_argument('config_file', type=str, help="Path to the logging config file.")
    parser.add_argument('save_dir', type=str, help="Path to the directory to save rosbag data.")
    parser.add_argument('--save_prefix', type=str, default="AVT_341_DATALOG", help="Prefix for output rosbag name.")
    parser.add_argument('--bag_format', type=str, default="", help="Customize bag file format. ONLY SUPPORTED IN ROS2.\nRos2 values: sqlite3 | mcap. Pass empty string to use default format (sqlite3 for older versions)")
    parser.add_argument('--config_file_out', type=str, default="logging_config.yaml", help="Name to use for copied bag configuration file that will appear in output bag directory.")
    args = parser.parse_args()

    if args.bag_format and is_ros1:
        sys.exit("Bag format customization is only supported in ROS2.")

    if args.bag_format and args.bag_format not in ['sqlite3', 'mcap']:
        sys.exit(f"Invalid bag format [{args.bag_format}]. Use no argument to select default type or explicitly select sqlite3 | mcap")

    # File naming constants
    time_YYMMDD = datetime.now().strftime('%y%m%d')
    time_HHMMSS = datetime.now().strftime('%H%M%S')
    args.save_name = f"{time_YYMMDD}_{args.save_prefix}_{time_HHMMSS}"
    args.save_path = os.path.join(args.save_dir, args.save_name)

    return args


if __name__ == "__main__":

    is_ros1 = os.environ['ROS_VERSION'] == '1'
    args = parse_args(is_ros1)

    bag_config_loader = BagConfigLoader(args.config_file)
    if bag_config_loader.in_error():
        sys.exit(bag_config_loader.error)

    record_topics = bag_config_loader.record_topics

    try:
        # Record rosbag using CLI
        command = []
        if is_ros1:
            os.mkdir(args.save_path)
            command = ['rosbag', 'record', '-O', os.path.join(args.save_path, args.save_name), *record_topics]
        else:
            command = ['ros2', 'bag', 'record'] + (['-s', args.bag_format] if args.bag_format else []) + ['-o', args.save_path, *record_topics]
        process = subprocess.Popen(command, stdout=subprocess.PIPE)

        # Copy the logging config to the save path
        while not os.path.isdir(args.save_path):
            time.sleep(0.1) # Wait for save path to be created

        config_file_out_path = os.path.join(args.save_path, args.config_file_out)
        if bag_config_loader.has_template():
            with open(config_file_out_path, 'w') as outfile:
                yaml.dump(bag_config_loader.config_yaml, outfile, sort_keys=False)
        else:
            shutil.copy2(args.config_file, config_file_out_path)

        # Wait for logging to complete
        process.wait()
    except KeyboardInterrupt:
        process.send_signal(signal.SIGINT)
        process.wait()
        sys.exit("LOGGING COMPLETE")