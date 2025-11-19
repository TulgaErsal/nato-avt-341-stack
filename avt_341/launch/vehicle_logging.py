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
    TYPE = 'type'


class FieldSymbol:

    def __init__(self, key, raw_key, values):
        self.key = key
        self.slice_op = re.findall(r"\[(.*?)\]", raw_key)
        self.values = values if type(values) is list else [values]
        self.has_symbol_properties = any([type(v) is dict for v in self.values])

    def get_values(self, symbol_str):
        if self.slice_op:
            return eval(f"self.values[{self.slice_op[0]}]")

        # Symbol property
        symbol_property_key = symbol_str.split('.')[-1]
        if symbol_property_key != symbol_str:
            return [sv[symbol_property_key] for sv in self.values]

        return self.values

    def expansion_size(self):
        return len(self.get_values(self.key))

    def matches_raw_key(self, key_str):
        if self.has_symbol_properties:
            return any([f"{self.key}.{k}" == key_str for sv in self.values for k in sv.keys()])
        key_str = key_str.split('[')[0]     # Handles slicing operator
        return self.key == key_str

    def get_templated_values(self, field_str):
        for raw_key in re.findall(r"{(.*?)}", field_str):
            if self.matches_raw_key(raw_key):
                return raw_key, self.get_values(raw_key)
        return None, []


class BagConfigLoader:

    def __init__(self, config_path, vehicles_override):
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
            self._parse_yaml(vehicles_override)
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

    def _parse_yaml(self, vehicles_override):

        # Already parsed
        if self._config_yaml:
            return

        # Nothing to parse
        if not self.has_template():
            self._config_yaml = self.input_yaml
            return

        self._config_yaml = {BagConfigKeys.LOG_TOPICS: []}
        all_symbol_values = self.input_yaml[BagConfigKeys.SYMBOLS]
        all_symbol_values['vehicles'] = vehicles_override or all_symbol_values['vehicles']

        for topic_i_dict in self.input_yaml[BagConfigKeys.LOG_TOPICS]:

            # Find all unique symbols in all fields of topic dict
            raw_field_symbols = set([s for l in (re.findall(r"{(.*?)}", v) for v in topic_i_dict.values()) for s in l])
            symbols_map = {}
            for rs in raw_field_symbols:
                s = rs.split('[')[0].split('.')[0]
                if s not in all_symbol_values:
                    raise ValueError(f"Symbol definition {s} missing. Did you forget to update the symbols dictionary?")
                if s not in symbols_map:
                    symbols_map[s] = FieldSymbol(s, rs, all_symbol_values[s])

            # Make all items in topic_i_dict a list to accommodate template expansions
            for k, v in topic_i_dict.items():
                topic_i_dict[k] = [v]

            expanded_size = 1
            # Expand symbols in fields
            for s_key, symbol_obj in symbols_map.items():

                for k, v in topic_i_dict.items():
                    symbol_template, symbol_values = symbol_obj.get_templated_values(v[0])
                    if symbol_template:
                        # [{vehicles}_x, {vehicles}_y] -> [veh1_x, veh1_y, veh2_x, veh2_y]
                        # Order important in double for loop to play nicely with v * len(symbol_obj.values)
                        field_value = [vi.replace(f"{{{symbol_template}}}", sv) for sv in symbol_values for vi in v]
                    else:
                        field_value = v * symbol_obj.expansion_size()
                    topic_i_dict[k] = field_value
                    expanded_size = len(field_value)

            for _, v in topic_i_dict.items():
                assert len(v) == expanded_size

            for j in range(expanded_size):
                expanded_fields = {k: v[j] for k, v in topic_i_dict.items()}
                self._config_yaml[BagConfigKeys.LOG_TOPICS].append(expanded_fields)

        # Check all resulting parsed topics unique
        name_list = [topic_i['name'] for topic_i in self._config_yaml[BagConfigKeys.LOG_TOPICS]]
        duplicated_names = [name for name in set(name_list) if name_list.count(name) > 1]
        if duplicated_names:
            raise ValueError(f"All entries should have unique names after parsing. Duplicated names: {duplicated_names}")


    @property
    def config_yaml(self):
        return self._config_yaml

    def get_record_topics(self, exlude_occupancy_grid):
        if not self._config_yaml:
            return []

        return [t[BagConfigKeys.TOPIC] for t in self._config_yaml[BagConfigKeys.LOG_TOPICS]
                if t[BagConfigKeys.TOPIC] and (not exlude_occupancy_grid or t[BagConfigKeys.TYPE] != 'nav_msgs/msg/OccupancyGrid')]

def parse_args(is_ros1):
    parser = ArgumentParser(prog="rosrun avt_341 vehicle_logging.py" if is_ros1 else "ros2 run avt_341 vehicle_logging.py",
                            description="Records configured topics in a rosbag data structured.")

    parser.add_argument('config_file', type=str, help="Path to the logging config file.")
    parser.add_argument('save_dir', type=str, help="Path to the directory to save rosbag data.")
    parser.add_argument('--save_prefix', type=str, default="AVT_341_DATALOG", help="Prefix for output rosbag name.")
    parser.add_argument('--bag_name_override', type=str, default="", help="If set, overrides created bag name.")
    parser.add_argument('--bag_format', type=str, default="", help="Customize bag file format. ONLY SUPPORTED IN ROS2.\nRos2 values: sqlite3 | mcap. Pass empty string to use default format (sqlite3 for older versions)")
    parser.add_argument('--storage_config_file', type=str, default="", help="Yaml file containing storage options for bag. ONLY SUPPORTED IN ROS2.")
    parser.add_argument('--config_file_out', type=str, default="logging_config.yaml", help="Name to use for copied bag configuration file that will appear in output bag directory.")
    parser.add_argument('--vehicles_override', type=str, default="", help="Comma seperated list of vehicles to use in symbols of bag log config. Leave blank for no override.")
    parser.add_argument('--exclude_occupancy_grid', type=bool, default=False, help="If set, occupancy grids will be excluded from the bag file. Inclusion of occupancy grid may cause large bag file sizes if no compression used.")
    args = parser.parse_args()

    if args.bag_format and is_ros1:
        sys.exit("Bag format customization is only supported in ROS2.")

    if args.bag_format and args.bag_format not in ['sqlite3', 'mcap']:
        sys.exit(f"Invalid bag format [{args.bag_format}]. Use no argument to select default type or explicitly select sqlite3 | mcap")

    # File naming constants
    if args.bag_name_override:
        args.save_name = args.bag_name_override
    else:
        time_YYMMDD = datetime.now().strftime('%y%m%d')
        time_HHMMSS = datetime.now().strftime('%H%M%S')
        args.save_name = f"{time_YYMMDD}_{args.save_prefix}_{time_HHMMSS}"
    args.save_path = os.path.join(args.save_dir, args.save_name)
    args.vehicles = [v.strip() for v in args.vehicles_override.split(',')] if args.vehicles_override else []
    return args


if __name__ == "__main__":

    is_ros1 = os.environ['ROS_VERSION'] == '1'
    args = parse_args(is_ros1)

    bag_config_loader = BagConfigLoader(args.config_file, args.vehicles)
    if bag_config_loader.in_error():
        sys.exit(bag_config_loader.error)

    record_topics = bag_config_loader.get_record_topics(args.exclude_occupancy_grid)

    try:
        # Record rosbag using CLI
        command = []
        if is_ros1:
            os.mkdir(args.save_path)
            command = ['rosbag', 'record', '-O', os.path.join(args.save_path, args.save_name), *record_topics]
        else:
            command = (['ros2', 'bag', 'record']
                       + (['-s', args.bag_format] if args.bag_format else [])
                       + (['--storage-config-file', args.storage_config_file] if args.storage_config_file else [])
                       + ['-o', args.save_path, *record_topics])
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
        sys.exit(0)