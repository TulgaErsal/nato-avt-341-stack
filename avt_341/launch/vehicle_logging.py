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
from typing import Any, Dict, List

import yaml
import signal
import subprocess
import time
from datetime import datetime
import re
from argparse import ArgumentParser, Namespace

from ament_index_python.packages import get_package_share_directory


class BagConfigKeys:
    SYMBOLS = 'symbols'
    VEHICLES = 'vehicles'
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

    def __init__(self, config_path, vehicles_override: List[str], base_symbols: Dict[str, Any] = None):
        self._config_yaml : Dict[str, Any] = {}
        self._error = None
        self._base_symbols = base_symbols or {}
        self._symbols: Dict[str, Any] = {}

        try:
            with open(config_path, 'r') as f:
                self.input_yaml = yaml.safe_load(f)
        except FileNotFoundError:
            self._error = f"Unable to find config file [{config_path}]"
            return

        try:
            self._parse_yaml(vehicles_override)
        except Exception as e:
            self._error = f"Error parsing config [{config_path}]: {str(e)}"
            return

    def in_error(self):
        return self._error is not None

    @property
    def error(self):
        return self._error

    def has_template(self):
        return BagConfigKeys.SYMBOLS in self.input_yaml or bool(self._base_symbols)

    def _parse_yaml(self, vehicles_override):

        # Already parsed
        if self._config_yaml:
            return

        # Nothing to parse
        if not self.has_template():
            self._config_yaml = self.input_yaml
            return

        self._config_yaml = {k: v for k, v in self.input_yaml.items()
                             if k not in (BagConfigKeys.SYMBOLS, BagConfigKeys.LOG_TOPICS)}
        self._config_yaml[BagConfigKeys.LOG_TOPICS] = []
        if vehicles_override:
            self._config_yaml[BagConfigKeys.VEHICLES] = vehicles_override

        # Base symbols first; this file's own symbols override them.
        all_symbol_values = {**self._base_symbols, **self.input_yaml.get(BagConfigKeys.SYMBOLS, {})}
        # Resolve the 'vehicles' symbol: CLI override > this file's vehicles > inherited base vehicles.
        if vehicles_override:
            all_symbol_values[BagConfigKeys.VEHICLES] = vehicles_override
        elif BagConfigKeys.VEHICLES in self.input_yaml:
            all_symbol_values[BagConfigKeys.VEHICLES] = self.input_yaml[BagConfigKeys.VEHICLES]
        self._symbols = all_symbol_values

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
    def config_yaml(self) -> Dict[str, Any]:
        return self._config_yaml

    @property
    def symbols(self) -> Dict[str, Any]:
        return self._symbols

def get_record_topics(config_yaml: Dict[str, Any]) -> List[str]:
    if not config_yaml:
        return []

    return [topic for t in config_yaml.get(BagConfigKeys.LOG_TOPICS, []) if (topic := t.get(BagConfigKeys.TOPIC))]

def merge_configs(base_config: Dict[str, Any], derived_config: Dict[str, Any]) -> Dict[str, Any]:
    # Derived wins on every repeated key; lists are replaced wholesale by {**base, **derived}.
    merged = {**base_config, **derived_config}
    # Exception: log_topics is a union keyed by the unique 'name' field (derived wins per name).
    base_topics = base_config.get(BagConfigKeys.LOG_TOPICS, [])
    derived_topics = derived_config.get(BagConfigKeys.LOG_TOPICS, [])
    if base_topics or derived_topics:
        topics_by_name = {t['name']: t for t in base_topics}
        topics_by_name.update({t['name']: t for t in derived_topics})
        merged[BagConfigKeys.LOG_TOPICS] = list(topics_by_name.values())
    return merged

def save_config(config_yaml: Dict[str, Any], save_path: str, config_file_out: str) -> str:
    config_file_out_path = os.path.join(save_path, config_file_out)
    with open(config_file_out_path, 'w') as outfile:
        yaml.dump(config_yaml, outfile, sort_keys=False)
    return config_file_out_path

def parse_args() -> Namespace:

    default_base_bag_config = f"{os.path.join(get_package_share_directory('avt_341'), 'parameters/bag_config/base_bag_config.yaml')}"

    parser = ArgumentParser(prog="ros2 run avt_341 vehicle_logging.py",
                            description="Records configured topics in a rosbag data structured.")

    parser.add_argument('config_file', type=str, help="Path to the logging config file. Will be merged with base_config_file.")
    parser.add_argument('save_dir', type=str, help="Path to the directory to save rosbag data.")
    parser.add_argument('--base_config_file', type=str, default=default_base_bag_config, help="Prefix for output rosbag name.")
    parser.add_argument('--save_prefix', type=str, default="AVT_341_DATALOG", help="Prefix for output rosbag name.")
    parser.add_argument('--bag_name_override', type=str, default="", help="If set, overrides created bag name.")
    parser.add_argument('--bag_format', type=str, default="", help="Customize bag file format. Possible values: sqlite3 | mcap. Pass empty string to use default format (sqlite3 for older versions)")
    parser.add_argument('--storage_config_file', type=str, default="", help="Yaml file containing storage options for bag. ONLY SUPPORTED IN ROS2.")
    parser.add_argument('--config_file_out', type=str, default="logging_config.yaml", help="Name to use for copied bag configuration file that will appear in output bag directory.")
    parser.add_argument('--vehicles_override', type=str, default="", help="Comma separated list of vehicles to use in symbols of bag log config. Leave blank for no override.")
    parser.add_argument('--debug_generate_output_config', action='store_true', help="Skip rosbag recording; only parse and merge the configs and write the merged output config file. Useful for testing the parsing and merging logic.")
    _args = parser.parse_args()

    if _args.bag_format and _args.bag_format not in ['sqlite3', 'mcap']:
        sys.exit(f"Invalid bag format [{_args.bag_format}]. Use no argument to select default type or explicitly select sqlite3 | mcap")

    # File naming constants
    if _args.bag_name_override:
        _args.save_name = _args.bag_name_override
    else:
        time_YYMMDD = datetime.now().strftime('%y%m%d')
        time_HHMMSS = datetime.now().strftime('%H%M%S')
        _args.save_name = f"{time_YYMMDD}_{_args.save_prefix}_{time_HHMMSS}"
    _args.save_path = os.path.join(_args.save_dir, _args.save_name)
    _args.vehicles = [v.strip() for v in _args.vehicles_override.split(',')] if _args.vehicles_override else []
    return _args


if __name__ == "__main__":

    args = parse_args()

    base_loader = BagConfigLoader(args.base_config_file, args.vehicles)
    if base_loader.in_error():
        sys.exit(base_loader.error)

    derived_loader = BagConfigLoader(args.config_file, args.vehicles, base_symbols=base_loader.symbols)
    if derived_loader.in_error():
        sys.exit(derived_loader.error)

    merged_config: Dict[str, Any] = merge_configs(base_loader.config_yaml, derived_loader.config_yaml)
    record_topics: List[str] = get_record_topics(merged_config)

    # Debug mode: skip recording, just write out the parsed/merged config for inspection.
    if args.debug_generate_output_config:
        os.makedirs(args.save_path, exist_ok=True)
        saved_path = save_config(merged_config, args.save_path, args.config_file_out)
        print(f"[debug_generate_output_config] Wrote merged config ({len(record_topics)} record topics) to {saved_path}")
        sys.exit(0)

    try:
        # Record rosbag using CLI
        command = (['ros2', 'bag', 'record']
                   + (['-s', args.bag_format] if args.bag_format else [])
                   + (['--storage-config-file', args.storage_config_file] if args.storage_config_file else [])
                   + ['-o', args.save_path, *record_topics])
        process = subprocess.Popen(command, stdout=subprocess.PIPE)

        # Copy the logging config to the save path
        while not os.path.isdir(args.save_path):
            time.sleep(0.1) # Wait for save path to be created

        save_config(merged_config, args.save_path, args.config_file_out)

        # Wait for logging to complete
        process.wait()
    except KeyboardInterrupt:
        process.send_signal(signal.SIGINT)
        process.wait()
        sys.exit(0)