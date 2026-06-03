#########################################################################################################
# check_vehicle_log.py                                                                                  #
# ----------------------------------------------------------------------------------------------------- #
# Tool to check if a rosbag contains all of the topics defined in a vehicle logging config file.        #
#########################################################################################################
import os
import sys
import yaml
import argparse
from ament_index_python.packages import get_package_share_directory
try:
    avt_341_path = get_package_share_directory('avt_341')
    sys.path.append(os.path.join(avt_341_path, "launch"))
    from vehicle_logging import BagConfigLoader
except Exception as e:
    raise e


def check_vehicle_log(bag_path, config_file_name, bag_format="sqlite3", vehicles_override="", exclude_occupancy_grid=False):
    vehicles = [v.strip() for v in vehicles_override.split(',')] if vehicles_override else []

    # Parse input bag metadata
    meta = {}
    metadata_file = os.path.join(bag_path,"metadata.yaml")
    with open(metadata_file) as meta_file:
        try:
            meta = yaml.safe_load(meta_file)
        except yaml.YAMLError as exc:
            print(exc)
            exit(0)
    
    # Read available topics
    topics = {topic['topic_metadata']['name']: topic for topic in meta['rosbag2_bagfile_information']['topics_with_message_count']}

    # Load logging config
    bag_config_loader = BagConfigLoader(os.path.join(bag_path,config_file_name), vehicles)
    config_topics = set(bag_config_loader.get_record_topics(exclude_occupancy_grid))

    # Compare topics in bag with those in config
    missing_topics = []
    for topic in config_topics:
        if topic not in topics.keys():
            missing_topics.append(topic)
        elif topics[topic]['message_count'] == 0:
            missing_topics.append(topic)
    
    return missing_topics

def main():
    parser = argparse.ArgumentParser(description='Verify if a rosbag contains all of the topics defined in a vehicle logging config file.')
    parser.add_argument('bag_path', type=str, help='Path to the ROS 2 bag directory')
    parser.add_argument('--bag_format', type=str, default="sqlite3", help="Bag file format.\nValues: sqlite3 (default) | mcap")
    parser.add_argument('--vehicles_override', type=str, default="", help="Comma seperated list of vehicles to use in symbols of bag log config. Leave blank for no override.")
    parser.add_argument('--exclude_occupancy_grid', type=bool, default=False, help="If set, occupancy grids will be excluded from the bag file. Inclusion of occupancy grid may cause large bag file sizes if no compression used.")
    parser.add_argument('--log_config_name', type=str, default="logging_config.yaml", help="The filename of the logging config file to look for in each bag directory")
    args = parser.parse_args()
    
    missing_topics = check_vehicle_log(args.bag_path, args.log_config_name, args.bag_format, args.vehicles_override, args.exclude_occupancy_grid)

    # Print results
    if len(missing_topics) == 0:
        print("All topics in the config file are present in the bag.")
    else:
        print("The following topics are missing from the bag:")
        for topic in missing_topics:
            print(f"\t- {topic}")

if __name__ == "__main__":
    main()