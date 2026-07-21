#########################################################################################################
# check_vehicle_log.py                                                                                  #
# ----------------------------------------------------------------------------------------------------- #
# Tool to check if a rosbag contains all of the topics defined in a vehicle logging config file.        #
#########################################################################################################
import os
import yaml
import argparse


def check_vehicle_log(bag_path, config_file_name):
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
    with open(os.path.join(bag_path, config_file_name)) as config_file:
        config_yaml = yaml.safe_load(config_file)
    config_topics = {topic for t in config_yaml.get('log_topics', []) if (topic := t.get('topic'))}

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
    parser.add_argument('--log_config_name', type=str, default="logging_config.yaml", help="The filename of the logging config file to look for in each bag directory")
    args = parser.parse_args()
    
    missing_topics = check_vehicle_log(args.bag_path, args.log_config_name)

    # Print results
    if len(missing_topics) == 0:
        print("All topics in the config file are present in the bag.")
    else:
        print("The following topics are missing from the bag:")
        for topic in missing_topics:
            print(f"\t- {topic}")

if __name__ == "__main__":
    main()