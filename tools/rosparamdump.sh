#!/bin/bash

for node in $(ros2 node list); do
  # Prepend the default namespace to the node name
  echo "Interface information for node: $node"
  ros2 param list $node
  echo "-------------------------"
done
