# diagram.py
from diagrams import Cluster, Diagram, Edge, Node
from diagrams.aws.compute import EC2
from diagrams.aws.database import RDS
from diagrams.aws.network import ELB
from diagrams.generic.network import Switch
from diagrams.generic.storage import Storage
from diagrams.generic.os import Ubuntu
from diagrams.generic.os import LinuxGeneral
from diagrams.generic.compute import Rack
from diagrams.custom import Custom
from diagrams.generic.blank import Blank
from diagrams.generic.device import Mobile

graph_attr_custom = {
	"layout": "dot", #  Use one of: circo dot fdp neato nop nop1 nop2 osage patchwork sfdp twopi
	"direction": "LR",
	"splines": "compound" #ortho, spline, compound
}

with Diagram("NATO AVT 341 Autonomy Stack", show=True, graph_attr=graph_attr_custom):
	lidar_sensor = Node("LIDAR Sensor")
	avt_341_points = Node("avt341/points")
	odometry_source = Node("Odometry")
	avt_341_odom = Node("avt_341/odometry")
	perception_node = Node("avt_341_perception_node")
	avt_341_occupancy = Node("avt_341/occupancy_grid")
	avt_341_segmentation = Node("avt_341/segmentation_grid")
	new_waypoint = Node("avt_341/new_waypoints")
	global_path_node = Node("avt_341_global_path_node")
	path = Node("avt_341/global_path")
	waypoints = Node("avt_341/waypoints")
	current_waypoint = Node("avt_341/current_waypoint")
	distance = Node("avt_341/distance_to_current_waypoint")
	goal_reached = Node("avt_341/goal_reached")

	#color list: https://www.graphviz.org/doc/info/colors.html
	pub = Edge(color="blue", style="solid")
	sub = Edge(color="green", style="solid")

	lidar_sensor >> pub >> avt_341_points
	odometry_source >> pub >> avt_341_odom
	avt_341_points >> sub >> perception_node
	avt_341_odom >> sub >> perception_node
	perception_node >> pub >> avt_341_occupancy
	perception_node >> pub >> avt_341_segmentation
	avt_341_odom >> sub >> global_path_node
	avt_341_occupancy >> sub >> global_path_node
	avt_341_segmentation >> sub >> global_path_node
	new_waypoint >> sub >> global_path_node
	global_path_node >> pub >> path
	global_path_node >> pub >> waypoints
	global_path_node >> pub >> current_waypoint
	global_path_node >> pub >> distance
	global_path_node >> pub >> goal_reached


with Diagram("NATO AVT 341 Autonomy Stack Occlusion", show=True, graph_attr=graph_attr_custom):
	lidar_sensor = Node("LIDAR Sensor")
	avt_341_points = Node("avt341/points")
	odometry_source = Node("Odometry")
	avt_341_odom = Node("avt_341/odometry")
	perception_node = Node("avt_341_perception_node")
	avt_341_occupancy = Node("avt_341/occupancy_grid")
	avt_341_segmentation = Node("avt_341/segmentation_grid")
	new_waypoint = Node("avt_341/new_waypoints")
	global_path_node = Node("avt_341_global_path_node")
	path = Node("avt_341/global_path")
	waypoints = Node("avt_341/waypoints")
	current_waypoint = Node("avt_341/current_waypoint")
	distance = Node("avt_341/distance_to_current_waypoint")
	goal_reached = Node("avt_341/goal_reached")

	lidar_occlusion_node = Node("avt_341_lidar_occlusion_node")
	avt_341_occ_points = Node("avt_341/occ_points")

	#color list: https://www.graphviz.org/doc/info/colors.html
	pub = Edge(color="blue", style="solid")
	sub = Edge(color="green", style="solid")

	lidar_sensor >> pub >> avt_341_points
	odometry_source >> pub >> avt_341_odom
	# avt_341_points >> sub >> perception_node
	avt_341_points >> sub >> lidar_occlusion_node
	lidar_occlusion_node >> pub >> avt_341_occ_points
	avt_341_occ_points >> sub >> perception_node
	avt_341_odom >> sub >> perception_node
	perception_node >> pub >> avt_341_occupancy
	perception_node >> pub >> avt_341_segmentation
	avt_341_odom >> sub >> global_path_node
	avt_341_occupancy >> sub >> global_path_node
	avt_341_segmentation >> sub >> global_path_node
	new_waypoint >> sub >> global_path_node
	global_path_node >> pub >> path
	global_path_node >> pub >> waypoints
	global_path_node >> pub >> current_waypoint
	global_path_node >> pub >> distance
	global_path_node >> pub >> goal_reached
