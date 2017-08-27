#!/usr/bin/env python

# Import modules
# import numpy as np
# import sklearn
# from sklearn.preprocessing import LabelEncoder
# import pickle
# from sensor_stick.srv import GetNormals
# from sensor_stick.features import compute_color_histograms
# from sensor_stick.features import compute_normal_histograms
# from visualization_msgs.msg import Marker
# from sensor_stick.marker_tools import *
# from sensor_stick.msg import DetectedObjectsArray
# from sensor_stick.msg import DetectedObject
# from sensor_stick.pcl_helper import *

from pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
	get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
	return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
	yaml_dict = {}
	yaml_dict["test_scene_num"] = test_scene_num.data
	yaml_dict["arm_name"]  = arm_name.data
	yaml_dict["object_name"] = object_name.data
	yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
	yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
	return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
	data_dict = {"object_list": dict_list}
	with open(yaml_filename, 'w') as outfile:
		yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)
    print('Number of points in raw cloud: {}'.format(pcl_data.width))

    # TODO: Statistical Outlier Filtering
    outlier_filter = pcl_data.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(50)
    x = 1.0
    outlier_filter.set_std_dev_mul_thresh(x)
    cloud_filtered = outlier_filter.filter()
    print('Number of points after statistical outlier filtering: {}'.format(cloud_filtered.width))

    # TODO: Voxel Grid Downsampling
    vox = cloud_filtered.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()
    print('Number of points after downsampling: {}'.format(cloud_filtered.width))

    # TODO: PassThrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.0
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'x'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.3
    axis_max = 1.3
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()
    print('Number of points after passthrough filtering: {}'.format(cloud_filtered.width))

    # TODO: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()

    # TODO: Extract inliers and outliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)
    print('Number of points in object cloud after segmentation: {}'.format(cloud_table.width))

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(25000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    print('Number of objects after clustering: {}'.format(len(cluster_indices)))

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
    	for i, indice in enumerate(indices):
    		color_cluster_point_list.append([white_cloud[indice][0],
    			white_cloud[indice][1],
    			white_cloud[indice][2],
    			rgb_to_float(cluster_color[j])])
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    ros_table_cloud = pcl_to_ros(cloud_table)

    # TODO: Publish ROS messages
    pcl_world_pub.publish(pcl_msg)
    pcl_objects_pub.publish(ros_cluster_cloud)
    pcl_table_pub.publish(ros_table_cloud)
    print('Point clouds published!')    

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)

        # Grab the points for the cluster

        # Compute the associated feature vector

        # Make the prediction

        # Publish a label into RViz

        # Add the detected object to the list of detected objects.

    # Publish the list of detected objects

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    # try:
    # 	pr2_mover(detected_objects_list)
    # except rospy.ROSInterruptException:
    # 	pass

# function to load parameters and request PickPlace service
# def pr2_mover(object_list):

    # TODO: Initialize variables

    # TODO: Get/Read parameters

    # TODO: Parse parameters into individual variables

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list

        # TODO: Get the PointCloud for a given object and obtain it's centroid

        # TODO: Create 'place_pose' for the object

        # TODO: Assign the arm to be used for pick_place

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format

        # Wait for 'pick_place_routine' service to come up
        # rospy.wait_for_service('pick_place_routine')

        # try:
        # 	pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

        #     # TODO: Insert your message variables to be sent as a service request
        #     resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

        #     print ("Response: ",resp.success)

        # except rospy.ServiceException, e:
        # 	print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('pr2_perception', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_world_pub = rospy.Publisher("/pcl_world", PointCloud2, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)

    # TODO: Load Model From disk

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
    	rospy.spin()