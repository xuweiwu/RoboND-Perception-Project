#!/usr/bin/env python

import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

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

    # Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)
    rospy.loginfo('Number of points in raw cloud: {}'.format(pcl_data.width))

    # Statistical Outlier Filtering
    outlier_filter = pcl_data.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(50)
    x = 1.0
    outlier_filter.set_std_dev_mul_thresh(x)
    cloud_filtered = outlier_filter.filter()
    rospy.loginfo('Number of points after statistical outlier filtering: {}'.format(cloud_filtered.width))

    # Voxel Grid Downsampling
    vox = cloud_filtered.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()
    rospy.loginfo('Number of points after downsampling: {}'.format(cloud_filtered.width))

    # PassThrough Filter
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
    rospy.loginfo('Number of points after passthrough filtering: {}'.format(cloud_filtered.width))

    # RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()

    # Extract inliers and outliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)
    rospy.loginfo('Number of points in object cloud after segmentation: {}'.format(cloud_table.width))

    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(25000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    rospy.loginfo('Number of objects after clustering: {}'.format(len(cluster_indices)))
    
    # Create Cluster-Mask Point Cloud to visualize each cluster separately
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

    # Convert PCL data to ROS messages    
    # ros_table_cloud = pcl_to_ros(cloud_table)
    ros_objects_cloud = pcl_to_ros(cloud_objects)
    ros_clustered_objects_cloud = pcl_to_ros(cluster_cloud)

    # Publish ROS messages    
    # pcl_table_pub.publish(ros_table_cloud)
    pcl_objects_pub.publish(ros_objects_cloud)
    pcl_clustered_objects_pub.publish(ros_clustered_objects_cloud)
    
    detected_objects_list = []
    detected_objects_labels = []
    # Classify the clusters! (loop through each detected cluster one at a time)
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)

        # Compute the associated feature vector
        ros_cluster = pcl_to_ros(pcl_cluster)
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .3
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects_list.append(do)
    
    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects_list)


    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects_list)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # Initialize variables
    test_scene_num = Int32()
    test_scene_num.data = 3
    object_name = String()
    arm_name = String()
    box2arm = {'green':'right', 'red':'left'}
    pick_pose = Pose()
    place_pose = Pose()

    # Get/Read parameters of pick list
    object_list_param = rospy.get_param('/object_list')

    # Get/Read parameters of drop box
    drop_box_param = rospy.get_param('/dropbox')

    # Get labels and compute centroids of detected objects
    labels = []
    centroids = []
    for do in object_list:
        labels.append(do.label)
        points_arr = ros_to_pcl(do.cloud).to_array()
        tmp = np.mean(points_arr, axis=0)[:3]
        centroids.append([np.asscalar(tmp[0]), np.asscalar(tmp[1]), np.asscalar(tmp[2])])
    
    # Rotate PR2 in place to capture side tables for the collision map

    # Loop through the pick list
    dict_list = []  
    for i in range(0, len(object_list_param)):

        # Parse parameters into individual variables
        object_name.data = object_list_param[i]['name']
        object_group = object_list_param[i]['group']

        # Get the PointCloud for a given object and obtain it's centroid
        object_centroid = []
        for centroid, label in zip(centroids, labels):
            if label == object_name.data:
                object_centroid = centroid
        # Skip to the next object of the pick list, if it is not found in the list of detected objects
        if not object_centroid:
           print "Object %s is not found!"%object_name.data
           continue
        pick_pose.position.x, pick_pose.position.y, pick_pose.position.z = object_centroid

        # Create 'place_pose' for the object
        db_position = next(db['position'] for db in drop_box_param if db['group'] == object_group)
        place_pose.position.x, place_pose.position.y, place_pose.position.z = db_position
        
        # Assign the arm to be used for pick_place
        arm_name.data = box2arm[object_group]

        # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # Insert your message variables to be sent as a service request
            resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # Output your request parameters into output yaml file
    send_to_yaml('output3.yaml', dict_list)

if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('pr2_perception', anonymous=True)

    # Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # Create Publishers    
    # pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/pcl_cobjects", PointCloud2, queue_size=1)
    pcl_clustered_objects_pub = rospy.Publisher("/pcl_clustered_objects", PointCloud2, queue_size=1)    
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)    

    # Load Model From disk
    model = pickle.load(open('model3.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
    	rospy.spin()