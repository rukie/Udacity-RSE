#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from sensor_stick.features import *
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

import pcl
from pcl_helper import *

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

# Function to search list of dictionaries and return a selected value in selected dictionary
def search_dictionaries(key1, value1, key2, list_of_dictionaries):
    selected_dic = [element for element in list_of_dictionaries if element[key1] == value1][0]
    selected_val = selected_dic.get(key2)
    return selected_val

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    ###########################################################
    # TODO: Statistical Outlier Filtering
    outlier_filter = cloud.make_statistical_outlier_filter()
    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(25)
    # Set threshold scale_factor (all points further than this standard deviation are filtered out)
    x = 0.35
    # Any point with a mean distance larger than global (mean distance+x+std_dev)
    outlier_filter.set_std_dev_mul_thresh(x)
    # Finally call th filter function for magic
    cloud_outlier_filtered = outlier_filter.filter()

    # TODO: Voxel Grid Downsampling
    # Voxel Grid filter
    # Create a VoxelGrid filter object for input point cloud
    vox = cloud_outlier_filtered.make_voxel_grid_filter()
    # Choose a voxel (aka leaf) size
    # Note: 1 is a poor choice of leaf size
    LEAF_SIZE = 0.004
    # Set the leaf size
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    # Call the filter function
    cloud_vox_filtered = vox.filter()
    # TODO: PassThrough Filter
    # PassThrough filter
    # Create a PassThrough filter object
    passthrough = cloud_vox_filtered.make_passthrough_filter()
    #Assign axis and range to the passthrough filter object
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.55
    axis_max = 0.85
    passthrough.set_filter_limits(axis_min, axis_max)
    #Use the filter function to obtain the filtered cloud
    cloud_filtered = passthrough.filter()

    # PassThrough filter
    # Create a PassThrough filter object
    passthrough = cloud_filtered.make_passthrough_filter()
    #Assign axis and range to the passthrough filter object
    filter_axis = 'x'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.35
    axis_max = 10
    passthrough.set_filter_limits(axis_min, axis_max)
    #Use the filter function to obtain the filtered cloud
    cloud_filtered = passthrough.filter()

    # TODO: RANSAC Plane Segmentation
    # RANSAC plane segmentation
    # Create segmentation of the object
    seg = cloud_filtered.make_segmenter()
    # Set the model you wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    # Maximum distance for a point to be considered fitting the model
    max_distance = 0.006
    seg.set_distance_threshold(max_distance)
    # Call the segment function to obtain set of inlier indicies and model coefficients
    inliers, coefficients = seg.segment()
    extracted_inliers = cloud_filtered.extract(inliers, negative=True)
    extracted_outliers = cloud_filtered.extract(inliers, negative=False)

    #extracted_outliers = cloud_filtered - extracted_inliers
    # TODO: Extract inliers and outliers
    # Extract inliers
    extracted_inliers = cloud_filtered.extract(inliers, negative=True)
    # Extract outliers
    extracted_outliers = cloud_filtered.extract(inliers, negative=False)

    # TODO: Euclidean Clustering
    # Apply function to convert XYZRGB to XYZ
    white_cloud = XYZRGB_to_XYZ(extracted_inliers)
    # Create a K-D Tree
    tree = white_cloud.make_kdtree()
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold
    # as well as minimum and maximum cluster size (in points)
    #
    ec.set_ClusterTolerance(0.01)
    ec.set_MinClusterSize(200)
    ec.set_MaxClusterSize(8000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0], 
											white_cloud[indice][1],
											white_cloud[indice][2],
											rgb_to_float(cluster_color[j]) ])
    # Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    ros_cloud_outlier_fltr = pcl_to_ros(cloud_outlier_filtered)
    ros_cloud_vox_fltr = pcl_to_ros(cloud_vox_filtered)
    ros_cloud_objects = pcl_to_ros(extracted_inliers)
    ros_cloud_table = pcl_to_ros(extracted_outliers)
    ros_cloud_cluster = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    pcl_outlier_filer_pub.publish(ros_cloud_outlier_fltr)
    pcl_vox_filter_pub.publish(ros_cloud_vox_fltr)
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cloud_cluster)

# Exercise-3 TODOs:

	# Initialize variables
    detected_objects_labels = []
    detected_objects = []
    labeled_features = []

    # Classify the clusters! (loop through each detected cluster one at a time)
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = extracted_inliers.extract(pts_list)
        # Convert cluster to ROS data
        cloud = pcl_to_ros(pcl_cluster)
        # Compute the associated feature vector
        chists_rgb = compute_color_histograms(cloud, using_hsv=False)
        chists_hsv = compute_color_histograms(cloud, using_hsv=True)
        normals = get_normals(cloud)
        nhists = compute_normal_histograms(normals)

        cloud_arr = ros_to_pcl(cloud).to_array()
        cloud_XYZ = np.asarray(XYZRGB_to_XYZ(cloud_arr))
        #dimensions = compute_bounding_box_histograms(cloud_XYZ)
        length, width, height, volume = compute_bounding_box_histograms(cloud_XYZ)
        #feature = np.concatenate((chists_rgb, chists_hsv, nhists, dimensions))
        #feature = np.concatenate((chists_hsv, nhists, dimensions))
        feature = np.concatenate((chists_hsv, nhists, length, width, height, volume))
        labeled_features.append([feature, index])

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += 0.4
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = cloud #ros_cluster
        detected_objects.append(do)

    # Publish the list of detected objects
    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    detected_objects_pub.publish(detected_objects)

#    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
#    # Could add some logic to determine whether or not your object detections are robust
#    # before calling pr2_mover()
    if len(detected_objects)>0:
        try:
            pr2_mover(detected_objects, ros_cloud_table)
        except rospy.ROSInterruptException:
            pass
    else:
        rospy.logwarn('No objects detected.')
# function to load parameters and request PickPlace service
def pr2_mover(object_list, ros_cloud_table):

    # TODO: Initialize variables
    TEST_SCENE_NUM = std_msgs.msg.Int32()
    TEST_SCENE_NUM.data = 3
    OBJECT_NAME = std_msgs.msg.String()
    OBJECT_GROUP = std_msgs.msg.String()
    PICK_POSE = geometry_msgs.msg.Pose()
    PLACE_POSE = geometry_msgs.msg.Pose()
    WHICH_ARM = std_msgs.msg.String()
    yaml_dict_list = []

    # TODO: Get/Read parameters
    # get parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')
    # TODO: Parse parameters into individual variables

    # TODO: Rotate PR2 in place to capture side tables for the collision map
#    # Rotate Right
#    pr2_base_mover_pub.publish(-1.57)
#    rospy.sleep(15.0)
#    # Rotate Right
#    pr2_base_mover_pub.publish(1.57)
#    rospy.sleep(30.0)
#    # Rotate Right
#    pr2_base_mover_pub.publish(-1.57)
#    rospy.sleep(15.0)

    # Calculate detected objects centroids
    labels = []
    centroids = [] #tuples (x,y,z)
    for object in object_list:
        labels.append(object.label)
        points_arr = ros_to_pcl(object.cloud).to_array()
        centroids.append(np.mean(points_arr, axis=0)[:3])

    # TODO: Loop through the pick list
    #for i in enumerate(object_list_param):
    for i in range(0, len(object_list_param)):
        # TODO: Get the PointCloud for a given object and obtain it's centroid
        OBJECT_NAME.data = object_list_param[i]['name']
        OBJECT_GROUP.data = object_list_param[i]['group']

        # TODO: Create 'place_pose' for the object
        try:
            index = labels.index(OBJECT_NAME.data)
        except ValueError:
            print("Object not detected: {}".format(OBJECT_NAME.data))
            continue
        
        PICK_POSE.position.x = np.asscalar(centroids[index][0])
        PICK_POSE.position.y = np.asscalar(centroids[index][1])
        PICK_POSE.position.z = np.asscalar(centroids[index][2])

        position = search_dictionaries('group', OBJECT_GROUP.data, 'position', dropbox_param)
        PLACE_POSE.position.x = position[0]
        PLACE_POSE.position.y = position[1]
        PLACE_POSE.position.z = position[2]

        # TODO: Assign the arm to be used for pick_place
        WHICH_ARM.data = search_dictionaries('group', OBJECT_GROUP.data, 'name', dropbox_param)
        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_dict = make_yaml_dict(TEST_SCENE_NUM, WHICH_ARM, OBJECT_NAME, PICK_POSE, PLACE_POSE)
        yaml_dict_list.append(yaml_dict)
        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')


        # Publish collision avoidance points
        ros_cloud_collision = ros_cloud_table
        #ros_cloud_collision = pcl_to_ros(cloud_collision)
        pr2_3d_map_points_pub.publish(ros_cloud_collision)

#        try:
#            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

#            # TODO: Insert your message variables to be sent as a service request
#            resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)
#            #resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)
#            print ("Response: ",resp.success)

#        except rospy.ServiceException, e:
#            print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file

    send_to_yaml("output_"+str(TEST_SCENE_NUM.data) + ".yaml", yaml_dict_list)

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous = True)
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_outlier_filer_pub = rospy.Publisher("/pcl_outlier_fltr", PointCloud2, queue_size=1)
    pcl_vox_filter_pub = rospy.Publisher("/pcl_vox_fltr", PointCloud2, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)
    pr2_base_mover_pub = rospy.Publisher("/pr2/world_joint_controller/command", Float64, queue_size=10)
    pr2_3d_map_points_pub = rospy.Publisher("/pr2/3D_map/points", PointCloud2, queue_size=1)

    # TODO: Load Model From disk
    model = pickle.load(open('model_3.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
