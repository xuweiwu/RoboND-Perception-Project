## Project: Perception Pick & Place

---


**Required Steps for a Passing Submission:**
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  See the example `output.yaml` for details on what the output should look like.  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!


**Extra Challenges: Complete the Pick & Place**

7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

[//]: # (Image References)

[image1]: ./misc_images/cm_norm_1.jpg
[image2]: ./misc_images/cm_norm_2.jpg
[image3]: ./misc_images/cm_norm_3.jpg
[image4]: ./misc_images/screenshot_t1_objects.png
[image5]: ./misc_images/screenshot_t1_objects_clustered.png
[image6]: ./misc_images/screenshot_t2_objects.png
[image7]: ./misc_images/screenshot_t2_objects_clustered.png
[image8]: ./misc_images/screenshot_t3_objects.png
[image9]: ./misc_images/screenshot_t3_objects_clustered.png


## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points 

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

Filtering and RANSAC plane fitting are implemented in `pr2_robot/scripts/perception_project.py`, line 51 to 94. The resulted point clouds of the objects are published to `/pcl_cobjects` topic, and can be seen from the following screenshots with respect to individual test worlds.

test world 1:
![alt text][image7]
test world 2:
![alt text][image9]
test world 3:
![alt text][image11]

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.

Euclidean clustering for segmentation is implemented in `pr2_robot/scripts/perception_project.py`, line 96 to 116. The resulted point clouds of the clusters are published to `/pcl_clusters` topic, and can be seen from the following screenshots with respect to individual test worlds. Note that in test world 3 the cluster for glue cannot be found, because most part of it is hidden by the book in the camera view.

test world 1:
![alt text][image8]
test world 2:
![alt text][image10]
test world 3:
![alt text][image12]

#### 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

Both `compute_color_histograms()` and `compute_normal_histograms()` functions in `sensor_stick/src/sensor_stick/features.py` are completed to extract features of color and normal vectors. 40 sample point clouds are recorded for each object in the individual pick list (see `sensor_stick/scripts/capture_features.py`). The SVMs are then trained using linear kernels (see `sensor_stick/scripts/train_svm.py`). The normalized confusion matrices of the trained models are shown below.

pick list 1:
![alt text][image7]
pick list 2:
![alt text][image9]
pick list 3:
![alt text][image11]

Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd


### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

And here's another image! 
![demo-2](https://user-images.githubusercontent.com/20687560/28748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  



