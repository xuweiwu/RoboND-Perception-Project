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

[image1]: ./misc_images/screenshot_t1_objects.png
[image2]: ./misc_images/screenshot_t2_objects.png
[image3]: ./misc_images/screenshot_t3_objects.png
[image4]: ./misc_images/screenshot_t1_objects_clustered.png
[image5]: ./misc_images/screenshot_t2_objects_clustered.png
[image6]: ./misc_images/screenshot_t3_objects_clustered.png
[image7]: ./misc_images/cm_norm_1.jpg
[image8]: ./misc_images/cm_norm_2.jpg
[image9]: ./misc_images/cm_norm_3.jpg


## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points 

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

Filtering and RANSAC plane fitting are implemented in `pr2_robot/scripts/perception_project.py`, line 51 to 94. The resulted point clouds of the objects are published to `/pcl_cobjects` topic, and can be seen from the following screenshots with respect to individual test worlds.

test world 1:
![alt text][image1]
test world 2:
![alt text][image2]
test world 3:
![alt text][image3]

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.

Euclidean clustering for segmentation is implemented in `pr2_robot/scripts/perception_project.py`, line 96 to 116. The resulted point clouds of the clusters are published to `/pcl_clusters` topic, and can be seen from the following screenshots with respect to individual test worlds. Note that in test world 3 the cluster for glue cannot be found, because most part of it is hidden by the book in the camera view.

test world 1:
![alt text][image4]
test world 2:
![alt text][image5]
test world 3:
![alt text][image6]

#### 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

Both `compute_color_histograms()` and `compute_normal_histograms()` functions in `sensor_stick/src/sensor_stick/features.py` are completed to extract features of color and normal vectors. 40 sample point clouds are recorded for each object in the individual pick lists (see `sensor_stick/scripts/capture_features.py`). The SVMs are then trained using linear kernels (see `sensor_stick/scripts/train_svm.py`). The normalized confusion matrices of the trained models are shown below.

pick list 1:

order | name | group | dropbox
--- | --- | --- | ---
1 | biscuits | green | right
2 | soap | green | right
3 | soap2 | red | left

![alt text][image7]

pick list 2:
 
order | name | group | dropbox
--- | --- | --- | ---
1 | biscuits | green | right
2 | soap | green | right
3 | book | red | left
4 | soap2 | red | left
5 | glue | red | left

![alt text][image8]

pick list 3:
    
order | name | group | dropbox
--- | --- | --- | ---
1 | sticky_notes | red | left
2 | book | red | left
3 | snacks | green | right
4 | biscuits | green | right
5 | eraser | red | left
6 | soap2 | green | right
7 | soap | green | right
8 | glue | red | left    

![alt text][image9]

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

The implemented perception pipeline can correctly identify 100% (3/3) of objects in test world 1, 100% (5/5) in test world 2, and 87.5% (7/8) in test world 3. The output yaml files can be found in `output_results` and the screenshots of label markers in RViz are already shown above.

### Conclusion

By using filtering and plane segmentation techniques, the point clouds of the objects can be successfully extracted from the raw point cloud of the environment. However, this is mainly due to the highly simplified environment model, where the point cloud of the table can be easily filtered out. The Euclidean clustering technique works fine, but it might fail when two objects stay too close to each other, or when an object is hidden behind other objects. One possible solution is to perform object recognition and  update the list of detected objects after finishing a single pick and place task. Another problem is that during the pick and place process the original positions of the objects might be changed due to collisions with the robot arms. This can be overcome by means of publishing an additional point cloud for 3D collision map, which makes the robot be aware of collidable objects.
