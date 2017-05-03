# Rotation matrix convert and publish

- This package is a ROS package that could transform your 3x4 transformation matrix to position & quaternion, then publish using tf.


## Usage
- launch the package:
```
roslaunch pose_transformation convert.launch
```
- Arguments : 
```
<arg name="tcp_link" default="tcp_link"/>
<arg name="camera_link" default="camera_link"/>
<arg name="publish_tf" default="true"/>
<arg name="matrix_file" default="rotation.txt" />
```
- **tcp_link** and **camera_link** are the tcp and camera frame you want to call in tf tree.
- **publish_tf** is set to true by default. If you set to **false**, it will only convert the rotation matrix to quaternion,.
- **matrix_file** is located inside the ```/data``` directory. Now, I use **rotation.txt** as an example. You could store all your tcp to camera matrix inside ```/data``` and choose which to convert and publish
