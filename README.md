# Sahayak - An Autonomous Covid Aid-Bot

## Python Requirements
``` shell
pip install -r requirement.txt
```
## Required ROS Packages
``` shell
xargs sudo apt install < ros_requirements.txt
```
## CAD Model of Sahayak 
* URDF and mesh files of the SolidWorks Assembly are generated using the [sw_urdf_exporter](http://wiki.ros.org/sw_urdf_exporter).
* After installing the exporter, the `SW2URDF` add-in gets appended in the add-ins list of SolidWorks. Further detailed instructions are documented [here](http://wiki.ros.org/sw_urdf_exporter/Tutorials/Export%20an%20Assembly).
<p align="center">
<img src="https://user-images.githubusercontent.com/64797216/125199209-c0034e00-e282-11eb-9d1a-38fca513cba6.gif" width="400">
</p>


## Result
### ROS Controls:
#### Teleoperation using Keyboard inputs

<p align="center">
<img src="https://user-images.githubusercontent.com/64685403/121785722-15dbcc00-cbd9-11eb-86ee-6b86179110eb.gif" width="500">
</p>

### Visual Odometry:
Implemented Visual Odometry pipeline using the following 
* 2D to 2D Motion Estimation Algorithm

<p align="center">
<img src="https://user-images.githubusercontent.com/64685403/121784729-37d25000-cbd3-11eb-9314-2a5fbb041a9a.png" width="400">
</p>

* 3D to 2D Motion Estimation Algorithm
[Result Video](https://drive.google.com/file/d/173dWtAgQprP5A2eiIHDtA-GP9SdjEXO3/view?usp=sharing)


<p align="center">
<img src="https://user-images.githubusercontent.com/64685403/121784831-e4accd00-cbd3-11eb-8202-94722c2689f7.png" width="400">
</p>


### Mapping:

#### GMapping
[GMapping](http://wiki.ros.org/gmapping) is a ROS Package used creating a 2D occupancy grid map of an enviroment using a 2D LiDAR, tf and Odometry data. 

To map the default enviroment using GMapping run the below commands in seperate terminals:
 
* ``` roslaunch sahayak_mapping gmap-launch_all.launch```

* ``` roslaunch sahayak_navigation scan_matcher.launch```

<p align="center">
<img src="https://user-images.githubusercontent.com/69981745/125203882-3a8a9880-e298-11eb-830b-166dbaab8e16.gif" width="500">
</p>

#### Hector SLAM
[Hector SLAM](http://wiki.ros.org/hector_slam) is a ROS package which is used for creating a 2D occupancy grid map of an enviroment using a 2D LiDAR, tf data and Odometry data (Optional).
<p align="center">
<img src="https://user-images.githubusercontent.com/64685403/121784874-276ea500-cbd4-11eb-948d-58333b687bce.png" width="350">
</p>

#### RTAB (Real Time Appearance Based) Map:
[RTAB-Map](http://wiki.ros.org/rtabmap_ros) is a ROS Package which uses a RGB-D camera to generate a 3D map of an enviroment.

To map the default enviroment using RTAB Map run:
* ```roslaunch sahayak_mapping rtab-mapping.launch ```
<p align="center">
<img src="https://user-images.githubusercontent.com/64797216/121785126-98628c80-cbd5-11eb-9d54-c349228d4ee7.gif" width="500">
</p>

### ROS Navigation Stack:

#### AMCL 
Sahayak was localised in a map using Adaptive Monte Carlo Localisation (AMCL) which uses adaptive particle filters.
<p align="center">
<img src="https://user-images.githubusercontent.com/69981745/125203696-2befb180-e297-11eb-93c2-7971d5cc969d.gif" width="600">
</p>


#### Path Planning
Path Planning involves finding a lowest cost path based on 2D Local and Global [Costmaps](http://wiki.ros.org/costmap_2d).
##### ROS Packages using in Path Planning:
* [AMCL](http://wiki.ros.org/amcl)
* [costmap_2d](http://wiki.ros.org/costmap_2d)
* [dwa_local_planner](http://wiki.ros.org/dwa_local_planner)
* [global_planner](http://wiki.ros.org/global_planner)

Run ``` roslaunch sahayak launch_all.launch ``` in a terminal to launch Path Planning launch file. Use `2D Nav Goal` in RVIZ to give a goal to Sahayak.

<p align="center">
<img src="https://user-images.githubusercontent.com/69981745/125204893-0bc2f100-e29d-11eb-83c9-d00fbe9d5126.gif" width="600">
</p>


