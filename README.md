# Program a Real Self-Driving Car
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)
#### by Thomas Dunlap

### Rubric:
* Carla (Udacity's self-driving car) must successfully navigate around a track.

### Code Structure:
![Code Structure](./imgs/code_structure.png)

### Waypoint Updater Node

This node publishes a specific number of waypoints ahead of the vehicle with the appropriate target velocities relative to traffic lights and obstacles.

The node subscribes to topics:

    /base_waypoints
    /current_pose

and publishes a list of waypoints to:

    /final_waypoints

The /base_waypoints topic publishes a list of all waypoints for the track, so this list includes waypoints both before and after the vehicle (note that the publisher for /base_waypoints publishes only once). For this step in the project, the list published to /final_waypoints should include just a fixed number of waypoints currently ahead of the vehicle:

The first waypoint in the list published to /final_waypoints should be the first waypoint that is currently ahead of the car.
The total number of waypoints ahead of the vehicle that should be included in the /final_waypoints list is provided by the LOOKAHEAD_WPS variable in waypoint_updater.py.


From here, the messages contain a header and a Waypoint list named waypoints. Each waypoint has pose and twist data, where twist.twist data contains 3D linear and angular velocities.

### Drive-By-Wire Node

Once messages are being published to /final_waypoints, the vehicle's waypoint follower publishes twist commands to the /twist_cmd topic. The goal of the drive-by-wire node (dbw_node.py) is to subscribe to /twist_cmd and use various controllers to provide appropriate throttle, brake, and steering commands. These commands can then be published to the following topics:

    /vehicle/throttle_cmd
    /vehicle/brake_cmd
    /vehicle/steering_cmd

A safety driver may take control of the car during testing, so you will need to be mindful of DBW status. The DBW status can be found by subscribing to /vehicle/dbw_enabled.

DBW can be toggled by clicking "Manual" in the simulator GUI.


Within the twist controller package, you will find the following:

    dbw_node.py

This python file implements the dbw_node publishers and subscribers. You will need to write ROS subscribers for the /current_velocity, /twist_cmd, and /vehicle/dbw_enabled topics. This file also imports the Controller class from twist_controller.py which will be used for implementing the necessary controllers. The function used to publish throttle, brake, and steering is publish.

Note that throttle values passed to publish should be in the range 0 to 1, although a throttle of 1 means the vehicle throttle will be fully engaged. Brake values passed to publish should be in units of torque (N*m). The correct values for brake can be computed using the desired acceleration, weight of the vehicle, and wheel radius.

    twist_controller.py

This file contains a stub of the Controller class. You can use this class to implement vehicle control. For example, the control method can take twist data as input and return throttle, brake, and steering values. Within this class, you can import and use the provided pid.py and lowpass.py if needed for acceleration, and yaw_controller.py for steering. Note that it is not required for you to use these, and you are free to write and import other controllers.

    yaw_controller.py

A controller that can be used to convert target linear and angular velocity to steering commands.

    pid.py

A generic PID controller that can be used in twist_controller.py.

    lowpass.py

A generic low pass filter that can be used in lowpass.py.

### Traffic Light Classifier

The traffic light detection node (tl_detector.py) subscribes to four topics:

    /base_waypoints provides the complete list of waypoints for the course.
    /current_pose can be used used to determine the vehicle's location.
    /image_color which provides an image stream from the car's camera. These images are used to determine the color of upcoming traffic lights.
    /vehicle/traffic_lights provides the (x, y, z) coordinates of all traffic lights.

The node should publish the index of the waypoint for nearest upcoming red light's stop line to a single topic:

    /traffic_waypoint

For example, if waypoints is the complete list of waypoints, and an upcoming red light's stop line is nearest to waypoints[12], then 12 should be published /traffic_waypoint. This index can later be used by the waypoint updater node to set the target velocity for waypoints[12] to 0 and smoothly decrease the vehicle velocity in the waypoints leading up to waypoints[12].

The permanent (x, y) coordinates for each traffic light's stop line are provided by the config dictionary, which is imported from the traffic_light_config file:

config_string = rospy.get_param("/traffic_light_config")
self.config = yaml.load(config_string)

Your task for this portion of the project can be broken into two steps:

    Use the vehicle's location and the (x, y) coordinates for traffic lights to find the nearest visible traffic light ahead of the vehicle. This takes place in the process_traffic_lights method of tl_detector.py. You will want to use the get_closest_waypoint method to find the closest waypoints to the vehicle and lights. Using these waypoint indices, you can determine which light is ahead of the vehicle along the list of waypoints.
    Use the camera image data to classify the color of the traffic light. The core functionality of this step takes place in the get_light_state method of tl_detector.py. There are a number of approaches you could take for this task. One of the simpler approaches is to train a deep learning classifier to classify the entire image as containing either a red light, yellow light, green light, or no light. One resource that's available to you is the traffic light's position in 3D space via the vehicle/traffic_lights topic.

Note that the code to publish the results of process_traffic_lights is written for you already in the image_cb method.
Traffic Light Detection package files

Within the traffic light detection package, you will find the following:

    tl_detector.py

    This python file processes the incoming traffic light data and camera images. It uses the light classifier to get a color prediction, and publishes the location of any upcoming red lights.

    tl_classifier.py

    This file contains the TLClassifier class. You can use this class to implement traffic light classification. For example, the get_classification method can take a camera image as input and return an ID corresponding to the color state of the traffic light in the image. Note that it is not required for you to use this class. It only exists to help you break down the classification problem into more manageable chunks. Also note that Carla currently has TensorFlow 1.3.0 installed. If you are using TensorFlow, please be sure to test your code with this version before submission.
    traffic_light_config

    This config file contains information about the camera (such as focal length) and the 2D position of the traffic lights's stop line in world coordinates.

Helper Tool in the Simulator

In order to help you acquire an accurate ground truth data source for the traffic light classifier, the Udacity simulator publishes the current color state of all traffic lights in the simulator to the /vehicle/traffic_lights topic in addition to the light location. This state can be used to generate classified images or subbed into your solution to help you work on another single component of the node. The state component of the topic won't be available when running your solution in real life so don't rely on it in the final submission. However, you can still reference this topic in real life to get the 3D world position of the traffic light.
Using Different Classification Models for the Simulator and Site

We will test your team's code both in the simulator and on the testing site. Due to differences in the appearance of the site and simulator traffic lights, using the same traffic light classification model for both might not be appropriate. The self.config dictionary found in the TLDetector class of tl_detector.py contains an is_site boolean. You can use this boolean to load a different classification model depending on the context.

### Final Waypoint Updater

Once traffic light detection is working properly, you can incorporate the traffic light data into your waypoint updater node. To do this, you will need to add a subscriber for the /traffic_waypoint topic and implement the traffic_cb callback for this subscriber.

There are several helpful methods that you can use:

    get_waypoint_velocity(self, waypoint): gets the linear velocity (x-direction) for a single waypoint.
    set_waypoint_velocity(self, waypoints, waypoint, velocity): sets the linear velocity (x-direction) for a single waypoint in a list of waypoints. Here, waypoints is a list of waypoints, waypoint is a waypoint index in the list, and velocity is the desired velocity.
    distance(self, waypoints, wp1, wp2): Computes the distance between two waypoints in a list along the piecewise linear arc connecting all waypoints between the two. Here, waypoints is a list of waypoints, and wp1 and wp2 are the indices of two waypoints in the list. This method may be helpful in determining the velocities for a sequence of waypoints leading up to a red light (the velocities should gradually decrease to zero starting some distance from the light).

To accomplish this part of the project successfully, you will need to adjust the target velocities for the waypoints leading up to red traffic lights or other obstacles in order to bring the vehicle to a smooth and full stop. You should aim to have a smooth decrease in velocity leading up to the stopping point.

It will be up to you determine what the deceleration should be for the vehicle, and how this deceleration corresponds to waypoint target velocities. As in the Path Planning project, acceleration should not exceed 10 m/s^2 and jerk should not exceed 10 m/s^3.




This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
