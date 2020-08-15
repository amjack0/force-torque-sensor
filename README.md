# force-torque-sensor
This repository is created to simulate the force torque sensor on universal robot.

# &#x1F539; Getting started 
This repo will generate the force torque data on the wrist_3_link of the UR5.


# &#x1F539; Running this repo

1) source the required environment.
2) Make sure to clone UR5 robot from here ``` https://github.com/amjack0/universal-robot-with-force-torque-sensor ``` , As there are changes requied in robot description file to have ``` force torque sensor ```
3) ``` roslaunch ur_gazebo ur5.launch ```
4) Make sure to change the ``` gazebo_transport_topic_to_sub ``` in the node as per your topic. after adding the sensor in robot.urdf.xacro file, launch the gazebo model and check the sensor topic name by ```gz topic -l ```.
4) ``` rosrun ft_sensor ft_sensor_gazebo_transport_to_ros_topic ``` will start publishing the wrench. 
