# Multi-domain coordinated motion for autonomous surface and underwater vehicles

*** WORK IN PROGRESS ***

The aim of this project is to model the integration of aiding signals from an USV into the navigation system of an AUV. This will enable the AUV to know its global position based on the GPS position of the USV and range-only measurements from the surface. One way and two ways communications will be explored.
A second objective aims at developing behaviour rules for the vehicles that will allow them to work around an inspection target, taking into account their physical and navigational constraints while allowing the AUV to benefit from the external position information. The constraints to respect will be to keep the tether between the USV and the AUV manageable during the motions.
The objectives of this project are to develop multiple strategies to evaluate in terms of robustness and stability of the vehicles’ motion under differing levels of GNSS and communications noise and different environmental conditions.

## Related Packages
Required: Ubuntu 16.04 with ROS Kinetic (http://wiki.ros.org/kinetic). Also, the following packages are required: 

* UUV-simulator:

  https://github.com/uuvsimulator/uuv_simulator

* Desistek SAGA ROV vehicle:

  https://github.com/uuvsimulator/desistek_saga.git

* Heron USV:

  https://clearpathrobotics.com/blog/2019/01/heron-usv-gets-a-new-simulator/

## Environment Setup
Create the Workspace:
```
$ source /opt/ros/kinetic/setup.bash
$ mkdir -p ~/thesis_ws/src
$ cd ~/thesis_ws/
$ catkin_make
```
Install all related packages and the current work:
```
$ cd ~/thesis_ws/src
$ git clone https://github.com/uuvsimulator/uuv_simulator
$ git clone https://github.com/uuvsimulator/desistek_saga.git
$ git clone https://github.com/gregoire-hw/go_thesis.git
$ cd ~/catkin_ws
& catkin_make
```

## Set the different sensors:
### Desistek Saga:
The DVL needs to be added to the desistek saga.
```
$ roscd desistek_saga_description/urdf/
$ sudo gedit desistek_saga_sensors.xacro
```
Add the following lines:
```xml
<!-- DVL -->
<xacro:default_dvl_macro
  namespace="${namespace}"
  parent_link="${namespace}/base_link"
  inertial_reference_frame="${inertial_reference_frame}">
  <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:default_dvl_macro>
```
### Heron:

## Work done:
- [x] Launch a UUV simulator world with an ASV and an AUV
- [x] Range-Only measurements
- [X] AUV localisation using DVL and IMU
- [ ] ASV localisation using GPS
- [ ] Localisation using EKF

## List of program files:
File | Description
-----|------------
desistek_dvl.py | Converts data from DVL and IMU into the position of the desistek.
heron_controller.py | Control heron's thrusters.
heron_thrustersManager.py | Get the thrusters values from heron_controller.py and send them to the ASV.
lbl.py | Simulates the LBL range-only measurements between the two robots.
