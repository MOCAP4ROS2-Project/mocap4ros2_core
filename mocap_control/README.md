# mocap_control

`mocap_control` implements a protocol that controls the start and stop of a mocap system, although it can be used to control the start of other systems that have to synchronize with the mocap systems.

The scenario to be controlled is shown in the following figure. Mocap systems, or systems that synchronize with them, are on the same network. Some of them are implemented in ROS2, and some others in ROS, connected with a ROS <-> ROS2 bridge. This protocol applies to systems in both standards. There may even be systems that require an analog trigger, and this protocol would apply to a digital <-> Analog bridge. All machines on the network are supposed to be synchronized by NTP or another more precise protocol.

In the network there is a control node that triggers the start or stop of each of the systems.

**All controlled systems are lifecycle nodes**. All nodes are assumed to be running, and in an inactive state. What this protocol does is take them to the active state when starting, and to the inactive state when stopping.

![mocap_contro_arch](https://user-images.githubusercontent.com/3810011/118769923-fa52ff00-b880-11eb-8eb2-443dc5121a2b.png)

This protocol uses two types of messages: 
* `mocap_controls_msgs/msg/Control`: Uses the topic `/mocap_control` (QoS reliable), and controls the startup and shutdown of systems

```
int8 START=0
int8 ACK_START=1
int8 STOP=2
int8 ACK_STOP=3

int8 control_type
builtin_interfaces/Time stamp

string system_source
string capture_session_id
string[] capture_systems
```



* `mocap_controls_msgs/msg/MocapInfo`: Uses the topic `/mocap_environment` (QoS reliable + transient_local) and each system publishes information about it, including which topics it uses to publish the information it produces. It is necessary to publish this info if you want to use the [RQT Mocap Control](https://github.com/MOCAP4ROS2-Project/mocap4ros2_applications/tree/main/rqt_mocap_control).

```
int8 ROS2=0
int8 ROS1=1

string system_source
int8 ros_version_source
string[] topics

```

The protocol is quite simple, and is shown in the figure below

![mocap4ros_protocol](https://user-images.githubusercontent.com/3810011/118773447-21abcb00-b885-11eb-95f4-38f3f72da5ef.png)

1. When the control node triggers the system startup, it sends a message whose `control_type = START`, and in` capture_systems` includes the identifiers (the node name) of the systems it wants to boot. If it is empty, it is understood that you want to start all available ones.
2. Each system responds with a message whose `control_type = ACK_START`. What is relevant about this message is the timestamp, which allows the control node to find out if there are significant deviations in the system clock.
3. The stop follows the same scheme, but with STOP and ACK_STOP type messages.

If you follow this protocol, you will be able to use the MOCAP4ROS2 control application, implemented as an RQT plugin.

![Captura de pantalla 2021-05-19 09:49:30](https://user-images.githubusercontent.com/3810011/118775945-bfa09500-b887-11eb-8394-1a2c3ea82719.png)

**You always be able to start/stop your system using the `ros2 lifecycle` commands**

## Uso de `ControlledLifecycleNode` (ROS and ROS2)

To use this protocol you have two alternatives: The first is to implement the protocol yourself. The second is to use a library that is implemented within `mocap_control`, which completely simplifies the management of this protocol. An example of this second alternative is in the [mocap4ros2_system_example](https://github.com/MOCAP4ROS2-Project/mocap4ros2_system_example) package . Note that there is a [ROS version](https://github.com/MOCAP4ROS2-Project/mocap4ros2_core/tree/noetic/mocap_control) that implements in ROS what would be a lifecycle node.

The magic is done by the `ControlledLifecycleNode` class, which in turn inherits from` rclcpp_lifecycle :: LifecycleNode`. If you make your node inherit from this class, it implements the protocol.

In addition to using the normal callbacks (`on_activate`,` on_deactivate`, ...) you can redefine the following methods, in case you want to be notified of these transitions:

```
virtual void control_start(const mocap_control_msgs::msg::Control::SharedPtr msg);
virtual void control_stop(const mocap_control_msgs::msg::Control::SharedPtr msg);
```

## ROS1 Support

Although MOCAP4ROS2 is a project primarily for ROS2, it is possible to have mocap systems or other systems that synchronize with it on ROS1. There is a version of [mocap_control in ROS1](https://github.com/MOCAP4ROS2-Project/mocap4ros2_core/tree/noetic/mocap_control/include/mocap_control) (branch: noetic) that implements something to simulate lifecycles nodes, and attending to the mocap_control protocol.

The key to using these components in ROS1 is having a bridge, but the standard version of ros1_bridge is not enough. You must use [this version](https://github.com/fmrico/ros1_bridge/tree/mocap4ros2).

Including this package in your workspace implies that the way to compile it will be a little different. These are the steps:

1. As prerequisite, you need to have a ROS1 workspace apart, built with the `mocap_control_msgs` package for` noetic`, at least. 
2. Open a terminal in which **you haven't** sourced ROS or ROS2. We will do it manually in the first build. 
3. Clone the custom `ros1_bridge` package in `${MOCAP4ROS2_ROS2_WS}/src`:
```
git clone -b mocap4ros2 https://github.com/fmrico/ros1_bridge.git
```
4. Source the underlay ROS2:
```
source /opt/ros/foxy/setup.bash
```
5. Compile all the workspace (in `${MOCAP4ROS2_ROS2_WS}`), except the bridge (from now on, yo will compile your workspace always with this option):
```
colcon build --symlink-install --packages-skip ros1_bridge
```
6. Source the overlays:
```
source ${MOCAP4ROS2_ROS1_WS}/devel/setup.bash
source ${MOCAP4ROS2_ROS2_WS}/install/setup.bash
```
7. Compile the bridge
```
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
```
8. Now you can add the `source ${MOCAP4ROS2_ROS2_WS}/install/setup.bash` to your `${HOME}/.bashrc`
9. You can use the Rqt Mocap Control to start the bridges, or start them manually:
```
# Terminal 1
ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics
# Terminal 2
ros2 run ros1_bridge mocap_info_1_to_2
```

---
### MOCAP4ROS2
The project [MOCAP4ROS2](https://rosin-project.eu/ftp/mocap4ros2) is funded as a Focused Technical Project by [ROSIN](http://rosin-project.eu/).


<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png"
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg"
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287.
