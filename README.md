<h1 align="center">ATLANTIS</h1>

<h1 align="center">About</h1>
Atlantis is a multi-layered simulation framework for bridging AI and robotics. Atlantis allows to incrementally incorporate complexity, starting from abstract planning and optimization, and progressing towards real-world deployment. Through a stack of simulators, Atlantis facilitates the seamless transition from AI-driven problem-solving to practical implementation, thus addressing the inherent challenges of top-down integration between AI and robotics. \
Atlantis contains three layers: 

1. Discrete Event Simulation (DES)
2. Non-Physics Simulation (NPS)
3. Physics Simulation (PS) 

Each layer is  associated with a different simulator and works with a different level of abstraction and representation. 

Atlantis can be used to test different applications domains including:

1. Material-Flow 
2. Manipulation
3. Navigation


There are two different ways to use Atlantis, source installation or docker image.

<h1 align="center">Docker Installation</h1>

First, install [Docker](https://www.docker.com/) following the instructions provided at this [link](https://docs.docker.com/engine/install/).

To build the docker image, run
```
cd <path/to/atlantis>
docker build --pull --rm -f 'src/atlantis/Dockerfile' -t 'atlantisws:latest' 'src/atlantis'
```
To allow GUI, run:
```
xhost +local:docker
```
To run the docker image:
```
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd):/home/milo/ws/src/atlantis \
  atlantisws:latest
```

Then you can skip to the Running section

<h1 align="center">Source Installation</h1>

Start by installing the dependencies.

# ROS2 and Rviz2
Install [ROS2](https://docs.ros.org/#ros-for-beginners) Jazzy following the instruction provided at this [link](https://docs.ros.org/en/jazzy/). 

Install [Rviz2](https://github.com/ros2/rviz) by running the command:
```
sudo apt install ros-<ros2-distro>-rviz2
```
# Navigation 2
Install [Nav2](https://docs.nav2.org/getting_started/index.html#installation) by running the command:

```
sudo apt install ros-<ros2-distro>-navigation2
sudo apt install ros-<ros2-distro>-nav2-bringup
```

# Navigo 
Install [Navigo](https://github.com/PaoloForte95/navigo)
```
git clone https://github.com/PaoloForte95/navigo
colcon build --packages-select navigo
```

# Common Interface
Clone the repository for common interface msgs, srvs, and actions.
```
git clone https://github.com/PaoloForte95/common_interfaces
colcon build --packages-select location_msgs material_handler_msgs object_detection_msgs standard_msgs  
```
# Collection

Get a collection of scenarios and machines from [atlantis_collection] (https://gitsvn-nt.oru.se/pofe/atlantis_collection)
```
git clone https://gitsvn-nt.oru.se/pofe/atlantis_collection.git
colcon build --packages-select atlantis_collection
```

<h1 align="center">Build</h1>

Clone this repository, using the command
```
git clone https://github.com/PaoloForte95/atlantis.git
```

To build, run the command:
```
colcon build --packages-up-to atlantis
```
To launch the Atlantis framework, run the command:
```
ros2 launch atlantis atlantis_launch.py simulator_level:=<level> scenario:=<path/to/your/scenario>
```
where the *level* can be: 
1. DES
2. NPS
3. PS

<h1 align="center">Running</h1>

One example is:
```
ros2 launch atlantis atlantis_launch.py scenario:=src/atlantis_collection/scenarios/mine/map1/mine1.yaml simulator_level:=NPS
```
To run a set of actions, run:
```
ros2 run atlantis_simple_commander simple_scheduler
```