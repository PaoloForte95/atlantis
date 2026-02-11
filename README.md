<h1 align="center">Atlantis</h1>

# About
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

# Dependencies

## Navigation 2
Install [Nav2](https://docs.nav2.org/getting_started/index.html#installation) by running the command:

```
sudo apt install ros-<ros2-distro>-navigation2
sudo apt install ros-<ros2-distro>-nav2-bringup
```

## Navigo 
Install [Navigo](https://github.com/PaoloForte95/navigo)
```
git clone https://github.com/PaoloForte95/navigo
colcon build --packages-select navigo
```

## Athena (Planning)
Install Athena  by following the instructions provided at this [link](https://gitsvn-nt.oru.se/pofe/athena).


## Collection

Get a collection of scenarios and machines from [atlantis_collection] (https://gitsvn-nt.oru.se/pofe/atlantis_collection)
```
git clone https://gitsvn-nt.oru.se/pofe/atlantis_collection.git
colcon build --packages-select atlantis_collection
```

# Build

Clone this repository, using the command
```
git clone --recurse-submodules https://gitsvn-nt.oru.se/pofe/atlantis
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

One example is:
```
ros2 launch atlantis atlantis_launch.py scenario:=src/atlantis/atlantis_collection/scenarios/mine/mine1.yaml simulator_level:=NPS
```
To run a set of actions, run:
```
ros2 run atlantis_simple_commander simple_scheduler
```