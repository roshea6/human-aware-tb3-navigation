# Human Aware Navigation for Turtlebot 3
Final project for my Cooperating Autonomous mobile robots course (CPE-631 S 20). Implementing a human aware navigation framework that allows a Turtlebot 3 to traverse through a human filled environment with human comfort in mind.

## How to use 
1. Clone this repository into your catkin_ws/src

2. Clone the following repositories into your catkin_ws/src
```
git clone https://github.com/ral-stevens/CPE631Final.git
git clone -b melodic-devel https://github.com/marinaKollmitz/human_aware_navigation.git
git clone -b melodic-devel https://github.com/marinaKollmitz/people.git
git clone -b melodic-devel https://github.com/marinaKollmitz/lattice_planner.git
git clone -b melodic-devel https://github.com/marinaKollmitz/human_aware_navigation.git
git clone -b melodic-devel https://github.com/marinaKollmitz/timed_path_follower.git
git clone https://github.com/DLu/wu_ros_tools.git
```

3. Make sure you have all the required dependencies installed
```
rosdep install --from-paths src --ignore-src --rosdistro melodic -y
```

4. Build your workspace
```
catkin_make
```
5. Launch the simulation and navigation framework
```
roslaunch human-aware-tb3-navigation human_aware_tb3.launch
```
6. Run the costmap clearing node
```
rosrun human-aware-tb3-navigation clear_costmap.py 

```
7. Use the 2D Nav Goal button in RVIZ to create a goal for the robot

<p align='center'>
    <img src="/media/demo.gif" alt="navigation_example" width="800"/>
</p>

