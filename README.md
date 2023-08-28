# Introduction:
A 3D human-robot interaction Gazebo simulator for our paper ["DRL-VO: Learning to Navigate Through Crowded Dynamic Scenes Using Velocity Obstacles"](
https://doi.org/10.1109/TRO.2023.3257549
)([arXiv](https://arxiv.org/pdf/2301.06512.pdf)) in IEEE Transactions on Robotics (T-RO) 2023, and ["Towards Safe Navigation Through Crowded Dynamic Environments"](
https://doi.org/10.1109/IROS51168.2021.9636102
) in IROS 2021, which is modified from https://github.com/srl-freiburg/pedsim_ros.
The aim of the simulator is to provide a close-to-real-world simulated environment for mobile robots navigating around pedestrians. 
Detailed usage can be found at our [DRL-VO control policy](https://github.com/TempleRAIL/drl_vo_nav.git).

Here is a GIF showing how this simulator works: 

![simulation_demo](pedsim_simulator/images/1.simulation_demo.gif "simulation_demo") 

## Modifications:
This simulator is based on the Gazebo simulator and PEDSIM library, which uses the social forces model to guide the motion of individual pedestrians. The main modifications from https://github.com/srl-freiburg/pedsim_ros are as follows:
* Integrate Pedsim_ros with the 3D gazebo simulator and turtlebot2 robot;
* Add an additional social force to the robot and other pedestrians;
* Add different test scenarios for robot navigation;
* Change the meshes of spawned agents in the gazebo to make them look like real humans; 

<img src=pedsim_simulator/images/2.gazebo_macro.png width=500/> <img src=pedsim_simulator/images/3.gazebo_micro.png width=500/>

-------------------------------------------------------------------------------------------------------------------------------

## Requirements
* Ubuntu system >= 16.04
* ROS-Kinetic/Melodic/Noetic

## Installation
The default version is ROS Noetic.
```
cd ~
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
git clone https://github.com/TempleRAIL/robot_gazebo.git
git clone https://github.com/TempleRAIL/pedsim_ros_with_gazebo.git
wget https://raw.githubusercontent.com/zzuxzt/turtlebot2_noetic_packages/master/turtlebot2_noetic_install.sh 
sudo sh turtlebot2_noetic_install.sh 
cd ..
catkin_make
```

## Usage:
```
roslaunch pedsim_simulator robot.launch
```

## Acknowledgements
All contributors from https://github.com/srl-freiburg/pedsim_ros.

## Citation
```
@article{xie2023drl,
  author={Xie, Zhanteng and Dames, Philip},
  journal={IEEE Transactions on Robotics}, 
  title={DRL-VO: Learning to Navigate Through Crowded Dynamic Scenes Using Velocity Obstacles}, 
  year={2023},
  volume={39},
  number={4},
  pages={2700-2719},
  doi={10.1109/TRO.2023.3257549}
}

@inproceedings{xie2021towards,
  title={Towards safe navigation through crowded dynamic environments},
  author={Xie, Zhanteng and Xin, Pujie and Dames, Philip},
  booktitle={2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={4934--4940},
  year={2021},
  organization={IEEE}
}
```


