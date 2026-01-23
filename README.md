# Wheebot - Autonomous (Robotic) Wheelchair

<!-- PROJECT LOGO -->
<br />
<p align="center">
   <img src="images/wheebot.png" alt="Wheebot">
</p>

## Table of Contents

* [About](#about)
* [Getting Started](#getting-started)
  * [Prerequisites](#prerequisites)
* [Usage](#usage)
* [Contributing](#contributing)
* [License](#license)
* [Contact](#contact)
* [Acknowledgements](#acknowledgements)

<!-- ABOUT -->  
## About
This repository contains the material used in the course **Self Driving and ROS 2 - Learn by Doing! Plan & Navigation** that is currently available on the following platforms:
* [Udemy](https://www.udemy.com/course/self-driving-and-ros-2-learn-by-doing-plan-navigation/?referralCode=1F6AFF9E9A8E7D5D057A)

Have you ever developed a path-planning algorithm for your robot?
Do you want to know more about Motion Planning algorithms and how to use them to autonomously move your robot while avoiding obstacles using a Costmap of the environment?

Then this course will teach you exactly that, with many more topics:
* Path Planning
* Motion Planning
* Decision Making
* Behavior Trees
* Obstacle Avoidance
* Costmaps
* Nav2

Furthermore, all the laboratory classes in which we are going to develop the actual Software of our mobile robot are available both in **Python** and in **C++**
to let you the freedom of choosing the programming language you like the most or become proficient in both!

<!-- GETTING STARTED -->
## Getting Started
You can decide whether to build the real robot or just have fun with the simulated one. The course can be followed either way, most of the lessons and most of the code will work the same in the simulation as in the real robot

### Prerequisites
You don't need any prior knowledge of ROS 2 nor of Self-Driving, I'll explain all the concepts as they came out and as they are needed to implement new functionalities to our robot.
A basic knowledge of programming, either using **C++** or **Python** is required as this is not a Programming course and so I'll not dwell too much on basic Programming concepts.

To prepare your PC you need:
* Install Ubuntu 24.04 on PC or in Virtual Machine
Download the ISO [Ubuntu 24.04](https://ubuntu.com/download/) for your PC
* Install [ROS Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html) or [ROS Iron](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html) on your Ubuntu 24.04
* Install ROS 2 missing libraries. Some libraries that are used in this project are not in the standard ROS 2 package. Install them with:
```sh
sudo apt-get update && sudo apt-get install -y \
     ros-jazzy-ros2-controllers \
     ros-jazzy-gz* \
     ros-jazzy-ros-gz* \
     ros-jazzy-ros2-control* \
     ros-jazzy-joint-state-publisher-gui \
     ros-jazzy-joy \
     ros-jazzy-joy-teleop \
     ros-jazzy-nav2* \
     ros-jazzy-tf-transformations
```

<!-- USAGE -->
## Usage
To Launch the Simulation of the Robot 
1. Clone the repo
```sh
git clone https://github.com/AntoBrandi/Self-Driving-and-ROS-2-Learn-by-Doing-Plan-Navigation.git
```
2. Build the ROS 2 workspace
```sh
cd ~/Self-Driving-and-ROS-2-Learn-by-Doing-Plan-Navigation/tree/jazzy/Section9_Build_the_Robot/bumperbot_ws/src
```
```sh
colcon build
```
3. Source the ROS 2 Workspace
```sh
. install/setup.bash
```
4. Launch the Gazebo simulation
```sh
ros2 launch bumperbot_bringup simulated_robot.launch.py
```

<!-- LICENSE -->
## License

Distributed under the Apache 2.0 License. See `LICENSE` for more information.


<!-- CONTACT -->
## Contact

Antonio Brandi - [LinkedIn]([linkedin-url]) - antonio.brandi@outlook.it

My Projects: [https://github.com/AntoBrandi](https://github.com/AntoBrandi)


<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements
* [Course Cover Images](https://www.linkedin.com/in/delia-garc%C3%ADa-masegosa-109bb040/)
* [Turtlebot 3 Software](https://github.com/ROBOTIS-GIT/turtlebot3)
* [Turtlebot 3 Hardware](https://cad.onshape.com/documents/2586c4659ef3e7078e91168b/w/14abf4cb615429a14a2732cc/e/9ae9841864e78c02c4966c5e)


<!-- MARKDOWN LINKS & IMAGES -->
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=flat-square&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/in/antonio-brandi-512166bb/
[udemy-shield]: https://img.shields.io/badge/-Udemy-black.svg?style=flat-square&logo=udemy&colorB=555
[udemy-url]: https://www.udemy.com/course/self-driving-and-ros-2-learn-by-doing-map-localization/?referralCode=8FC4AC725C57F7A93F79
