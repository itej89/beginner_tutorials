# beginner_tutorials

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
---
# Overview:
 * This repository consists of a basic ROS2 publisher subscriber.
 * Two seperate packages were created. 
    - "cpp_pubsub_msgs" package containts the custome message file. 
    -  "cpp_pubsub" package contains the actual the publisher and subscriber nodes.
 * The publisher publishes a constant message periodically and prints to the console
 * The subscriber node recieves the string and prints to the console
 * Launch file is also created which launches all the nodes at once and modifies one parameter in the client node.
 * Google Style Guide was followed in all the cpp files.
 * CPPCHECK and CPPLINT was run to perform static code analysis.
 * "results" folder contains the output of CPPCHECK and CPPLINT in a text file.
 * doxygen documentation is added to the "docs" folder inside cpp_pubsub.


## Developer:
 - Tej Kiran 
    - UID: 119197066

## Dependencies/Requirements: 
 - Laptop
 - Ubuntu 22.04 or higher
 - VS Code/Terminal
 - ROS 2 Humble

## How to build
``` bash
# Source ros environemnt
  source /opt/ros/humble/setup.bash
# Cloning the repository
  git clone --recurse-submodules <repo-link>
# cd to repository
  cd beginner_tutorials
# Compile and build the project:
  colcon build
#
```


## Instructions to run static code analysis:
 ```bash
 # Navigate to src folder in package
 cd <ros2_workspace>/src/beginner_tutorials/src

 # run the following command
 cppcheck --enable=all --std=c++17 ./cpp_pubsub/src/*.cpp --suppress=missingIncludeSystem --suppress=missingInclude --suppress=unmatchedSuppression > ./results/cppcheckreport

 # The report can be viewed at ./beginner_tutorials/results/cppcheckreport

 ```

## Instructions to check Google Style:
 
```bash
# Navigate to source folder
  cd beginner_tutorials/cpp_pubsub/src

#  run the following command
 cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order ./cpp_pubsub/src/*.cpp   > ./results/cpplintreport > ./results/cpplintreport

 # The report can be viewed at ./beginner_tutorials/results/cpplintreport
 ```

## Instructions to generate doxygen documentation:

```bash
# Navigate to doc folder in package
 cd beginner_tutorials/cpp_pubsub/doc
 
# Run the following command
 doxygen

 # To see the documentation
 cd beginner_tutorials/cpp_pubsub/doc/html
 firefox index.html
 ```


## How to run the demo
### Running individual nodes
    Individual nodes can be invoked by follwoing the below steps.
- ### Running publisher node
```bash
# Source ros environemnt
  source /opt/ros/humble/setup.bash
# Source project
  source /opt/ros/humble/setup.bash
# run talker node
  ros2 run cpp_pubsub onetalker
```
- ### Running subscriber node
```bash
# Source ros environemnt
  source /opt/ros/humble/setup.bash
# Source project
  source /opt/ros/humble/setup.bash
# run talker node
  ros2 run cpp_pubsub onelistener
```
- ### Results
![alt text](./result_images/node_results.png)

### Running launch file
      The publisher and subscriber nodes can also be launched from the launch file using the below steps.
- ### Instructions
```bash
# Source ros environemnt
  source /opt/ros/humble/setup.bash
# Source project
  source /opt/ros/humble/setup.bash
# run talker node
  ros2 launch cpp_pubsub pubsub_launch.py
  ```
- ### Results
![alt text](./result_images/launch_results.png)

## Dependency Installation: 
- ROS 2 Humble:
  - Follow the below website instructions to install ROS 2 Humble based on your Ubuntu version
    - Ubuntu 22.04:
      - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#install-ros-2-packages