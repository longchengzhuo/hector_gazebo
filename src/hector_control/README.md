ROS2 Package for hector v2 control package

## Dependencies:
* [Boost](http://www.boost.org) (version 1.5.4 or higher)
* [CMake](http://www.cmake.org) (version 2.8.3 or higher)
* [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page) (>3.3)

## Run Control For Simulated Environment
```
ros2 run hector_control hector_ctrl --ros-args -p "simulated_robot":=true
```

## Run Control For Physical Robot
```
ros2 run hector_control hector_ctrl --ros-args -p "physical_robot":=true
```
