# hector_mujoco
Compiling needs to use the following command
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
## keyboard callbacks:

* space - pause/unpause
* Q - to set robot on/off the ground
  * if robot on ground, pressing Q will set hang it mid air but simulaion continues running
  * if robot on air, pressing Q will set it on the ground and simulation is paused
