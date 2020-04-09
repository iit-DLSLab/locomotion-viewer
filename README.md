<img src="https://github.com/orsoromeo/jet-leg/blob/master/figs/feasible_region.png" alt="hyqgreen" width="400"/>  <img src="https://github.com/orsoromeo/jet-leg/blob/master/figs/four_stance.png" alt="planning" width="400"/>
<img src="https://github.com/orsoromeo/jet-leg/blob/master/figs/force_polygons.png" alt="hyqgreen" width="400"/>  <img src="https://github.com/orsoromeo/jet-leg/blob/master/figs/foothold_planning.png" alt="planning" width="400"/>


# Locomotion-viewer: a tool for visualizing locomotion-related geometric objects
This library specializes on top of the ros melodic [rviz-visual-tools](https://github.com/PickNikRobotics/rviz_visual_tools) package to construct geometrical objects related to robotics legged locomotion such as support regions, trajectories and 3D feasibility constraints. All interfaces are provided through Eigen.

## What you can visualize in Rviz with locomotion-viewer:
- dashed lines
- arbitrary 2D polygons of different colors 
- 3D objects such as tetrahedrons and complex hexahedrons

## dependencies:
```
Eigen
ros-melodic-rviz-visual-tools
```

The above ROS dependencies can be installed with the following commands:
```
sudo apt install ros-melodic-rviz-visual-tools
```
