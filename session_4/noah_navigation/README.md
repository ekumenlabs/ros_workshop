
First, mind launching noah gazebo simulation.

```sh
roslaunch noah_bot_launcher noah_navigationless.launch
```

Then, launch navigation using move_base (No map)
```sh
roslaunch noah_navigation move_base.launch
```

Rviz can be launched by doing:
```sh
roslaunch noah_navigation rviz.launch
```
