
First, mind launching noah gazebo simulation in small_house map

```sh
roslaunch noah_bot_launcher noah_navigationless.launch world_name:=small_house
```

Then, launch map navigation using move_base
```sh
roslaunch noah_map_navigation move_base_map_navigation.launch
```

Rviz can be launched by doing:
```sh
roslaunch noah_navigation rviz.launch
```
