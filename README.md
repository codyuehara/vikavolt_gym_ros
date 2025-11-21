# vikavolt_gym_ros
ROS2 Wrapper for vikavolt gym env

``` 
docker build -t vikavolt_gym_ros .
```

```
docker compose up
```

```
docker exec -it vikavolt_gym_ros-sim-1 /bin/bash
```

```
source .bashrc
ros2 launch vikavolt_gym_ros gym_bridge_launch.py
```
