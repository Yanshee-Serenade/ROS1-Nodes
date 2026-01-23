# Serenade ROS1 nodes

- Serenade-ROS

## Build

1. Build node with `docker compose build`

## Run

### ORB-SLAM3

> [!WARNING]
> Building ORB-SLAM3 is disabled now.
> To build ORB-SLAM3, uncomment related part in Dockerfile.

```
roslaunch orb_slam3_ros pi.launch
```