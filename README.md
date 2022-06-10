# h6x-Internship

- gazebo 9+ (Tested on 11)
- ros2-galactic

## 実行方法


```bash
mkdir -p ~/ros2_ws/src
source /opt/ros/galactic/setup.bash
cd ~/ros2_ws/src
git clone https://github.com/HarvestX/h6x-Internship.git
cd ~/ros2_ws
colcon build --symlink-install

ros2 launch h6x_internship_gazebo world.launch.py
```

