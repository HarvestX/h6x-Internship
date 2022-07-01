# h6x-Internship

- gazebo 9+ (Tested on 11)
- ros2-galactic

<br>

## ビルド


```bash
mkdir -p ~/ws_galactic/src
source /opt/ros/galactic/setup.bash
cd ~/ws_galactic/src
git clone https://github.com/HarvestX/h6x-Internship.git
cd ../
bash src/h6x-Internship/setup.bash
colcon build --symlink-install
```

<br>

## 実行方法

```bash
source ~/ws_galactic/install/setup.bash
ros2 launch h6x_internship_gazebo world.launch.py
```

