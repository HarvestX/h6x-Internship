<img src="https://harvestx.jp/img/logo-red.svg" width="30%">

# h6x-Internship
HarvestX Internship contens.

[Start from here](./docs).

---

## Requirements
- Linux OS
  - [Ubuntu 20.04](https://releases.ubuntu.com/20.04/)
- ROS 2
  - [Galactic Geochelone](https://index.ros.org/doc/ros2/Installation/Galactic/)
- Gazebo 11

---

## Install
### Locate package in workspace
```bash
mkdir -p ~/ws_galactic/src
cd ~/ws_galactic/src
git clone git@github.com:HarvestX/h6x-Internship.git
```

### Run script to install dependencies
```bash
source /opt/ros/galactic/setup.bash
cd ~/galactic_ws/src/h6x-Internship
./setup.bash
exec -l $SHELL
```

### Build Source
Open new terminal and type followings.
```bash
source /opt/ros/galactic/setup.bash
cd ~/galactic_ws
colcon build
```

---

## Usage
- Display each system
```bash
ros2 launch h6x_internship_gazebo world.launch.py
```

---

