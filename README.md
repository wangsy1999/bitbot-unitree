# bitbot-gz

Usage:

```sh
mkdir -p bitbot_gz_ws/src && cd bitbot_gz_ws
git clone https://github.com/dknt0/bitbot-gz.git src/bitbot_gz
colcon build --symlink-install
source install/setup.zsh
ros2 launch bitbot_gz bitbot_gz.launch.py
ros2 run bitbot_gz main_app
```

TODO:
1. Write a gazebo plugin to reset robot states while keeping ros-controller working.

