# Kalman Filter

```bash
mkdir kalman_ws/src -p
cd kalman_ws/src
git clone https://github.com/ivbelkin/kalman_filter.git
cd ..
source /opt/ros/melodic/setup.bash
catkin_make
source devel/setup.bash
roslaunch kalman_filter main.launch \
    odom_topic:=/husky_velocity_controller/odom \
    odom_filtered_topic:=/husky_velocity_controller/odom_filtered \
    play:=true \
    bag:=/home/ivb/Downloads/17.bag
```