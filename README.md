## Setup (w/ catkin tools)
```bash
mkdir -p catkin_ws/src
cd catkin_ws
catkin init
cd src
git clone <THIS REPO>
cd ..
catkin build
source devel/setup.bash
rosrun orb_test orb_test_node  # needs roscore to be running
```
