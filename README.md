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

## Load parameters from yaml file
To load the parameters for e.g. the newer college dataset:
```bash
rosparam load config/newer_college.yaml
```

This will store the params in the parameter server, so they will be available
when running the node with `rosrun`.

The yaml file with params can also be specified in a launch file and
will then be loaded into the param server on launch.

## Rosservice usage
To toggle the `force_degeneracy` service, use
```bash
rosservice call /force_degeneracy
```

Enabled means LOAM is forced degenerate. Disabled means it is nominal.