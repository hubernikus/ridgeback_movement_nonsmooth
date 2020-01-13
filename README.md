# Ridgebacke Movement in the LASA laboratory

##
Do the following steps:

In your catkin src directory clone the repository
```
git clone https://github.com/epfl-lasa/kuka-lwr-ros.git
```


## Setup
Set rosmaster to ridgeback in 

SSH onto the ridgeback
```
administrator -X @172.16.0.135
```
PW: ridgeback

Synchronize time:
```
sudo ntpdate 172.16.0.XXX
```
with the local ip of the computer (not the ridgeback), e.g IP=172.16.0.189

Check that the delay is (almost) zero:
```
ntpdate -q 172.16.0.XXX
```
Run vision system
```
roslaunch ridgeback_movement vrpn-lab_south.launch 
```

## Run Obstacle Avoidance 

```
rosrun ridgeback_movement obstacle_avoidance_optitrack.py
```








