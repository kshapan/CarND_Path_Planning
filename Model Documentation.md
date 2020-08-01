# **Path Planning**

## Writeup

---

**Path Planning Project**

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.


## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1971/view) individually and describe how I addressed each point in my implementation.

---
## Compilation

The code compiles. No changes to the CMakeLists.txt were made.

---
## Valid Trajectories

All the criteria are fullfilled most of the time and trajectory is smooth and jerk less.

---
## Reflection

### Processing of the sensor fusion data

The vehicles received from sensor fusion are checked for traffic using their lateral Frenet coordinate. The vehicles outside the roadway where the ego vehicle is moving are ignored.

### Path planning logic

The output of this section are the desired lane and desired velocity. These are later used as a reference for trajectory generation. These parameters need to be tracked and updated, so they are defined as a global variables that are pased to the function by reference.

#### Choosing the correct lane

The decision whether some lane is driveable or not is based on the if the front vehicle is slower and distance between ego vehicle and front vehicle is less than 30 m and if other lane is available for lane change. If any vehicle finds itself in the specific area around the ego longitudinal Frenet position, the lane in which this vehicle is is considered undriveable. If any vehicle is about to reach the longitudinal Frenet position of the ego vehicle within some given range, the lane in which this vehicle is is considered undriveable. When ego lane becomes undriveable, the algorithm looks for the lane to change to. If more than one option is available, the car chooses the lane with the largest relative vehicle distance. If ego lane becomes undriveable, and no lane is available for lane change, the velocity reference is adjusted so it matches the velocity of the first vehicle in front. The velocity is changed step-wise, so it doesn't violate the max acceleration criteria. 

### Trajectory Generator

The whole logic used for trajectory generation is copied from the David's and Aaron's Q&A session. The only difference to their implementation is getting the reference lane.

### Problem

Sometimes, its observed that when vehicle changes lane, it sometimes come close of the lane boundary, which could be dangerous. This can be solved by refining the map way-points, or by doing the spline fitting. of the points themselves.
