## Path-planning Project Writeup ##


----
### Project Overview ###

>In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

the video of the resulte of my project can be found [here.]()

### Rublic Points ###

The following points were achieved as the [video.]()
* The car is able to drive at least 4.32 miles without incident
* The car drives according to the speed limit.
* Max Acceleration and Jerk are not Exceeded.
* Car does not have collisions.
* The car stays in its lane, except for the time between changing lanes.
* The car is able to change lanes


#### There is a reflection on how to generate paths ####

I created Othercar class for handle the cars detect by sensor fusion (in Othercar.cpp).

After getting telemetry data from simulater (#221-239 in main.cpp), I created the detected other_cars instances by detect_other_cars() (#247 in main.cpp). Then I recognized the other cars and those lanes, distances from own vehicle, and those speeds. (#21-45 in Othercar.cpp).

Next, I prepare to create next trajectory based on the instruction video which udacity provided (#252-284 in main.cpp). Then, I added selecting the next lane based on the costs calculated from several aspects (#48-89 in othercar.cpp).

Finally, I used spline library to create the final trajectory. (#310-349 in main.cpp). As the project [video](), it succeeded to meet project's criteria, but it may be better to introduce vehicle state concept (keeping lane, prepare to lane change, lange change) and make dicision for lane change more conservatively or precisely. 

