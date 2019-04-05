# SDC-ND-Path-Planning-Project-Highway-Driving-_Term3-Proj1
SDC-ND-Path Planning Project (Highway Driving)-Term3-Project1

![](https://github.com/emilkaram/SDC-ND-Path-Planning-Project-Highway-Driving-_Term3-Proj1/blob/master/images/image1.JPG)


# Introduction:
Path planning in self-driving cars
This is the general view of self-driving autonomous system integration :
The blocks inside the container are the parts of the path planning procedure;
Prediction:
We predict situations in over environment in order to get you to the destination safely and efficiently. For this project I had to build collision detection, that predicts a possible collision with two cars.

![](https://github.com/emilkaram/SDC-ND-Path-Planning-Project-Highway-Driving-_Term3-Proj1/blob/master/images/image2.png)

Behavior planner takes input :
•	map of the world,
•	route to the destination
•	prediction about what static and dynamic obstacles are likely to do
Output: Suggested maneuver for the vehicle which the trajectory planner is responsible for reaching collision-free, smooth and safe behavior.
![](https://github.com/emilkaram/SDC-ND-Path-Planning-Project-Highway-Driving-_Term3-Proj1/blob/master/images/image3.png)

![](https://github.com/emilkaram/SDC-ND-Path-Planning-Project-Highway-Driving-_Term3-Proj1/blob/master/images/image5.png)

Trajectory generation :
•	Current position (s, d)
•	Desired lane (s+30, r*lane+(r/2))
•	Desired lane (s+60, r*lane+(r/2))
•	Desired lane (s+90, r*lane+(r/2))

# The Goal of this Project
In this project, the goal is to design a path planner that is able to create smooth, safe trajectories for the car to follow along a 3-lane highway with traffic. A successful path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.
Also, need to control the speed and lane of a car given the sensor fusion data of cars around me via the simulator
The highway track has other vehicles, all going different speeds, but approximately obeying the 50 MPH speed limit.
The car transmits its location, along with its sensor fusion data, which estimates the location of all the vehicles on the same side of the road.

# Approach:
1.	First, get the car to drive forward at constant velocity about 50mph.
2.	Next, move straight at a constant velocity of 50mph and stay in current lane.
3.	Next, use spline.h library to interpolate the path poins to follow the lane and  avoid jerks.
4.	Next, sensing the car in front of me and slowing down if necessary distance is <=30m.
5.	At this point to avoid jerk in the start up, I set the starting velocity to 5mph and then gradually increase it over time at about 5m/s^2.
6.	For lane change, attempted blind chang a lane if there was a car ahead at <=30m.
7.	To avoid collisions, check cars on the right and left lanes then attempt a lane change if is safe.
Point Paths
The path planner output a list of x and y global map coordinates.
Each pair of x and y coordinates is a point, and all of the points together form a trajectory. I used 50 points, and maintained the x list the same length as the y list.
Every 20 ms the car moves to the next point on the list. 
The car's new rotation becomes the line between the previous waypoint and the car's new location.
The car moves from point to point perfectly by the simulator, so I don't have to worry about building a controller for this project.



# Velocity
The velocity of the car depends on the spacing of the points. 
Because the car moves to a new waypoint every 20ms, the larger the spacing between points, the faster the car will travel. 
The speed goal is to have the car traveling at (but not above) the 50 MPH speed limit as often as possible. However, there will be times when traffic gets in the way and car need to slow down to avoid collision.

Acceleration calculated by comparing the rate of change of average speed over 0.2 second intervals. So I set my acceleration and de-acceleration to 5m/s^2 to avoid jerk 
The jerk is calculated as the average acceleration over 1 second intervals. In order for the passenger to have an enjoyable ride both the jerk and the total acceleration should not exceed 10 m/s^2. So I set my acceleration and de-acceleration to 5m/s^2 to avoid jerk.
Using Previous Path Points
I coded to builds a 50 point path, it starts the new path with whatever previous path points were left over from the last cycle. Then we append new waypoints, until the new path has 50 total waypoints.
Using information from the previous path ensures that there is a smooth transition from cycle to cycle. But the more waypoints we use from the previous path, the less the new path will reflect dynamic changes in the environment.
Ideally, we might only use a few waypoints from the previous path and then generate the rest of the new path based on new data from the car's sensor fusion information.
Previous path data given to the Planner
 ["previous_path_x"] The previous list of x points previously given to the simulator
["previous_path_y"] The previous list of y points previously given to the simulator
Previous path's end s and d values
["end_path_s"] The previous list's last point's frenet s value
["end_path_d"] The previous list's last point's frenet d value
Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)
["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.
Timing
The simulator runs a cycle every 20 ms (50 frames per second), but my C++ path planning program will provide a new path at least one 20 ms cycle behind.
The simulator will simply keep progressing down its last given path while it waits for a new generated path.
This means that using previous path data becomes even more important when higher latency is involved. 
Imagine, for instance, that there is a 500ms delay in sending a new path to the simulator. As long as the new path incorporates a sufficient length of the previous path, the transition will still be smooth.
A concern, though, is how accurately we can predict other traffic 1-2 seconds into the future. An advantage of newly generated paths is that they take into account the most up-to-date state of other traffic.
Setting Point Paths with Latency
As a mentioned, my C++ path planner will at the very least be one cycle behind the simulator because the C++ program can't receive and send data on the same cycle. As a result, any path that the simulator receives will be from the perspective of a previous cycle. This might mean that by the time a new path reaches the simulator, the vehicle has already passed the first few waypoints on that path.
Luckily I don't have to worry about this too much. The simulator has built-in tools to deal with this timing difference. The simulator actually expects the received path to be a little out of date compared to where the car is, and the simulator will consider which point on the received path is closest to the car and adjust appropriately.
Highway Map
Inside data/highway_map.csv there is a list of waypoints that go all the way around the track. The track contains a total of 181 waypoints, with the last waypoint mapping back around to the first. The waypoints are in the middle of the double-yellow dividing line in the center of the highway.
The track is 6945.554 meters around (about 4.32 miles). If the car averages near 50 MPH, then it should take a little more than 5 minutes for it to go all the way around the highway.
The highway has 6 lanes total - 3 heading in each direction. Each lane is 4 m wide and the car should only ever be in one of the 3 lanes on the right-hand side. The car should always be inside a lane unless doing a lane change.



# Waypoint Data
Each waypoint has an (x,y) global map position, and a Frenet s value and Frenet d unit normal vector (split up into the x component, and the y component).
The s value is the distance along the direction of the road. The first waypoint has an s value of 0 because it is the starting point.
The d vector has a magnitude of 1 and points perpendicular to the road in the direction of the right-hand side of the road. The d vector can be used to calculate lane positions. For example, if you want to be in the left lane at some waypoint just add the waypoint's (x,y) coordinates with the d vector multiplied by 2. Since the lane is 4 m wide, the middle of the left lane (the lane closest to the double-yellow dividing line) is 2 m from the waypoint.
If you would like to be in the middle lane, add the waypoint's coordinates to the d vector multiplied by 6 = (2+4), since the center of the middle lane is 4 m from the center of the left lane, which is itself 2 m from the double-yellow dividing line and the waypoints.
Converting Frenet Coordinates
We have included a helper function, getXY, which takes in Frenet (s,d) coordinates and transforms them to (x,y) coordinates.
Interpolating Points
If you need to estimate the location of points between the known waypoints, you will need to "interpolate" the position of those points.
In previous lessons we looked at fitting polynomials to waypoints. Once you have a polynomial function, you can use it to interpolate the location of a new point.
There are also other methods you could use. For example, Bezier curve fitting with control points, or spline fitting, which guarantees that the generated function passes through every point.
Here is a great and easy to setup and use spline tool for C++, contained in just a single header file.

The map of the highway is in data/highway_map.txt
Each waypoint in the list contains [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.
The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.


# Sensor Fusion
It's important that the car doesn't crash into any of the other vehicles on the road, all of which are moving at different speeds around the speed limit and can change lanes.
The sensor_fusion variable contains all the information about the cars on the right-hand side of the road.
The data format for each car is: [ id, x, y, vx, vy, s, d]. The id is a unique identifier for that car. The x, y values are in global map coordinates, and the vx, vy values are the velocity components, also in reference to the global map. Finally s and d are the Frenet coordinates for that car.
The vx, vy values can be useful for predicting where the cars will be in the future. For instance, if you were to assume that the tracked car kept moving along the road, then its future predicted Frenet s value will be its current s value plus its (transformed) total velocity (m/s) multiplied by the time elapsed into the future (s).
The map of the highway is in data/highway_map.txt
Each waypoint in the list contains [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.
The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.
Main car's localization Data (No Noise)
["x"] The car's x position in map coordinates
["y"] The car's y position in map coordinates
["s"] The car's s position in frenet coordinates
["d"] The car's d position in frenet coordinates
["yaw"] The car's yaw angle in the map
["speed"] The car's speed in MPH



# Changing Lanes

The last consideration is how to create paths that can smoothly changes lanes. Any time the ego vehicle approaches a car in front of it that is moving slower than the speed limit, the ego vehicle should consider changing lanes.
The car should only change lanes if such a change would be safe, and also if the lane change would help it move through the flow of traffic better.
For safety, a lane change path should optimize the distance away from other traffic. For comfort, a lane change path should also result in low acceleration and jerk. The acceleration and jerk part can be solved from linear equations for s and d functions. Examples of this can be found in the Trajectory Generation quizzes entitled, "Quintic Polynomial Solver" and "Polynomial Trajectory".
The provided Eigen-3.3 library can solve such linear equations. The getXY helper function can transform (s,d) points to (x,y) points for the returned path.

# Model Documentation:
Generation of the three key future points of the car, this is used by the spline library as reference
vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
To fill in the gaps using spline library, we do the following:
// Calculate how to break up spline points so that we travel at our desired
// reference velocity and avoid jerk violation.
double target_x = 30.0;
double target_y = s(target_x);
double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
Finally to fill in the paths points using spline, we do the following:
// Fill up the rest of our path planner after filling it with previous points, here we
// will always output 50 points.
for (int i = 1; i <= 50 - previous_path_x.size(); i++) {

    double N = (target_dist/(0.02*ref_vel/2.24)); // 0.5 mph
    double x_point = x_add_on + (target_x) / N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // rotate back to normal after rotating earlier
    x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;
}
Finally the logic to change lanes if handled as follows:
if (too_close) {
    ref_vel -= .224; // 0.5 mph

    if (lane == 0 && !right_too_close) {
        lane = 1;
    } else if (lane == 1) {
        if (!left_too_close) {
            lane = 0;
        } else if (!right_too_close) {
            lane = 2;
        }
    } else if (lane == 2 && !left_too_close) {
        lane = 1;
    }

} else if (ref_vel < 49.5) {
    ref_vel += .224;
}
============






# Basic Build Instructions
1.	Clone this repo.
2.	Make a build directory: mkdir build && cd build
3.	Compile: cmake .. && make
4.	Run it: ./path_planning.
Click on the "Simulator" button in the bottom of the Udacity workspace, which will open a new virtual desktop. You should see a "Simulator" icon on the virtual desktop. Double-click the "Simulator" icon in that desktop to start the simulator.
Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

# Conclusion:
•	The car was able to drive more than 4.32 miles without incident.
•	The screen capture below the current/best miles driven without incident. 
•	The car drives according to the speed limit about 50mph.
•	The car drove within the speed limit. Also the car did not driving much slower than speed limit unless obstructed by traffic.
•	Max Acceleration and Jerk were not Exceeded.
•	The car did not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.
•	Car did not have collisions.
•	The car did not come into contact with any of the other cars on the road.
•	The car stayed in its lane, except for the time between changing lanes.
•	The car did not spend more than a 3-second length out side the lane lanes during changing lanes.
•	The car was able to change lanes smoothly.
•	The car was able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.

# Video link:


