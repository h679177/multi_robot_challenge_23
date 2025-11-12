### Run instructions:

Ros commands needed are
ros2 launch multi_robot_challenge_23 rescue_robots_w[chosen world].launch.py 
ros2 run scoring scoring

The launch file will start the wall following and marker detection algorithms. Scoring will need to be started separately. 

### Code description:
The code written by the group is located in wall_follower.py and marker_detection.py. The algorithm used for moving/navigation first creates a division line between the two robots. The robots will then move straight ahead until they encounter a wall, and will proceed to follow said wall using a wall-followere algorithm. The two robots move in different directions, one having the wall on their left side and the other on the right (depending on where their starting position is). They do this until they encounter the division line again, they then proceed to turn and find a new wall,  the idea behind this is that the robots will find walls on the inside of the map and follow these, and this way cover most of the map and finding at least most of the aruco-codes. Every time they get to the division line, the robots will do this behaviour, this is as previously stated to cover most of the map, but also to keep the two robots on their side of the map, avoiding any collisions to happen, and to avoid unnecessary double-coverage of the maps, where they both cover the same parts of the maps. 

### Work left to do:
If we were more people in our group and/or had more time we would have implemented the logic behind the big fires, where two robots are supposed to be within the perimiter of the aruco-code to gain points. We would also tweak the wall-follower; right now the robots sometimes get stuck in corners, which is not time-efficient nor the wanted behaviour for them. The robots also sometimes drive past aruco-markers, especially when they are located right after an outer corner, which is also not ideal behaviour for them. We would also have to work more on the code for the marker-detection, the way our code works now the detection algorithm has some weird behaviours; they detect and try to register markers where there are none (they see some that they arent supposed to see, and they also have the wrong coordinates. When using the camera to try to see what they see, we cannot see the markers they are supposedly detecting). Ideally they would also not detect them from too far away either, which is something we would also want to fix. 
