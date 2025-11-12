### Run instructions:

Ros commands needed are
ros2 launch multi_robot_challenge_23 rescue_robots_w[chosen world].launch.py 
ros2 run scoring scoring

The launch file will start the wall following and marker detection algorithms. Scoring will need to be started separately. 

### Code description:
The code written by the group is located in wall_follower.py and marker_detection.py. The algorithm used for moving/navigation first creates a division line between the two robots. The robots will then move straight ahead until they encounter a wall, and will proceed to follow said wall 

### Work left to do:
