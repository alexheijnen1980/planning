# Purpose of This Repo

This repository contains the starter code to launch in the SDC Planning course workspace.

# Project Notes: behavior_planner_FSM.cpp

## TODO-lookahead: 
One way to find a reasonable lookahead distance is to find the distance you will need to come to a stop while traveling at speed V and  using a comfortable deceleration.  

### Solution
From section 26 of the motion planning lecture: distance_to_Vf = (Vf^2 - Vi^2) / 2a. In case of coming to a stop Vf = 0 m/s. The maximum (comfortable) deceleration is set at -4.0 m/s^2 based information in [this](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7559987/) article. Therefore, the look_ahead_distance = computed as follows:
```
double a_decel_max = 4.0; // m/s^2 
auto look_ahead_distance = -1 * pow(velocity_mag, 2.0) / (2 * a_decel_max);
```

## TODO-goal behind the stopping point: 
Put the goal behind the stopping point (i.e the actual goal location) by "_stop_line_buffer". HINTS: remember that we need to go back in the opposite direction of the goal/road, i.e you should use: ang = goal.rotation.yaw + M_PI and then use cosine and sine to get x and y.

### Solution  
From section 13 of the motion planning lecture: offsets can be calculated using simple trigonometry. Unlike the example we are not looking for a point perpendicular to a 'center goal', but behind a stopping point. Hence the ang variable should be computed using goal.rotation.yaw + M_PI rather than goal.rotation.yaw + M_PI/2. 
```
auto ang = goal.rotation.yaw + M_PI;
goal.location.x += _stop_line_buffer * cos(ang);
goal.location.y += _stop_line_buffer * sin(ang);
```

## TODO-goal speed at stopping point:
What should be the goal speed??

### Solution  
The speed at the stopping point should be zero.
```
goal.velocity.x = 0.0;
goal.velocity.y = 0.0;
goal.velocity.z = 0.0;
```

## TODO-goal speed in nominal state: 
What should be the goal speed now that we know we are in nominal state and we can continue freely? Remember that the speed is a vector. HINT: _speed_limit * std::sin/cos (goal.rotation.yaw).

### Solution
From section 13 of the motion planning lecture: velocity components can be calculated using simple trigonometry. 
```
goal.velocity.x = _speed_limit * cos(goal.rotation.yaw);
goal.velocity.y = _speed_limit * sin(goal.rotation.yaw);
goal.velocity.z = 0;
```

### TODO-maintain the same goal when in DECEL_TO_STOP state: 
Make sure the new goal is the same as the previous goal (_goal). That way we keep/maintain the goal at the stop line.

### Solution
```
goal = _goal;
```

## TODO-calculate the distance, use distance rather than speed, move to STOPPED state
It turns out that when we teleport, the car is always at speed zero. In this the case, as soon as we enter the DECEL_TO_STOP state, the condition that we are <= _stop_threshold_speed is ALWAYS true and we move straight to "STOPPED" state. To solve this issue (since we don't have a motion controller yet), you should use "distance" instead of speed. Make sure the distance to the stopping point is <= P_STOP_THRESHOLD_DISTANCE. Uncomment the line used to calculate the distance.

### Solution
The line to compute the distance was already uncommented.
```
auto distance_to_stop_sign = utils::magnitude(goal.location - ego_state.location);
// LOG(INFO) << "Ego distance to stop line: " << distance_to_stop_sign;

if (utils::magnitude(ego_state.velocity) <= _stop_threshold_speed) {
  if (distance_to_stop_sign <= P_STOP_THRESHOLD_DISTANCE) {
    _active_maneuver = STOPPED;
    _start_stop_time = std::chrono::high_resolution_clock::now();
    // LOG(INFO) << "BP - changing to STOPPED";
    }
}
```

## TODO-maintain the same goal when in STOPPED state: 
Make sure the new goal is the same as the previous goal. That way we keep/maintain the goal at the stop line.

### Solution
```
goal = _goal;
```
## TODO-move to FOLLOW_LANE state
What state do we want to move to, when we are "done" at the STOPPED state?
      
### Solution
```      
_active_maneuver = FOLLOW_LANE;
```

# Project Notes: cost_functions.cpp

## TODO-circle placement:
Where should the circles be at? The code below is NOT complete. HINT: use CIRCLE_OFFSETS[c], sine and cosine to calculate x and y: cur_y + CIRCLE_OFFSETS[c] * std::sin/cos(cur_yaw).

### Solution
From section 31 of the motion planning lecture: velocity circle center locations can be calculated using simple trigonometry. 

```
auto circle_center_x = cur_x + CIRCLE_OFFSETS[c] * cos(cur_yaw); 
auto circle_center_y = cur_y + CIRCLE_OFFSETS[c] * sin(cur_yaw); 
```

## TODO-distance from circles to obstacles/actor:
How do you calculate the distance between the center of each circle and the obstacle/actor?

### Solution
From section 31 of the motion planning lecture: a collision can be identified by checking if the distance between the circle centers is ever closer than the sum of the radius of the circles (representing the ego vehicle and the actors). 

```
double dist = std::sqrt(
              std::pow((circle_center_x - actor_center_x), 2) +
              std::pow((circle_center_y - actor_center_y), 2));
```

## TODO-distance between last point on spiral and main goal: 
How do we calculate the distance between the last point on the spiral (spiral[n-1]) and the main goal (main_goal.location). 
Use spiral[n - 1].x, spiral[n - 1].y and spiral[n - 1].z.
Use main_goal.location.x, main_goal.location.y and main_goal.location.z
Ex: main_goal.location.x - spiral[n - 1].x

### Solution
```
auto delta_x = main_goal.location.x - spiral[n - 1].x;
auto delta_y = main_goal.location.y - spiral[n - 1].y;
auto delta_z = main_goal.location.z - spiral[n - 1].z;
```

# Project Notes: motion_planner.cpp

## TODO-Perpendicular direction: 
ADD pi/2 to the goal yaw.
  
### Solution
```  
auto yaw = goal_state.rotation.yaw + M_PI / 2; 
```

## TODO-offset goal location: 
Calculate the x and y position of the offset goals using "offset" (calculated above) and  knowing that the goals should lie on a perpendicular line to the direction (yaw) of the main goal. You calculated this direction above (yaw_plus_90). 
HINT: use std::cos(yaw_plus_90) and std::sin(yaw_plus_90) 

### Solution
From section 13 of the motion planning lecture: offsets can be calculated using simple trigonometry.

```
goal_offset.location.x += offset * cos(yaw); 
goal_offset.location.y += offset * sin(yaw); 
```

# Project Notes: velocity_profile_generator.cpp

## TODO-calc distance:
Use one of the common rectilinear accelerated equations of motion to calculate the distance traveled while going from v_i (initial velocity) to v_f (final velocity) at a constant acceleration/deceleration "a". 
HINT look at the description of this function. Make sure you handle div by 0.
    
### Solution
From section 26 of the motion planning lecture: distance_to_Vf = (Vf^2 - Vi^2) / 2a.
```
d = std::abs((v_f * v_f - v_i * v_i) / (2 * a));
```

## TODO-calc final speed: 
Calculate the final distance. HINT: look at the description of this function. Make sure you  handle negative discriminant and make v_f = 0 in that case. If the discriminant is inf or nan return infinity.

### Solution
From section 25 of the motion planning lecture: Vf = sqrt(2a * sf + Vi*2)
```
double disc = v_i * v_i + 2 * a * d;
```
# Project Notes: planning_params.h

## TODO-num of paths (goals):

### Solution
Section 15 of the motion planning lecture introduces 7 goals.
```
#define P_NUM_PATHS 7
```