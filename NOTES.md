# Purpose of This Repo

This repository contains the starter code to launch in the SDC Planning course workspace.

# Project Notes

## TODO-Lookahead: 
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
``