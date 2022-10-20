# Purpose of This Repo

This repository contains the starter code to launch in the SDC Planning course workspace.

# Project Notes

## TODO-Lookahead: 
One way to find a reasonable lookahead distance is to find the distance you will need to come to a stop while traveling at speed V and  using a comfortable deceleration.  

From section 26 of the motion planning lecture: distance_to_Vf = (Vf^2 - Vi^2) / 2a. In case of coming to a stop Vf = 0 m/s. The maximum (comfortable) deceleration is set at -4.0 m/s^2 based information in [this](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7559987/) article. Therefore, the look_ahead_distance = computed as follows:
```
double a_decel_max = 4.0; // m/s^2 
auto look_ahead_distance = -1 * pow(velocity_mag, 2.0) / (2 * a_decel_max);
```

## TODO-goal behind the stopping point: 
Put the goal behind the stopping point (i.e the actual goal location) by "_stop_line_buffer". HINTS: remember that we need to go back in the opposite direction of the goal/road, i.e you should use: ang = goal.rotation.yaw + M_PI and then use cosine and sine to get x and y.

From section 13 of the motion planning lecture: offsets can be calculated using simple trigonometry. Unlike the example we are not looking for a point perpendicular to a 'center goal', but behind a stopping point. Hence the ang variable should be computed using goal.rotation.yaw + M_PI rather than goal.rotation.yaw + M_PI/2. 
```
auto ang = goal.rotation.yaw + M_PI;
goal.location.x += _stop_line_buffer * cos(ang);
goal.location.y += _stop_line_buffer * sin(ang);
```

