# slic3r_coverage_planner
A coverage planner for ROS2 using libslic3r as core logic

# Parameter
## Clockwise
By default, outer perimeter gets followed counter clockwise. 
With the optional parameter clockwise, (true/false) you can change the direction.

```
    <node pkg="slic3r_coverage_planner" type="slic3r_coverage_planner" name="slic3r_coverage_planner" output="screen">
      <param name="clockwise" type="bool" value="true" />
    </node>
```

## equally_spaced
By default, coverage planner calculates a path which consists of a pose every 10 cm, even on straight lines. For example a path from point (0,0) to (1,0) will result in a path with 10 poses.
This can cause issues when using other planners than ftc_local_planner. By setting the optional parameter equally_spaced (true/false), you can change this behavior. If set to false, the path will only contain poses where needed (i.e. direction changes). 

## visualize_plan
Publish plan as marker array. Useful for debugging purposes only.

# Usage
## Run the service
```
ros2 run slic3r_coverage_planner coverage_planner
```
## Test service
After starting the service, you can run ``slic3r_coverage_client.py`` to call the service. This script publishes the generated path segments (array). It first waits for some subscribers for topic ``slic3r_path`` and will start publishing each Path[] array with a delay of 5 seconds in between.
You can use RViz2 to display the paths.