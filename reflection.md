## Reflections

This is an interesting project, challenging in different way. I took down the projects and rebuild it a few times 
before being able to make it to a stage that can pass the requirements. 

A lot of time and effort was spent on figuring out reasonable way to control vehicle to drive along track. Leaving less time to learn real path planning contents.

Instead having direct control input such as throttle, steering angle and etc, we have to feed proposed points into simulator teleports the car to the location sent into in a indeterministic rate which makes controlling a vehicle tricky and confusing from time to time. Maybe the reasoning behind it is to 
avoid directly considering vehicle dynamics and control.

Per introduction of the nano program, I am eager to learn how in real world vehicle dynamics 
is used to control the vehicle or having a simulator with realistic physics engine and vehicle model so that 
we can learn how vehicle dynamics is involved in motion control. However the actual content on these topics is close to none.  

To complete this project, three major issues need to be addressed:

Trajectory generation
Based on the desired immediate behavior, the trajectory planning component will determine which trajectory is best for executing this behavior.

Prediction
The prediction component estimates what actions other objects might take in the future. For example, if another vehicle were identified, the prediction component would estimate its future trajectory.

Behavioral planning
The behavioral planning component determines what behavior the vehicle should exhibit. For example stopping at a traffic light or intersection, changing lanes.

Model Documentation:

Trajectory generation

During initial attempts, trajectory was generated under Frenet (s, d) coordinates.

Waypoints in Cartesian to Frenet conversion is done using spline function such as 
`x = f(s), y = f(s), dx = f(s), dy = f(s)` 

I am able to drive the car around track in single lane with help of a simple P controller. Under limit time I failed to find a solution 
that makes the trajectories generated meet jerk and acceleration requirements consistently when changing lanes.

In this submission, spline interpolated trajectories are planned and generated in ego Cartesian coordinates. 30 meter spaced navigation points in Frenet spaces are used for proposing trajectories. It really helps to avoid excessive local non-linearity when performaing coordinate transformation.   
Planning and generating trajectory under local/ego coordinates also help deal with waypoints wrapping around issue. Affine transformation between local and map coordinates keep linearity of car poses under two representations which is less prone to numerical issues.

As suggested in project introduction `target_x` is a emperical estimated based on proposed distance change between two time steps (approximately 0.02s). 

Prediction and planner are coupled, it is a work in progress, originally, several cost function was used to drive path decision, unless, it is fined tuned with multiple reasonable assumptions and other vehicle estimation, for the time being, under simpler cases like in high way driving, a simple condition rule based state machine can provide similar perforamnce.
