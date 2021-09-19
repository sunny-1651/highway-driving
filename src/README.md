# Speed Limit
In the for loop starting at line 578, the reference velocity would reduce if a car is too close to the front of the ego vehicle (see line 
492, "too close" means within 30m ahead). Otherwise, the ego vehicle accelerates until the reference velocity exceeds 49.5mph. This would 
ensure that the maximum speed of 50 mph would never be exceeded. 

# Maximum Acceleration and Jerk
Looking back at the for loop starting at line 578, reference velocity decreases 0.224mph per 0.02s when another car is too close, which 
roughly equals to 5m/s^2. When the car accelerates, the acceleration is even less, which is 0.1mph per 0.02s (2.23m/s^2). So acceleration 
and jerk in the s direction would never exceed the limits. However, to minimize jerk in the d direciton, some more work needs to be done. 
I use the chooseNextLane function to select the next lane to go to if another car is slowing down the ego vehicle (line 496). Under 
special circumstance (will be discussed in more details later), this funciton can make the car change two lanes consecutively, max 
acceleration and jerck can sometimes be exceeded. Lines 540-542 set the next three waypoints to follow. I discovered that by setting 
next_wp0's s position 40m ahead of the ego vehicle, i can make lane shift less abrupt and reduce acceleration and jerk in the d direction. 

# Car Does not Have Collisions
Collision with cars in front is already taken care of by the loop starting at line 578. Collision with cars behind was not considered, but 
since the other vehicles are usually slower than 45mph, this is not a big concern. Sideway collisions are handled by the collisionCost 
function. The chooseNextLane function returns the lane with the lowest cost among the current lane and the adjacent lanes. The lowest cost 
lane would be the next lane the car goes to. The cost function has two components: 1) inefficiency cost, which increases as vehicle speed 
goes slower; 2) collision cost, which increases as the distance between the ego vehicle and cars in the intended lane decreases. I designed 
the inefficiency cost and collision cost in such a way that if there's a car in the intended lane within +- 10m of the ego vehicle, the 
collision cost for switching lane will almost always exceed the inefficiency cost of keep lane, hence reducing the risk of sideway 
collision. I also implemented an additional safety feature to avoid sideway collision in the chooseNextLane funciton. I set 0.8 (line 201) 
as the cost threshold, if cost of all options exceed 0.8, then the car should keep lane no matter what. This would be useful when a car is 
very slow in front and close to the ego vehicle, and the cost for keep lane exceeds the cost for switching lane.

# Car Stays in Lane
When the car is not performing a lane shift, the "lane" value in lines 540-542 would always be the index for the current lane, so the next 
few waypoints will be in the current lane. Lines 540-542 takes in the s and d values of the waypoints and convert them to xy coordinates.
Then the waypoints values are feed into the spline tool to generate a spline. Lines 571-574 and 584-601 generate the next set of 50 points
for the car to follow. Those code are explained in the project Q&A video (starting from 33:04), thus I'm not going to repeat that here.

# Car Can Change Lanes
Alread explained in the third section above. See my chooseNextLane function and cost functions for details.

# To Stand Out: Optimized Lane Changing
See comments and code in line 241-255 for how I change two lanes consecutively. This nested if statement would bypass cost function 
comparison and shift the car from a side lane to the center lane. Then when the chooseNextLane function is called again, the car would
attempt to switch to the fastest lane. 
