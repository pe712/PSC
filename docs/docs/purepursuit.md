Theoretical operation of the pure pursuit algorithm
# Pure Pursuit
1. Determine the current position of the vehicle on the track
2. Find the closest point on the route to the vehicle
3. Find the goal point
4. Make the transformation to determine it in the vehicle frame of reference
5. Find the necessary curvature for the vehicle to reach this point
6. Send the request to the vehicle to modify its trajectory
7. Update the coordinates of the vehicle

Formula for pure tracking:
Gamma = 2x/l²

You can see the implementation [here](https://github.com/pe712/PSC/blob/main/node/pure_pursuit.py)
