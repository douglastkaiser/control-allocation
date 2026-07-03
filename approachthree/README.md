# Control Allocation Second Approach

Quadratic Programming Polytopes:
Same as first two approaches:
Pitch rate, yaw rate, airpseed commands come into three PIDs
Torque z and y and thrust commands come out.
Each motor is represented in a matrix that maps those commands to motor speeds

New to this approach:
MPT toolbox distributes torques in a way that accounts for motor saturation
