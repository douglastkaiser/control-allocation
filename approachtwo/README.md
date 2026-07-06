<!-- This file is auto-generated -->
# Control Allocation Second Approach

PseudoInverse mixing:

Same as first approach:
Pitch rate, yaw rate, airpseed commands come into three PIDs
Torque z and y and thrust commands come out.


New to this approach:

Each motor is represented in a matrix that maps those commands to motor speeds
Pseudoinverse distributes it in a least squares manner
```math
\tau = I \dot{w}
```

```math
SE = \frac{\sigma}{\sqrt{n}}
```
