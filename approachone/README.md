# Control Allocation First Approach

Linear split:

Pitch rate, yaw rate, airpseed commands come into three PIDs
Torque z and y and thrust commands come out.
These are then summed to create Motor spin speed commands
```math
\boldsymbol{\tau_i} = \mathbf{r_i} \times \mathbf{F_i}
```
```math
T = \sum_{i=1}^{n} F_i
```
```math
\mathbf{F_i} = C_{T} D^{4} \rho n^{2}
```
```math
\mathbf{F_i} = C n^{2}
```
