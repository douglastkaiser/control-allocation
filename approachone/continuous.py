import control as ctrl
import matplotlib.pyplot as plt
import numpy as np

# Define Plant Transfer Function: P(s) = 1 / (s^2 + 2s + 1)
plant_num = [1]
plant_den = [1, 2, 1]
P = ctrl.tf(plant_num, plant_den)

# Define PID Controller: C(s) = Kp + Ki/s + Kd*s
# Let's use Kp=10, Ki=5, Kd=2
# C(s) = (Kd*s^2 + Kp*s + Ki) / s
Kp, Ki, Kd = 10, 5, 2
pid_num = [Kd, Kp, Ki]
pid_den = [1, 0]
C = ctrl.tf(pid_num, pid_den)

# Calculate Open Loop and Closed Loop Transfer Functions
open_loop = C * P
closed_loop = ctrl.feedback(open_loop, 1)


# Analyze Stability Margins
gm, pm, wg, wp = ctrl.margin(open_loop)
print(f"Gain Margin: {gm:.2f}")
print(f"Phase Margin: {pm:.2f} degrees")

# Generate Bode Plot
plt.figure()
ctrl.bode_plot(open_loop, dB=True, Phase=True)
plt.suptitle("Open-Loop Bode Plot")
plt.show()


# Sensitivity and Complementary Sensitivity
S = ctrl.feedback(1, open_loop)
T = ctrl.feedback(open_loop, 1)

# Plotting
omega = np.logspace(-2, 2, 100)
plt.figure()
ctrl.bode_plot(T, omega, dB=True)
plt.suptitle("Closed-Loop Transfer Function (T)")
plt.show()
