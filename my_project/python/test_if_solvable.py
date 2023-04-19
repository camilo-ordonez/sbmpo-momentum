import matplotlib.pyplot as plt
import numpy as np
import math

tmax = 5

Fres = 1;
Fmax = 0.6*Fres;
mass = 1;
a = (Fmax-Fres)/mass

v0 = 1  # initial velocity in m/s
x0 = 0


# Time array
t = np.linspace(0, tmax, num=100)

v = v0 + a*t
x = x0  + v0*t + 0.5 * a * t**2



# Create subplots
fig, axs = plt.subplots(nrows=2, ncols=1, figsize=(8, 8))

# Position subplot
axs[0].plot(t, x, label='Position')
axs[0].set_xlabel('time (s)')
axs[0].set_ylabel('Position (m)')
axs[0].legend()

# Velocity subplot
axs[1].plot(t, v, label='Horizontal velocity')
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Velocity (m/s)')
axs[1].legend()

# Display the plot
plt.show()




