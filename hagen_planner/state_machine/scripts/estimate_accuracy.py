import numpy as np 
import ssa
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
import math

reference = np.load("/home/geesara/catkin_ws/src/planner/Hagen-Planner/hagen_planner/state_machine/data/reference_velocity.npy")
actual = np.load("/home/geesara/catkin_ws/src/planner/Hagen-Planner/hagen_planner/state_machine/data/actual_velocity.npy")
actual = np.transpose(actual)
reference = np.transpose(reference)
actual = actual[800:6500]
reference = reference[800:6500]

plt.rcParams["font.size"] = "30"
fig, axs = plt.subplots(5, sharex=True)
velocity_=[-0.8, 0.8]

axs[0].plot(actual[:,0], "red", linewidth=2, label=r'$v_x$')
axs[0].plot(reference[:,0], "blue", linewidth=2, label=r'$v^{ref}_x$')
axs[0].set_xlim([0, actual.shape[0]])
axs[0].set_ylim(velocity_)
custom_ticks = np.array([-0.4, 0, 0.4])
axs[0].set_yticks(custom_ticks)
axs[0].set_yticklabels(custom_ticks)
legend = axs[0].legend(loc='upper right', fontsize=23)

axs[1].plot(actual[:,1], "red", linewidth=2, label=r'$v_y$')
axs[1].plot(reference[:,1], "blue", linewidth=2, label=r'$v^{ref}_y$')
axs[1].set_xlim([0, actual.shape[0]])
axs[1].set_ylim(velocity_)
custom_ticks = np.array([-0.4, 0, 0.4])
axs[1].set_yticks(custom_ticks)
axs[1].set_yticklabels(custom_ticks)
legend = axs[1].legend(loc='upper right', fontsize=23)

axs[2].plot(actual[:,2], "red", linewidth=2, label=r'$v_z$')
axs[2].plot(reference[:,2], "blue", linewidth=2, label=r'$v^{ref}_z$')
axs[2].set_xlim([0, actual.shape[0]])
axs[2].set_ylim(velocity_)
custom_ticks = np.array([-0.4, 0, 0.4])
axs[2].set_yticks(custom_ticks)
axs[2].set_yticklabels(custom_ticks)
legend = axs[2].legend(loc='upper right', fontsize=23)

diff = np.linalg.norm(actual[:,0:3]-reference[:,0:3], axis=1)
print(diff.shape)
print(actual.shape)
mean_error = np.mean(diff)
std_diff = np.std(diff)
mean_error = [mean_error]*actual.shape[0]
xk = np.linspace(0, actual.shape[0], actual.shape[0])
axs[3].errorbar(xk, diff, std_diff, color='black',
             ecolor='lightgray', label=r'$|v-v^{ref}|_{L2} \pm std$')
axs[3].plot(mean_error, "green", linewidth=3, label=r'$\overline{|v-v^{ref}|}_{L2}$')
axs[3].set_xlim([0, actual.shape[0]])
axs[3].set_ylim(velocity_)
custom_ticks = np.array([0.1])
axs[3].set_yticks(custom_ticks)
axs[3].set_yticklabels(custom_ticks)
legend = axs[3].legend(loc='upper right', fontsize=23)

axs[4].plot(actual[:,3], "red", linewidth=2,  label=r'$Yaw$')
axs[4].plot(reference[:,3], "blue", linewidth=2,  label=r'$Yaw^{ref}$')
axs[4].set_xlim([0, actual.shape[0]])
axs[4].set_ylim([-3.14, 3.14])
axs[4].set_xlabel("number of readings (15 readings per second)")
fig.text(0.06, 0.6, 'm/s', ha='center', va='center', rotation='vertical')
axs[4].set_ylabel("rad")
f = lambda x: x/15.0
g = lambda x: x
ax2 = axs[0].secondary_xaxis("top", functions=(f,g))
legend = axs[4].legend(loc='upper right', fontsize=23)

plt.show()