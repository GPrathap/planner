import numpy as np 
import ssa
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
import math

# data = np.loadtxt("/home/geesara/catkin_ws/src/planner/Hagen-Planner/hagen_planner/state_machine/data/result.txt", delimiter=",")
# print(np.mean(data, axis=0))


reference = np.load("/home/geesara/catkin_ws/src/planner/Hagen-Planner/hagen_planner/state_machine/data/reference.npy")
actual = np.load("/home/geesara/catkin_ws/src/planner/Hagen-Planner/hagen_planner/state_machine/data/actual.npy")[800:6500]

result = []
length_ = 4
result = np.array(reference)[::2]

plt.rcParams["font.size"] = "30"
fig, axs = plt.subplots(4, sharex=True)


diff_result = result[0:actual.shape[0]]
diff = np.linalg.norm(actual-diff_result, axis=1)
# print(diff.shape)

x_axis_val = np.arange(900, actual.shape[0], 1)
axs[0].plot(result[:,0], "blue", linewidth=2, label=r'$p^{ref}_x$')
axs[0].plot(actual[:,0], "red", linewidth=2, label=r'$p_x$')
axs[0].set_xlim([0, result.shape[0]])
custom_ticks = np.array([-3, 0, 6])
axs[0].set_yticks(custom_ticks)
axs[0].set_yticklabels(custom_ticks)
legend = axs[0].legend(loc='upper right', fontsize=23)

axs[1].plot(result[:,1], "blue", linewidth=2, label=r'$p^{ref}_y$')
axs[1].plot(actual[:,1], "red", linewidth=2, label=r'$p_y$')
axs[1].set_xlim([0, result.shape[0]])
custom_ticks = np.array([0, 6, 12])
axs[1].set_yticks(custom_ticks)
axs[1].set_yticklabels(custom_ticks)
legend = axs[1].legend(loc='upper right', fontsize=23)
ax = axs[0]
ymin, ymax = ax.get_ylim()
ax.vlines(x=[800, 5700], ymin=ymin, ymax=ymax, linewidth=2, color='g')

axs[2].plot(result[:,2], "blue", linewidth=2, label=r'$p^{ref}_z$')
axs[2].plot(actual[:,2], "red", linewidth=2, label=r'$p_z$')
axs[2].set_xlim([0, result.shape[0]])
custom_ticks = np.array([0, 2, 4])
axs[2].set_yticks(custom_ticks)
axs[2].set_yticklabels(custom_ticks)
ax = axs[1]
ymin, ymax = ax.get_ylim()
ax.vlines(x=[800, 5700], ymin=ymin, ymax=ymax, linewidth=2, color='g')
legend = axs[2].legend(loc='upper right', fontsize=23)

f = lambda x: x/15.0
g = lambda x: x
ax2 = axs[0].secondary_xaxis("top", functions=(f,g))
fig.text(0.06, 0.5, 'm', ha='center', va='center', rotation='vertical')
ax = axs[2]
ymin, ymax = ax.get_ylim()
ax.vlines(x=[800, 5700], ymin=ymin, ymax=ymax, linewidth=2, color='g')

# axs[3].plot(diff, "blue", linewidth=2, label=r'$distance error$')
xk = np.linspace(0, actual.shape[0], actual.shape[0])
mean_error = np.mean(diff)
std_diff = np.std(diff)
mean_error = [mean_error]*actual.shape[0]
axs[3].errorbar(xk, diff, std_diff, color='black',
             ecolor='lightgray', label=r'$|p-p^{ref}|_{L2}\pm std$')
axs[3].plot(mean_error, "green", linewidth=3, label=r'$\overline{|p-p^{ref}|}_{L2}$')
axs[3].set_xlim([0, result.shape[0]])
custom_ticks = np.array([0, 0.36, 1.0])
axs[3].set_yticks(custom_ticks)
axs[3].set_yticklabels(custom_ticks)
legend = axs[3].legend(loc='upper right', fontsize=23)
ax = axs[3]
ymin, ymax = ax.get_ylim()
ax.vlines(x=[800, 5700], ymin=ymin, ymax=ymax, linewidth=2, color='g')
axs[3].set_xlabel("number of readings (15 readings per second)")
plt.show()