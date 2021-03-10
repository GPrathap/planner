import numpy as np 
import ssa
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
import math

data = np.loadtxt("/home/geesara/catkin_ws/src/planner/Hagen-Planner/hagen_planner/state_machine/data/result_234_co.txt", delimiter=",")

# reach = np.where(data[:,2]<1.5) or  np.where(data[:,2]>0)
reach = np.where(data[:,2]>0)
total_cout = data.shape[0]
pass_count = reach[0].shape[0]
reach_data = data[reach]
sf = (pass_count)/total_cout
mnpl = np.mean(reach_data, axis=0) 
print(sf,"&" ,mnpl[2],"&", mnpl[0])
# print(np.mean(data, axis=0))


