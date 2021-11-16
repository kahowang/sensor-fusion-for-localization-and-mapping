# import necessary module
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np

fused = np.loadtxt("fused_without_cons.txt")
fused_vel_data = np.loadtxt("fused_vel.txt")
fused_cons_data = np.loadtxt("fused_cons.txt")

# x = fused_vel_data[:,0]
y_fused = fused[:,1]
z_fused = fused[:,2]

y_fused_vel = fused_vel_data[:,1]
z_fused_vel = fused_vel_data[:,2]

y_fused_cons_data = fused_cons_data[:,1]
z_fused_cons_data = fused_cons_data[:,2]

plt.plot(y_fused, c='r', label ="y_fused")
# plt.plot(y_fused_vel, c='g', label ="Y_fused_vel")
plt.plot(y_fused_cons_data, c='b', label ="y_fused_cons")
plt.legend();
plt.title('y velocity ',fontsize=18,color='y')
plt.show()

plt.plot(z_fused, c='r', label ="z_fused")
plt.plot(z_fused_vel, c='g', label ="z_fused_vel")
plt.plot(y_fused_cons_data, c='b', label ="y_fused_cons")
plt.legend();
plt.title('z velocity ',fontsize=18,color='y')
plt.show()



