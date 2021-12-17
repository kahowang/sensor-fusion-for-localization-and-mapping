import matplotlib.pyplot as plt
import numpy as np

sc_matrix_cur_scan = np.loadtxt("sc_matrix_cur_scan.txt")
sc_matrix_lcoal_map = np.loadtxt("sc_matrix_local_map.txt")

def samplemat(mat):
    """Make a matrix with all zeros and increasing elements on the diagonal"""
    sector = mat.shape[0]   # x
    ring = mat.shape[1]     # y
    aa = np.zeros((sector,ring))
    for i in range(sector):
        for j in range(ring):
            if(mat[i,j]):
                aa[i,j]  = 1 #mat[i,j]
            else:
                aa[i, j] = mat[i,j]
    return aa

fig = plt.figure(figsize=(10,10))
ax1 =  fig.add_subplot(2,1,1)          # 2*1 分布，第1个

# Display matrix
#ax1.matshow(samplemat(sc_matrix_lcoal_map))   #,cmap = plt.cm.gray
ax1.matshow(sc_matrix_lcoal_map)   #,cmap = plt.cm.gray
plt.title("sc_local_map");

ax2 =  fig.add_subplot(2,1,2)          # 2*1 分布，第2个
#ax2.matshow(samplemat(sc_matrix_cur_scan))   #,cmap = plt.cm.gray
ax2.matshow(sc_matrix_cur_scan)   #,cmap = plt.cm.gray
plt.title("sc_current_scan ");

plt.show()