import open3d as o3d
import numpy as np
local_map_pcd = o3d.io.read_point_cloud("pcd/local_map.pcd")
cur_scan_pcd = o3d.io.read_point_cloud("pcd/cur_scan.pcd")

local_map_points = np.asarray(local_map_pcd.points)
cur_scan_points = np.asarray(cur_scan_pcd.points)


# print(local_map_pcd)#输出点云点的个数
# print(cur_scan_pcd)#输出点云点的个数
#
print(local_map_points)#输出点的三维坐标
# print(cur_scan_points)#输出点的三维坐标
#
# print('给所有的点上一个统一的颜色，颜色是在RGB空间得[0，1]范围内得值')

local_map_pcd.paint_uniform_color([0.9, 0.9, 0.9])        # blue
cur_scan_pcd.paint_uniform_color([1, 0, 0])        # red


o3d.visualization.draw_geometries([local_map_pcd,cur_scan_pcd])