#!/usr/bin/env python3

import open3d as o3d
import numpy as np


import do_pe
import helpers
import settings

scene_id = settings.indexes[0]
noise_level = settings.noise_levels[0]

def main():
    scene_pointcloud_file_name = settings.input_folder + 'scene_' + str(scene_id) + '.pcd'
    scene_pointcloud = o3d.io.read_point_cloud(scene_pointcloud_file_name)
    
    scene_pointcloud_noisy = helpers.add_noise(scene_pointcloud, 0, noise_level)
    #Change parameters when creating object pc
    object_mesh = o3d.io.read_triangle_mesh(settings.input_folder + "obj_000009_mm.stl")
    object_pointcloud = object_mesh.sample_points_poisson_disk(10000)
    object_pointcloud.colors = o3d.utility.Vector3dVector(np.zeros_like(object_pointcloud.points) + [255,0,255])
    o3d.visualization.draw_geometries([object_pointcloud, scene_pointcloud_noisy], window_name='Pre alignment')

    estimated_pose = do_pe.do_pose_estimation(scene_pointcloud_noisy, object_pointcloud)
    

    print("Final pose")
    print (estimated_pose)

    ground_truth = np.loadtxt(settings.input_folder + "gt_" + str(scene_id) + ".txt")

    print("Ground truth")
    print(ground_truth)

    print("Error")
    error_angle, error_pos = helpers.computeError(ground_truth,estimated_pose)
    print(error_angle, "degrees")
    print(error_pos, "mm")
    #object_pointcloud.colors = o3d.utility.Vector3dVector(np.zeros_like(object_pointcloud.points) + [255,0,255])
    o3d.visualization.draw_geometries([object_pointcloud.transform(estimated_pose), scene_pointcloud_noisy], window_name='Final alignment')

if __name__ == "__main__":
    main()