import numpy as np
import copy
import sdurw_math as sdurw_m
import open3d as o3d
import math

def add_noise(pcd, mu, sigma):
    noisy_pcd = copy.deepcopy(pcd)
    points = np.asarray(noisy_pcd.points)
    points += np.random.normal(mu, sigma, size=points.shape)
    noisy_pcd.points = o3d.utility.Vector3dVector(points)
    return noisy_pcd

def numpyToTransform3D(transform_np):
    assert(transform_np.shape[0] == 4)
    assert(transform_np.shape[1] == 4)

    translation_rw = sdurw_m.Vector3D(transform_np[0:3,3])
    rotation_rw = sdurw_m.Rotation3D(
        transform_np[0,0],
        transform_np[0,1],
        transform_np[0,2],
        transform_np[1,0],
        transform_np[1,1],
        transform_np[1,2],
        transform_np[2,0],
        transform_np[2,1],
        transform_np[2,2]
    )

    transform_rw = sdurw_m.Transform3D(translation_rw, rotation_rw)

    return transform_rw

def computeError(ground_truth, estimate_pose):
    gt_rw = numpyToTransform3D(ground_truth)
    ep_rw = numpyToTransform3D(estimate_pose)

    r_diff = ep_rw.R() * gt_rw.R().inverse()

    # Rotation error in degrees
    error_angle = math.degrees(sdurw_m.EAA(r_diff).angle())
    # Position error in mm
    error_pos = (gt_rw.P() - ep_rw.P()).norm2() * 1000

    return error_angle, error_pos

def filter_errors(errors, max_rotation_error, max_position_error):
    result = []
    for e in errors:
        if e[0] > max_rotation_error or e[1] > max_position_error:
            continue
        else:
            result.append(e)
    return result