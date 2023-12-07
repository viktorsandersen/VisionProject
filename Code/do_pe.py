import open3d as o3d
import copy
import numpy as np
from tqdm import tqdm
import random
#For testing
import helpers
import settings

#TODO: Filter/segment point cloud 
#Create object point cloud (adjust parameters)
#Local pose estimation to refine object pose
#Method's performance: Binary outcome summarised as percentages of cases where the method works
#Also the pose error when the pose estimate works (is it close to the real pose) by filtering out the failed cases and then calculate the distance to the real pose
#Done independently for orientation and position
#Lec 5 - points clouds, Lec 4.1 - depth sensing, lec 6/7 - xxx -> 3D pose, Lec 9 - kalman filter, lec 10 - particle filter

# This function just displays the effect of one of the functions visually, feel free to ignore or remove it.
def display_removal(preserved_points, removed_points):
    removed_points.paint_uniform_color([1, 0, 0])        # Show removed points in red
    preserved_points.paint_uniform_color([0.8, 0.8, 0.8])# Show preserved points in gray
    o3d.visualization.draw_geometries([removed_points, preserved_points])

def voxel_grid(input_cloud):
    voxel_down_cloud = input_cloud.voxel_down_sample(voxel_size=0.001)
    return voxel_down_cloud

def outlier_removal(input_cloud):
    cl, ind = input_cloud.remove_statistical_outlier(nb_neighbors=200, std_ratio=0.01)
    #display_removal(input_cloud.select_by_index(ind), input_cloud.select_by_index(ind, invert=True))
    return input_cloud.select_by_index(ind)

def spatial_filter(input_cloud):
    passthrough = input_cloud.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=(-float('inf'), -float('inf'), 260.0),
                                                                        max_bound=(float('inf'), float('inf'), 320.0)))
    #display_removal(passthrough, input_cloud)
    return passthrough

def do_pose_estimation(scene_pointcloud, object_pointcloud):
    scene_id = settings.indexes[0]
    ground_truth = np.loadtxt(settings.input_folder + "gt_" + str(scene_id) + ".txt")
    def GP(scene_pointcloud, object_pointcloud):
    #################################
    ##### Global pose estimation ####
    #################################
        it = 2000 #Was 1000 - 5000 works better
        thressq = 0.01**2
        globalPose = None

        # Compute surface normals
        object_pointcloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(10))
        scene_pointcloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(10))

        # Compute shape features
        obj_features = o3d.pipelines.registration.compute_fpfh_feature(object_pointcloud, search_param=o3d.geometry.KDTreeSearchParamRadius(0.05))
        scn_features = o3d.pipelines.registration.compute_fpfh_feature(scene_pointcloud, search_param=o3d.geometry.KDTreeSearchParamRadius(0.05))
        obj_features = np.asarray(obj_features.data).T
        scn_features = np.asarray(scn_features.data).T

        #For test
        noise_level = settings.noise_levels[0]
        scene_pointcloud_noisy = helpers.add_noise(scene_pointcloud, 0, noise_level)

        # Find feature matches
        corr = o3d.utility.Vector2iVector()
        for j in tqdm(range(obj_features.shape[0]), desc='Correspondences'):
            fobj = obj_features[j]
            dist = np.sum((fobj - scn_features)**2, axis=-1)
            kmin = np.argmin(dist)
            corr.append((j, kmin))

        # Create a k-d tree for scene
        tree = o3d.geometry.KDTreeFlann(scene_pointcloud)

        # Start RANSAC
        random.seed(123456789)
        inliers_best = 0
        for i in tqdm(range(it), desc='RANSAC'):   
            # Sample 3 random correspondences
            corri = o3d.utility.Vector2iVector(random.choices(corr, k=3))
            
            # Estimate transformation
            est = o3d.pipelines.registration.TransformationEstimationPointToPoint()
            T = est.compute_transformation(object_pointcloud, scene_pointcloud, corri)
            
            # Apply pose
            obj_aligned = o3d.geometry.PointCloud(object_pointcloud)
            obj_aligned.transform(T)
            
            # Validate
            inliers = 0
            for j in range(len(obj_aligned.points)):
                k, idx, dist = tree.search_knn_vector_3d(obj_aligned.points[j], 1)
                if dist[0] < thressq:
                    inliers += 1

            # Update result
            if inliers > inliers_best:
                print(f'Got a new model with {inliers}/{len(obj_aligned.points)} inliers!')
                inliers_best = inliers
                pose = T
                globalPose = pose
        #For test
        

        print("Ground truth global")
        print(ground_truth)

        print("Error global")
        print(helpers.computeError(ground_truth,globalPose))
        o3d.visualization.draw_geometries([object_pointcloud.transform(globalPose), scene_pointcloud], window_name='Final alignment')
        
        # Print pose
        print('Got the following pose:')
        print(pose)
        return globalPose 
    def LP(scene_pointcloud, object_pointcloud, globalPose):
    ###############################    
    ####### Local alignment #######
    ###############################
        # Create a k-d tree for scene
        #Lscene_pointcloud = scene_pointcloud.transform(globalPose)
        Lscene_pointcloud = scene_pointcloud
        Ltree = o3d.geometry.KDTreeFlann(Lscene_pointcloud)

        # Set ICP parameters
        Lit = 200  # Adjust as needed
        Lthressq = 0.01 ** 2  # Adjust as needed

        # Initialize LocalPose with globalPose
        LocalPose = globalPose

        # Transform object_pointcloud using globalPose
        Lobject_pointcloud = object_pointcloud.transform(globalPose)
        Lobj_aligned = o3d.geometry.PointCloud(Lobject_pointcloud)

        for i in tqdm(range(Lit), desc='ICP'):
            # 1) Find closest points
            LocalCorr = o3d.utility.Vector2iVector()
            for j in range(len(Lobj_aligned.points)):
                k, Lidx, Ldist = Ltree.search_knn_vector_3d(Lobj_aligned.points[j], 1)

                # Apply distance threshold to correspondences
                if Ldist[0] < Lthressq:
                    LocalCorr.append((j, Lidx[0]))

            # 2) Estimate transformation
            Lest = o3d.pipelines.registration.TransformationEstimationPointToPoint()
            LT = Lest.compute_transformation(Lobj_aligned, scene_pointcloud, LocalCorr)

            # 3) Apply pose
            Lobj_aligned.transform(LT)

            # 4) Update result
            LocalPose = LT @ LocalPose

            # Visualize alignment during each iteration (optional)
            if i % 50 == 0:
                print("Error local")
                print(helpers.computeError(ground_truth,LocalPose))
                o3d.visualization.draw_geometries([Lobj_aligned, scene_pointcloud], window_name='Alignment during ICP')

        print("Ground truth local")
        print(ground_truth)

        print("Error local")
        print(helpers.computeError(ground_truth,LocalPose), "LocalPose")
        print(helpers.computeError(ground_truth,LT), "LT")
        #print(helpers.computeError(Lobj_aligned,localPose), "Lobj_aligned")
        return LocalPose
    
    print("PointCloud before filtering: {} data points".format(len(scene_pointcloud.points)))
    #o3d.visualization.draw_geometries([scene_pointcloud], window_name = 'Pointcloud before filtering')
    scene_filtered = voxel_grid(scene_pointcloud)
    scene_filtered = outlier_removal(scene_filtered)
    #scene_filtered = spatial_filter(scene_filtered)
    #scene_pointcloud = scene_filtered
    print("PointCloud after filtering: {} data points".format(len(scene_filtered.points)))
    #o3d.visualization.draw_geometries([scene_filtered], window_name = 'Pointcloud after filtering')
    GlobalPose = GP(scene_filtered, object_pointcloud)
    LocalPose = LP(scene_filtered, object_pointcloud, GlobalPose)
    print("Error final")
    print(helpers.computeError(ground_truth,LocalPose), "LocalPose")
    return LocalPose
    #return globalPose
    #return np.identity(4)