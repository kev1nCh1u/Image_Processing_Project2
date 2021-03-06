import open3d as o3d
import copy
import numpy as np

# 2
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])

# 3
source = o3d.io.read_point_cloud("data\STN6xyzi.txt.pcd")
target = o3d.io.read_point_cloud("data\STN7xyzi.txt.pcd")
threshold = 0.02
trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
                         [-0.139, 0.967, -0.215, 0.7],
                         [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])
# trans_init = np.asarray([[1, 0, 0, 0],
#                         [0, 1, 0, 0],
#                         [0, 0, 1, 0], [0.0, 0.0, 0.0, 1.0]])
draw_registration_result(source, target, trans_init)

# 4
print("Initial alignment")
evaluation = o3d.pipelines.registration.evaluate_registration(
    source, target, threshold, trans_init)
print(evaluation)

# 5
print("Apply point-to-point ICP")
reg_p2p = o3d.pipelines.registration.registration_icp(
    source, target, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint())
print(reg_p2p)
print("Transformation is:")
print(reg_p2p.transformation)
draw_registration_result(source, target, reg_p2p.transformation)

# 6
reg_p2p = o3d.pipelines.registration.registration_icp(
    source, target, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
print(reg_p2p)
print("Transformation is:")
print(reg_p2p.transformation)
draw_registration_result(source, target, reg_p2p.transformation)

# 7
# print("Apply point-to-plane ICP")
# reg_p2l = o3d.pipelines.registration.registration_icp(
#     source, target, threshold, trans_init,
#     o3d.pipelines.registration.TransformationEstimationPointToPlane())
# print(reg_p2l)
# print("Transformation is:")
# print(reg_p2l.transformation)
# draw_registration_result(source, target, reg_p2l.transformation)

# 7-1
# print("Robust point-to-plane ICP, threshold={}:".format(threshold))
# loss = o3d.pipelines.registration.TukeyLoss(k=sigma)
# print("Using robust loss:", loss)
# p2l = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss)
# reg_p2l = o3d.pipelines.registration.registration_icp(source_noisy, target,
#                                                       threshold, trans_init,
#                                                       p2l)
# print(reg_p2l)
# print("Transformation is:")
# print(reg_p2l.transformation)
# draw_registration_result(source, target, reg_p2l.transformation)

# 8
def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result

result_icp = refine_registration(source, target, source_fpfh, target_fpfh,
                                 voxel_size)
print(result_icp)
draw_registration_result(source, target, result_icp.transformation)