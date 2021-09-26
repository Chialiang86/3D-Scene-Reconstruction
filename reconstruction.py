import open3d as o3d
import cv2 
import numpy as np
import glob

rgb_path = 'Data_collection/1/rgb/'
depth_path = 'Data_collection/1/depth/'
total_files = len(glob.glob('{}/*.png'.format(rgb_path)))

# set intrincsic function
WIDTH = 512
HEIGHT = 512
FOV = 0.5 * np.pi 
FX = WIDTH * np.tan(FOV / 2)
FY = HEIGHT * np.tan(FOV / 2)
CX = WIDTH / 2
CY = HEIGHT / 2
INTRINSIC = [WIDTH, HEIGHT, FX, FY, CX, CY]

def depth_image_to_point_cloud(rgb, depth, intrinsic):
    # convert to rgbd images
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb, depth, convert_rgb_to_intensity=False)
    # create point cloud by the given intrinsic parameters and rgbd images
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image,
        o3d.camera.PinholeCameraIntrinsic(intrinsic[0], intrinsic[1], intrinsic[2], intrinsic[3], intrinsic[4], intrinsic[5])
    )
    return pcd

def local_icp_algorithm(pc1, pc2, init_trans=None, thresh=0.001):
    return 0

demo_color = o3d.io.read_image('{}10.png'.format(rgb_path))
demo_depth = o3d.io.read_image('{}10.png'.format(depth_path))

pcd = depth_image_to_point_cloud(demo_color, demo_depth, INTRINSIC)
pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
print(pcd)
o3d.visualization.draw_geometries([pcd])