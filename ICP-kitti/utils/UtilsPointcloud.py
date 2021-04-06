import os 
import random
import numpy as np

def random_sampling(orig_points, num_points):
#    print (orig_points)
    assert orig_points.shape[0] > num_points

    points_down_idx = random.sample(range(orig_points.shape[0]), num_points)
    down_points = orig_points[points_down_idx, :]

    return down_points

def readScan(bin_path, dataset='KITTI'):
    if(dataset == 'KITTI'):
        return readKittiScan(bin_path)


def readKittiScan(bin_path):
    scan = np.fromfile(bin_path, dtype=np.float32)
    scan = scan.reshape((-1, 4))
    ptcloud_xyz = scan[:, :-1]
    return ptcloud_xyz
    


