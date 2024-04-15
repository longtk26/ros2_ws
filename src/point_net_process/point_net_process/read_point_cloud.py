#!/usr/bin/env python3
import numpy as np
from mmdet3d.apis import init_model, inference_detector

class  PointCloudML:
    def __init__(self):
        dataset = '/home/ngoclong/ros2_ws/data/2011_09_26/2011_09_26_drive_0020_sync/velodyne_points/data/0000000010.bin'
        config_file = '/home/ngoclong/ros2_ws/src/point_net_process/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-car.py'
        checkpoint_file = '/home/ngoclong/ros2_ws/src/point_net_process/hv_pointpillars_secfpn_6x8_160e_kitti-3d-car_20220331_134606-d42d15ed.pth'
        model = init_model(config_file, checkpoint_file)
        inference_detector(model, dataset)

def main():
    

    PointCloudML()
    



if __name__ == '__main__':
    main()