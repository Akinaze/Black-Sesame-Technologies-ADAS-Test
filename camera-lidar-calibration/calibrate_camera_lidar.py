#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
 * Copyright (C) 2019-2020 Yirami .
 * All rights reserved.
 * @file    calibrate_camera_lidar.py
 * @brief   class for camera lidar calibration .
 * @author  [Tang zhiyong](https://yirami.xyz)
 * @date    2020-04-16
 * @version 1.0
 * @note
 * @history
"""

import os
import numpy as np
import cv2
import re
import argparse

from calibrate import SaveMode, Calibrate


def yaml_template(sign: str, rx=0, ry=0, rz=0, sx=0, sy=0, sz=0):
    content = rf"""# DECLARE
## camera coordinate:
##   Z(forward along the optic axis)
##   X(right)
##   Y(down)
# Step 1: Check signature
Signature: "{sign}"
# Step 2: Skip file header
SkipHeaderRows:
    csv: 1
    pcd: 11
# Step 3: Transform world(lidar) to right hand coordinate(same to camera)
## prior always, 1:=do nothing; -1:=reverse this axis
RHTransformMark:
    x: 1
    y: 1
    z: 1
## only when `RHTransformMark` is undefined
RHTransformMat: !!opencv-matrix
    rows: 3
    cols: 3
    dt: d
    data: [1, 0, 0,
           0, 1, 0,
           0, 0, 1]
# Step 4: Pick up lidar's data, nonnegative integer:=column; -1:=does't exist
PickLidarDataMark:
    x: 0
    y: 1
    z: 2
    d: -1
    i: 3
# Step 5(Optional): (For calibrate) Set initial posture
## unit: rotate angle(degree), shift distance(meter)
## how to: other(world) coordinate -> camera coordinate (rotate then shift)
InitialPosture:
    rx: {rx:.3f}
    ry: {ry:.3f}
    rz: {rz:.3f}
    sx: {sx:.3f}
    sy: {sy:.3f}
    sz: {sz:.3f}

"""
    return content


class CalibrateCameraLidar(Calibrate):
    def __init__(self, save_mode=SaveMode.Auto, save_path=None):
        super().__init__(save_mode=save_mode, save_path=save_path)
        # config header
        self._signature = None
        self._skip_rows = None
        self._rh_transfrom = None
        self._lidar_data_layout = None
        self._initial_pos = None
        #
        cm = np.arange(0, 256, 1, dtype=np.uint8)
        self.color_map = cv2.applyColorMap(cm, cv2.COLORMAP_RAINBOW)

    @staticmethod
    def _init_from_autoware(config_file_path: str):
        camera = Calibrate._default_camera()
        yaml_file = cv2.FileStorage(config_file_path, cv2.FILE_STORAGE_READ)
        size_node = yaml_file.getNode("ImageSize")

        # print(size_node, type(size_node))
        # print(size_node.at(0), type(size_node))
        # print(size_node.at(0).real(), type(size_node.at(0).real()))
        # print(yaml_file.getNode("CameraMat").mat())       

        camera['resolution'] = (int(size_node.at(0).real()), int(size_node.at(1).real()))
        camera['distortion'] = np.squeeze(yaml_file.getNode("DistCoeff").mat(), 0)
        camera['intrinsic'] = yaml_file.getNode("CameraMat").mat()
        camera['extrinsic'] = yaml_file.getNode("CameraExtrinsicMat").mat()  # camera -> lidar
        camera['extrinsic'] = np.linalg.inv(camera['extrinsic'])  # lidar -> camera
        yaml_file.release()

        # print(camera, type(camera))

        return camera

    def _init_from_yaml(self, config_file_path: str):
        try:  # Load presets according to default configuration
            yaml_file = cv2.FileStorage(config_file_path, cv2.FILE_STORAGE_READ)
            self._signature = yaml_file.getNode("Signature").string()
            # print(type(yaml_file))
            # print(type(yaml_file.getNode("Signature")))    
            # print(type(self._signature))  
            csv_skip = int(yaml_file.getNode('SkipHeaderRows').getNode('csv').real())
            pcd_skip = int(yaml_file.getNode('SkipHeaderRows').getNode('pcd').real())
            # print(type(csv_skip))
            # print(csv_skip, pcd_skip)
            self._skip_rows = {'csv': csv_skip, 'pcd': pcd_skip}
            if yaml_file.getNode('RHTransformMark').isNone():
                self._rh_transfrom = yaml_file.getNode('RHTransformMat').mat()
                assert self._rh_transfrom.shape == (3, 3)
            else:
                rh_x = yaml_file.getNode('RHTransformMark').getNode('x').real()
                rh_y = yaml_file.getNode('RHTransformMark').getNode('y').real()
                rh_z = yaml_file.getNode('RHTransformMark').getNode('z').real()
                rh = np.zeros((3, 3), np.float64)
                rh[0, 0] = rh_x
                rh[1, 1] = rh_y
                rh[2, 2] = rh_z
                self._rh_transfrom = rh
            all_need = ['x', 'y', 'z', 'd', 'i']
            self._lidar_data_layout = {x: int(yaml_file.getNode('PickLidarDataMark').getNode(x).real()) for x in all_need}
            # print(self._lidar_data_layout)
            self._lidar_data_layout = {k: None if v < 0 else v for k, v in self._lidar_data_layout.items()}
            # print(self._lidar_data_layout)    
            all_need = ['rx', 'ry', 'rz', 'sx', 'sy', 'sz']
            self._initial_pos = {x: float(yaml_file.getNode('InitialPosture').getNode(x).real()) for x in all_need}
            # print(self._initial_pos)
        except IOError:
            print(f"[YLog/Error]: config file <{config_file_path}> not exist!")
            raise RuntimeError
        except:
            print(f"[YLog/Error]: Nonstandard config <{config_file_path}> format!")
            raise RuntimeError
        finally:
            yaml_file.release()
        if self._signature.upper() == "AUTOWARE":
            self._camera = self._init_from_autoware(config_file_path)
        else:
            raise NotImplementedError
        # init calibrate
        self.para = {'step_deg': 0.1, 'step_m': 0.1}
        self.para.update(self._initial_pos)
        # print(self.para)

    @staticmethod
    def _rotate_zyx_and_shift_xyz(rz_deg, ry_deg, rx_deg, sx_m, sy_m, sz_m):
        sin_rx = np.sin(np.deg2rad(rx_deg))
        cos_rx = np.cos(np.deg2rad(rx_deg))
        sin_ry = np.sin(np.deg2rad(ry_deg))
        cos_ry = np.cos(np.deg2rad(ry_deg))
        sin_rz = np.sin(np.deg2rad(rz_deg))
        cos_rz = np.cos(np.deg2rad(rz_deg))
        rx = np.array([[1, 0, 0], [0, cos_rx, sin_rx], [0, -sin_rx, cos_rx]])
        ry = np.array([[cos_ry, 0, -sin_ry], [0, 1, 0], [sin_ry, 0, cos_ry]])
        rz = np.array([[cos_rz, sin_rz, 0], [-sin_rz, cos_rz, 0], [0, 0, 1]])
        transform = np.zeros((4, 4), dtype=np.float)
        transform[0:3, 0:3] = rx @ ry @ rz
        transform[:, 3] = np.asarray([sx_m, sy_m, sz_m, 1])
        return transform

    def _xyz2uv(self, xyzdi):
        # print(xyzdi) # xyzdi为pcd data前三列: x, y, z, indensity
        xyz_w = np.column_stack((xyzdi[:, 0:3], np.ones(xyzdi.shape[0], dtype=float)))
        # print(xyz_w) # 用1替换xyzdi的indensity
        # rotate and shift
        xyz_c = self._camera['extrinsic'] @ xyz_w.T
        # perspective
        camera = np.zeros((3, 4), dtype=np.float)
        print(camera)
        camera[0:3, 0:3] = self._camera['intrinsic']
        print(camera)
        uvd_pseudo = camera @ xyz_c
        uvd = uvd_pseudo
        uvd[0, :] = uvd[0, :] / uvd[2, :]
        uvd[1, :] = uvd[1, :] / uvd[2, :]
        uvd = uvd.T
        # distortion
        uu, vv = self._correct_distortion(uvd[:, 0], uvd[:, 1])
        uvd[:, 0] = uu
        uvd[:, 1] = vv
        # attach depth
        uvd[:, 2] = xyzdi[:, 3]
        # attach intensity
        uvdi = np.column_stack((uvd, xyzdi[:, 4]))
        # filter forward
        uvdi = uvdi[xyz_c[2, :] > 0, :]
        # filter outer
        uvdi = [uvdi[i, :] for i in range(uvdi.shape[0]) if not self._is_exceed_image_size(uvdi[i, 0], uvdi[i, 1])]
        return np.asarray(uvdi)

    def _fine_tune(self, key):
        captured = True
        if key in [ord('a'), ord('A')]:
            self.para['rz'] -= self.para['step_deg']
        elif key in [ord('d'), ord('D')]:
            self.para['rz'] += self.para['step_deg']
        elif key in [ord('w'), ord('W')]:
            self.para['rx'] -= self.para['step_deg']
        elif key in [ord('s'), ord('S')]:
            self.para['rx'] += self.para['step_deg']
        elif key in [ord('q'), ord('Q')]:
            self.para['ry'] -= self.para['step_deg']
        elif key in [ord('e'), ord('E')]:
            self.para['ry'] += self.para['step_deg']
        elif key == 49:  # number pad 1
            self.para['step_m'] = max(0.01, self.para['step_m'] - 0.01)
        elif key == 51:  # number pad 3
            self.para['step_m'] = min(0.5, self.para['step_m'] + 0.01)
        elif key == 56:  # number pad 8
            self.para['sy'] -= self.para['step_m']
        elif key == 50:  # number pad 2
            self.para['sy'] += self.para['step_m']
        elif key == 52:  # number pad 4
            self.para['sx'] -= self.para['step_m']
        elif key == 54:  # number pad 6
            self.para['sx'] += self.para['step_m']
        elif key == 55:  # number pad 7
            self.para['sz'] -= self.para['step_m']
        elif key == 57:  # number pad 9
            self.para['sz'] += self.para['step_m']
        else:
            captured = False
        return captured

    def _render(self, source, size_adapt):
        render = source['image'].copy()
        self._camera['extrinsic'] = self._rotate_zyx_and_shift_xyz(self.para['rz'], self.para['ry'], self.para['rx'], self.para['sx'], self.para['sy'], self.para['sz'])
        uvdi = self._xyz2uv(source['pts_cloud'])
        for pt in uvdi:
            # 反射率
            # gg = min(int(pt[3]), 255)
            # 距离
            gg = int(255 * float(int(pt[2]) % 80) / 80)  # 80米内均匀
            cc = self.color_map[gg, 0, :]
            color = (cc[0].item(), cc[1].item(), cc[2].item())
            cv2.circle(render, (int(pt[0]), int(pt[1])), 2, color, thickness=-1)
        info = "Keyboard info : Pitch(W/S) Yaw(A/D) Roll(Q/E) Save(K) Quit(Esc)"
        cv2.putText(render, info, (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)
        info = "SX(NP4/NP6) SY(NP8/NP2) SZ(NP7/NP9) Step(NP1/NP3)"
        cv2.putText(render, info, (50, 100), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)
        str_ex = f"Camera params : " \
                 f"shift step:{self.para['step_m']:.2f} (m) "\
                 f"sx:{self.para['sx']:.2f} " \
                 f"sy:{self.para['sy']:.2f} " \
                 f"sz:{self.para['sz']:.2f} (m) " \
                 f"rx:{self.para['rx']:.2f} " \
                 f"ry:{self.para['ry']:.2f} " \
                 f"rz:{self.para['rz']:.2f} (deg)"
        cv2.putText(render, str_ex, (50, 150), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)
        if size_adapt:
            wr = self._camera['resolution'][0] / 1920
            hr = self._camera['resolution'][1] / 1080
            mr = wr if wr > hr else hr
            if mr > 0.8:
                size = (int(0.8 * self._camera['resolution'][0] / mr), int(0.8 * self._camera['resolution'][1] / mr))
                render = cv2.resize(render, size, interpolation=cv2.INTER_CUBIC)
        cv2.imshow("calibrate", render)

    def _rewrite_to_yaml(self, config_file_path):
        yaml_file = cv2.FileStorage(config_file_path, cv2.FILE_STORAGE_WRITE)
        yaml_file.writeComment("Raw Data: camera coordinate -> other(lidar) coordinate")
        world2camera = self._camera['extrinsic']
        yaml_file.write('CameraExtrinsicMat', np.linalg.inv(world2camera))  # camera -> world
        yaml_file.write('CameraMat', self._camera['intrinsic'])
        yaml_file.write('DistCoeff', self._camera['distortion'][np.newaxis, :])
        yaml_file.release()
        with open(config_file_path, 'r+') as yaml_file:
            content = yaml_file.readlines()
            insert_idx = content.index('---\n')
            rewrite_items = ['rx', 'ry', 'rz', 'sx', 'sy', 'sz']
            rewrite_init = {it: self.para[it] for it in rewrite_items}
            new_content = ''.join(content[:insert_idx + 1]) + yaml_template('Autoware', **rewrite_init) + ''.join(content[insert_idx + 1:])
            yaml_file.seek(0)
            yaml_file.write(new_content)
            yaml_file.write(f"ImageSize: [ {self._camera['resolution'][0]}, {self._camera['resolution'][1]} ]\n")
            yaml_file.write('ReprojectionError: "Unavailable"')

    def load_source(self, source_path: dict):
        name_reg = r"_(\d+\.\d+)([\.\w]+)$"
        image = cv2.imread(source_path['image'])
        pts_file_ext = os.path.splitext(source_path['pts_cloud'])[-1]
        itime = re.search(name_reg, os.path.split(source_path['image'])[-1])
        if itime:
            itime = itime.group(1)
        ptime = re.search(name_reg, os.path.split(source_path['pts_cloud'])[-1])
    
        if ptime:
            ptime = ptime.group(1)

        # check layout
        must_have = {'x', 'y', 'z', 'i'}
        chk_layout = {k for k, v in self._lidar_data_layout.items() if v is not None}
        if must_have - chk_layout:
            raise RuntimeError
        if pts_file_ext == '.csv':
            pts_cloud = np.loadtxt(source_path['pts_cloud'], skiprows=self._skip_rows['csv'], dtype=np.str, delimiter=",")
        elif pts_file_ext == '.pcd':
            pts_cloud = np.loadtxt(source_path['pts_cloud'], skiprows=self._skip_rows['pcd'], dtype=np.str)
        else:
            raise NotImplementedError
        if self._lidar_data_layout['d'] is None:
            pick = [self._lidar_data_layout['x'], self._lidar_data_layout['y'], self._lidar_data_layout['z'], self._lidar_data_layout['i']]
            pts_3d = pts_cloud[:, pick].astype(np.float)
 
            pts_dis = np.sqrt(np.sum(np.power(pts_3d[:, :3], 2), axis=1))
            pts_3d = np.insert(pts_3d, 3, pts_dis, axis=1)
        else:
            pick = [self._lidar_data_layout['x'], self._lidar_data_layout['y'], self._lidar_data_layout['z'], self._lidar_data_layout['d'], self._lidar_data_layout['i']]
            pts_3d = pts_cloud[:, pick].astype(np.float)
        if itime is None or ptime is None:
            return {'image': image, 'pts_cloud': pts_3d}
        else:
            return {'image': image, 'image_timestamp': itime, 'pts_cloud': pts_3d, 'pts_timestamp': ptime}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Parse arguments for lidar calibrate demo.')
    parser.add_argument('-c', '--config', default=r"./data/hesai1013/camera_lidar.yaml", help="Please input config file's path, maybe a yaml file.")
    parser.add_argument('-i', '--image', default=r"./data/hesai1013/camera_1_1602574158.97533393.jpeg", help="Please input images file path.")
    parser.add_argument('-l', '--lidar', default=r"./data/hesai1013/hesai_1602574158.881494.pcd", help="Please input lidar file path.")
    args = parser.parse_args()
    CALIB_YAML_PATH = args.config
    IMAGE_PATH = args.image
    POINTS_CLOUD_PATH = args.lidar

    demo_calib = CalibrateCameraLidar()
    demo_calib.init_camera(CALIB_YAML_PATH)
    demo_calib.calibrate(demo_calib.load_source({'image': IMAGE_PATH, 'pts_cloud': POINTS_CLOUD_PATH}))
