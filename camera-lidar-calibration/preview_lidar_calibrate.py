#!/usr/bin/python3
# -*- coding: utf-8 -*-
'''
Copyright (C) 2020 Yirami.
All rights reserved.

@Brief: Preview lidar-camera calibration.
@Author: [yirami.tang](https://yirami.xyz)
@Email: i@yirami.xyz
@Since: 2020-09-21 10:49:27
@LastEditors: Yirami
@LastEditTime: 2020-11-06 17:20:02
@Version: 
'''

import os
import shutil
import numpy as np
import cv2
import re
import argparse

from calibrate_camera_lidar import CalibrateCameraLidar


class CameraLidarPreviewer(CalibrateCameraLidar):
    def __init__(self):
        super().__init__()

    def _render(self, source: dict, size_adapt: bool):
        if source['image'] is None: return None  # 图像存在问题，不进行处理
        render = source['image'].copy()
        uvdi = self._xyz2uv(source['pts_cloud'])
        for pt in uvdi:
            # 反射率
            # gg = min(int(pt[3]), 255)
            # 距离
            gg = int(255 * float(int(pt[2]) % 200) / 200)  # 200米内均匀
            cc = self.color_map[gg, 0, :]
            color = (cc[0].item(), cc[1].item(), cc[2].item())
            cv2.circle(render, (int(pt[0]), int(pt[1])), 2, color, thickness=-1)
        info = f"Image timestamp: {source['image_timestamp']}"
        cv2.putText(render, info, (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)
        info = f"Lidar  timestamp: {source['pts_timestamp']}"
        cv2.putText(render, info, (50, 100), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)
        if size_adapt:
            wr = self._camera['resolution'][0] / 1920
            hr = self._camera['resolution'][1] / 1080
            mr = wr if wr > hr else hr
            if mr > 0.8:
                size = (int(0.8 * self._camera['resolution'][0] / mr), int(0.8 * self._camera['resolution'][1] / mr))
                render = cv2.resize(render, size, interpolation=cv2.INTER_CUBIC)
        return render

    def preview(self, source, size_adapt: bool = True):
        if isinstance(source, dict):
            render = self._render(source, size_adapt)
            cv2.imshow("preview", render)
            cv2.waitKey(500000)
        else:
            for src in source:
                assert isinstance(src['image'], str) and isinstance(src['pts_cloud'], str)
                render = self._render(self.load_source(src), size_adapt)
                if render is None:
                    print(f"[YLog/Error]: Preview aborted, because of empty image.")
                    break
                if 'save_to' in src:
                    save_name = os.path.split(src['image'])[-1]
                    cv2.imwrite(os.path.join(src['save_to'], save_name), render)
                else:
                    cv2.imshow("preview", render)
                    cv2.waitKey(50)

    def load_source(self, source_path, lidar_time_adj: float = None, save_preview: bool = True):
        if os.path.isfile(source_path['image']):
            assert os.path.isfile(source_path['image']) == os.path.isfile(source_path['pts_cloud'])
            return super().load_source(source_path)
        else:
            matched_source = []
            all_images = [(source_path['image'], fn, re.search(r"_(\d+\.\d+)([\.\w]+)$", fn).group(1)) for fn in os.listdir(source_path['image'])]
            if lidar_time_adj:
                all_pcd = [(source_path['pts_cloud'], fn, str(float(re.search(r"_(\d+\.\d+)([\.\w]+)$", fn).group(1)) + lidar_time_adj)) for fn in os.listdir(source_path['pts_cloud'])]
            else:
                all_pcd = [(source_path['pts_cloud'], fn, re.search(r"_(\d+\.\d+)([\.\w]+)$", fn).group(1)) for fn in os.listdir(source_path['pts_cloud'])]
            all_src = all_images + all_pcd
            all_src.sort(key=lambda xx: xx[-1])
            save_path = os.path.join(os.path.split(source_path['image'])[0], f'save_{str(lidar_time_adj)}' if lidar_time_adj else 'save')
            for idx, pack in enumerate(all_src):
                if os.path.splitext(pack[1])[-1] == '.pcd':
                    pre_pack = None if idx == 0 else all_src[idx - 1]
                    next_pack = None if idx == len(all_src) - 1 else all_src[idx + 1]
                    if pre_pack and os.path.splitext(pre_pack[1])[-1] == '.pcd':
                        pre_pack = None
                    if next_pack and os.path.splitext(next_pack[1])[-1] == '.pcd':
                        next_pack = None
                    if pre_pack and next_pack:
                        curr_time = float(pack[-1])
                        pre_time = float(pre_pack[-1])
                        next_time = float(next_pack[-1])
                        pick_pack = pre_pack if curr_time - pre_time < next_time - curr_time else next_pack
                    elif pre_pack:
                        pick_pack = pre_pack
                    elif next_pack:
                        pick_pack = next_pack
                    else:
                        continue
                    if save_preview:
                        matched_source.append({'image': os.path.join(pick_pack[0], pick_pack[1]), 'pts_cloud': os.path.join(pack[0], pack[1]), 'save_to': save_path})
                    else:
                        matched_source.append({'image': os.path.join(pick_pack[0], pick_pack[1]), 'pts_cloud': os.path.join(pack[0], pack[1])})
            if matched_source:
                if save_preview:
                    if os.path.exists(save_path):
                        shutil.rmtree(save_path, ignore_errors=True)
                    os.mkdir(save_path)
            else:
                print("[YLog/Error]: No pcd files.")
            return matched_source


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Parse arguments for preview demo.')
    parser.add_argument('-c', '--config', default=r"./data/camera_hesai/camera_lidar.yaml", help="Please input config file's path, maybe a yaml file.")
    parser.add_argument('-d', '--data', default=None, help="Please input data acquisition path.")
    parser.add_argument('-i', '--image', default=r"./data/camera_hesai/camera_1_1602574158.97533393.jpeg", help="Please input images file or dir path.")
    parser.add_argument('-l', '--lidar', default=r"./data/camera_hesai/hesai_1602574158.881494.pcd", help="Please input lidar file or dir path.")
    parser.add_argument('-o', '--offset', default=None, help="Please input offset of lidar's timestamp (positive if lidar earlier and unit in sec).")
    parser.add_argument('-s', '--save', action="store_true", help="Save or show preview directly, default save to `save_<adj_time>`.")
    args = parser.parse_args()
    CALIB_YAML_PATH = args.config
    DATA_ACQ = args.data
    previewer = CameraLidarPreviewer()
    previewer.init_camera(CALIB_YAML_PATH)
    if DATA_ACQ:
        assert os.path.isdir(DATA_ACQ)
        if os.path.isdir(os.path.join(DATA_ACQ, 'tmp')):  # 单个采集包
            IMAGE_PATH = os.path.join(DATA_ACQ, 'tmp', 'ramdisk', 'camera1')
            POINTS_CLOUD_PATH = os.path.join(DATA_ACQ, 'tmp', 'ramdisk', 'hesai_pcd')
            previewer.preview(previewer.load_source({'image': IMAGE_PATH, 'pts_cloud': POINTS_CLOUD_PATH}, float(args.offset), save_preview=args.save))
            print(f"Preview {DATA_ACQ} DONE.")
        else:  # 采集包文件夹
            all_dirs = [dd for dd in os.listdir(DATA_ACQ) if os.path.isdir(os.path.join(DATA_ACQ, dd, 'tmp'))]
            for dd in all_dirs:
                IMAGE_PATH = os.path.join(DATA_ACQ, dd, 'tmp', 'ramdisk', 'camera1')
                POINTS_CLOUD_PATH = os.path.join(DATA_ACQ, dd, 'tmp', 'ramdisk', 'hesai_pcd')
                previewer.preview(previewer.load_source({'image': IMAGE_PATH, 'pts_cloud': POINTS_CLOUD_PATH}, float(args.offset), save_preview=args.save))
                print(f"Preview {os.path.join(DATA_ACQ, dd)} DONE.")
    else:  # 直接指定图像、激光雷达数据路径
        IMAGE_PATH = args.image
        POINTS_CLOUD_PATH = args.lidar
        previewer.preview(previewer.load_source({'image': IMAGE_PATH, 'pts_cloud': POINTS_CLOUD_PATH}, save_preview=args.save))
    pass
