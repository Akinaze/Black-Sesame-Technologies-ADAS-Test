#!/usr/bin/python3
# -*- coding: utf-8 -*-
'''
Copyright (C) 2019-2020 Yirami.
All rights reserved.

@Brief: Abstract class for camera calibration .
@Author: [yirami.tang](https://yirami.xyz)
@Email: i@yirami.xyz
@Since: 2020-04-16 10:28:57
LastEditors: Haorong Jiang
LastEditTime: 2021-12-09 11:19:29
@Version: 1.0
'''

import abc
import os
import sys
import shutil
import time
import numpy as np
from enum import Enum
import cv2

Y_COORDINATE_DECLARATION = \
"""
Coordinate System

  world coordinate                        camera coordinate
          ^ Z       ^ X                               ^ Z
          |        /                                /
          |      /                                /
          |    /                                O - - - - -> X
          |  /                                  |
          |/                                    |
 Y <- - - O                                     v Y

"""


class ErrorCode(Enum):
    Normal = 0
    InitError = 1
    DistortionError = 2


class SaveMode(Enum):
    Auto = 0
    Override = 1
    SafeOverride = 2
    YJson = 3


class Calibrate:
    def __init__(self, save_mode=SaveMode.Auto, save_path=None):
        self._camera = self._default_camera()
        self._error_code = ErrorCode.Normal
        print(Y_COORDINATE_DECLARATION)
        self.para = {}
        assert isinstance(save_mode, SaveMode)
        self._save_mode = save_mode
        self._save_path = save_path

    @staticmethod
    def _default_camera():
        return {'resolution': [], 'distortion': [], 'intrinsic': [], 'extrinsic': []}  # other coordinate -> camera coordinate

    @abc.abstractmethod
    def _init_from_yaml(self, config_file_path: str):
        raise NotImplementedError

    def init_camera(self, config_file_path: str):
        if os.path.splitext(config_file_path)[-1] in ('.yml', '.yaml'):
            if self._save_mode == SaveMode.Auto or self._save_mode == SaveMode.SafeOverride:
                self._save_mode = SaveMode.SafeOverride
                self._save_path = config_file_path
            elif self._save_mode == SaveMode.Override:
                self._save_path = config_file_path
            elif self._save_mode == SaveMode.YJson:
                pass
            self._init_from_yaml(config_file_path)
        else:
            self._error_code = ErrorCode.InitError
            raise RuntimeError

    def _is_exceed_image_size(self, u, v):
        return not (0 <= u < self._camera['resolution'][0] and 0 <= v < self._camera['resolution'][1])

    def _pixel_uv2xy(self, u, v):
        x = (u - self._camera['intrinsic'][0][2]) / self._camera['intrinsic'][0][0]
        y = (v - self._camera['intrinsic'][1][2]) / self._camera['intrinsic'][1][1]
        return x, y

    def _pixel_xy2uv(self, x, y):
        u = x * self._camera['intrinsic'][0][0] + self._camera['intrinsic'][0][2]
        v = y * self._camera['intrinsic'][1][1] + self._camera['intrinsic'][1][2]
        return u.astype(np.int), v.astype(np.int)

    def _correct_distortion(self, u, v):
        if not len(self._camera['distortion']):
            return u, v
        elif len(self._camera['distortion']) < 4:
            self._error_code = ErrorCode.Normal
        x, y = self._pixel_uv2xy(u, v)
        r = np.square(x) + np.square(y)
        k1 = self._camera['distortion'][0]
        k2 = self._camera['distortion'][1]
        p1 = self._camera['distortion'][2]
        p2 = self._camera['distortion'][3]
        k_radial = (1 + k1 * r + k2 * r * r)
        tan_x = 2 * p1 * x * y + p2 * (r + 2 * x * x)
        tan_y = p1 * (r + 2 * y * y) + 2 * p2 * x * y
        cx = x * k_radial + tan_x
        cy = y * k_radial + tan_y
        return self._pixel_xy2uv(cx, cy)

    def _fine_tune(self, key):
        captured = False
        return captured

    @abc.abstractmethod
    def _render(self, source, size_adapt):
        raise NotImplementedError

    @abc.abstractmethod
    def _rewrite_to_yaml(self, config_file_path):
        raise NotImplementedError

    def _save(self):
        assert self._save_mode != SaveMode.Auto
        if self._save_mode == SaveMode.Override:
            pass
        elif self._save_mode == SaveMode.SafeOverride:
            file_name, file_extend = os.path.splitext(self._save_path)
            backup = file_name + '_backup' + time.strftime("%Y%m%d%H%M%S", time.localtime()) + file_extend
            shutil.copyfile(self._save_path, backup)
            self._rewrite_to_yaml(self._save_path)
        elif self._save_mode == SaveMode.YJson:
            pass
        print(f"Save calibration file {self._save_path} success!")
        sys.exit()

    @staticmethod
    @abc.abstractmethod
    def load_source(source_path):
        raise NotImplementedError

    def calibrate(self, source, size_adapt=True):
        self._render(source, size_adapt)
        while True:
            key = cv2.waitKey(30)
            if key != -1:  # Key pressed
                if key == 27:  # ESC
                    break
                elif key in [ord('k'), ord('K')]:
                    self._save()
                elif self._fine_tune(key):
                    self._render(source, size_adapt)


if __name__ == '__main__':
    pass
