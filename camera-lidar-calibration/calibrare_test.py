'''
Description: calibrare_test.py
Author: Haorong Jiang
Date: 2021-12-14 17:04:45
LastEditors: Haorong Jiang
LastEditTime: 2021-12-15 00:17:13
version: 1.0
'''

import argparse
import json
import os

import cv2
from matplotlib import image
import matplotlib.pyplot as plt
import numpy as np


class Color():
    Blue = (255, 0, 0)
    Green = (0, 255, 0)
    Red = (0, 0, 255)


class PolynomialRegression():
    def __init__(self):
        pass

    def img_show(self, img_path, pts_list):
        render = cv2.imread(img_path)
        for point in pts_list:
            cv2.circle(img=render, center=point, radius=3, color=Color.Red, thickness=-1)
        cv2.imshow('img_show', render)
        # plt.imshow(render, )
        k = cv2.waitKey(0)
        if k == 27:
            cv2.destroyAllWindows()

    def json_parse(self, json_path):

        flag = 0

        for root, dirs, files in os.walk(json_path):
            for file in files:
                if file.endswith('L_V4.json'):
                    with open(os.path.join(root, file), 'r', encoding='utf-8', newline='') as json_file:
                        json_file = json.load(json_file)
                        for json_content in json_file[1:]:
                            timestamp = json_content['filename'].split('/')[4][:]

                            if flag == 1:
                                break

                            pts_list = []
                            for json_message in json_content['annotations']:

                                if json_message['class'] == 'lane_single_line':
                                    if json_message['lane_type'] in ['ego_right', 'ego_left']:

                                        x_list = list(map(int, json_message['xn']))
                                        y_list = list(map(int, json_message['yn']))
                                        pts_list.extend(list(zip(x_list, y_list)))
                                        poly = np.poly1d(np.polyfit(x_list, y_list, 3))
                                        # print(poly)

                            print(pts_list)
                            self.img_show(IMG_PATH + timestamp, pts_list)

                            flag = 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Test of calibration')
    parser.add_argument('-j', '--json_path', default='./2021-09-28-06-59-02/', help='Please input the path of the json file folder.')
    parser.add_argument('-i', '--img_path', default='./2021-09-28-06-59-02/', help='Please input the path of the img file folder.')
    args = parser.parse_args()
    JSON_PATH = args.json_path
    IMG_PATH = args.img_path

    pr = PolynomialRegression()
    pr.json_parse(JSON_PATH)
