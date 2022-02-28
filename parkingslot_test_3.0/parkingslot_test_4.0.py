'''
Description: parkingslot_test
Author: Haorong Jiang
Date: 2021-11-16 18:05:42
LastEditors: Haorong Jiang
LastEditTime: 2022-01-07 21:04:14
version: 4.0
'''

import argparse
import csv
import math
import os
import shutil

import cv2
import numpy as np
import shapely
from shapely.geometry import Polygon

# !!!此处设置gt.csv、dt.csv、包含所有原图的文件夹路径、你想要存放测试结果的文件夹路径!!!
# 也可以通过命令行参数设置
# company computer's path
#################################################################
# gt_file_path = 'C:/Users/xirui.tang/Desktop/SVM_Birdview/gt.csv'
# dt_file_path = 'C:/Users/xirui.tang/Desktop/SVM_Birdview/dt.csv'
# img_folder_path = 'C:/Users/xirui.tang/Desktop/SVM_Birdview/'  # 原图的文件夹路径，此处最后要加'/'!
# test_result_path = 'C:/Users/xirui.tang/Desktop/SVM_Birdview/result/'  # 测试结果报告的路径，此处最后要加'/'!
#################################################################

# PC's path
#################################################################
gt_file_path = 'C:/Users/haoro/Desktop/Black Sesame Technologies/SVM_Birdview/gt.csv'
dt_file_path = 'C:/Users/haoro/Desktop/Black Sesame Technologies/SVM_Birdview/dt.csv'
img_folder_path = 'C:/Users/haoro/Desktop/Black Sesame Technologies/SVM_Birdview/'  # 原图的文件夹路径，此处最后要加'/'!
test_result_path = 'C:/Users/haoro/Desktop/Black Sesame Technologies/SVM_Birdview/result/'  # 测试结果报告的路径，此处最后要加'/'!
#################################################################


class ParkingSlotTest():
    def __init__(self):
        self._img_resolution = [1024, 1024]
        self.iou_threshold = 0.5

        self.oc_threshold = 0.4
        self.tr_threshold = 0.4

        self.slot_total = 0
        self.reversing_slot_total = 0
        self.parallel_slot_total = 0
        self.oblique_slot_total = 0
        self.no_anno_slot_total = 0

        self.dt_detect_num = 0
        self.dt_reversing_detect_num, self.dt_reversing_type_match_num, self.dt_reversing_type_mismatch_num = 0, 0, 0
        self.dt_parallel_detect_num, self.dt_parallel_type_match_num, self.dt_parallel_type_mismatch_num = 0, 0, 0
        self.dt_oblique_detect_num, self.dt_oblique_type_match_num, self.dt_oblique_type_mismatch_num = 0, 0, 0
        self.dt_no_anno_detect_num = 0

        self.dt_miss_num, self.dt_reversing_miss_num, self.dt_parallel_miss_num, self.dt_oblique_miss_num, self.dt_no_anno_miss_num = 0, 0, 0, 0, 0

        self.gt_count_only_once_flag = False  # 在每个匹配的timestamp中，所有gt车位（最多6个）只画一次，用此标志位进行记录
        self.img_path_get_flag = False  # 再每一个匹配的timestamp中，获取此timestamp对应的img路径成功。

        print('\n初始化成功，开始测试...')

    @staticmethod
    def _rate_calculate(dividend: int, divisor: int):
        """
        返回除法的结果，若除数为0返回-1
        """
        return dividend / divisor if divisor != 0 else -1

    @staticmethod
    # 将世界坐标系转换为图像坐标系, 默认分辨率为1024*1024
    def _world2img(world: list) -> list:
        img_0 = (world[0] + 10) * 1024 / 20
        img_1 = (10 - world[1]) * 1024 / 20
        img = [img_0, img_1]
        return img

    def _folder_create(self):  # create a folder for test result
        # 在test_result_path路径下创建imgs_all文件夹, 存放所有画好gt和dt（无论正确与否）的图片
        if not os.path.exists(test_result_path + 'imgs_all/'):
            os.mkdir(test_result_path + 'imgs_all/')
        else:
            shutil.rmtree(test_result_path + 'imgs_all/')
            os.mkdir(test_result_path + 'imgs_all/')
        # 在test_result_path路径下创建imgs_incorrect文件夹, 存放不正确(有问题)的图片
        if not os.path.exists(test_result_path + 'imgs_incorrect/'):
            os.mkdir(test_result_path + 'imgs_incorrect/')
        else:
            shutil.rmtree(test_result_path + 'imgs_incorrect/')
            os.mkdir(test_result_path + 'imgs_incorrect/')
        # 在test_result_path路径下创建imgs_correct文件夹, 存放完全正确的图片
        if not os.path.exists(test_result_path + 'imgs_correct/'):
            os.mkdir(test_result_path + 'imgs_correct/')
        else:
            shutil.rmtree(test_result_path + 'imgs_correct/')
            os.mkdir(test_result_path + 'imgs_correct/')

    # 读取当前timestamp的图片路径
    def _read_img_path(self, img_folder_path: str, gt_timestamp_forread):  # 参数分别为图片目录路径、时间戳
        for root, dirs, files in os.walk(img_folder_path):
            for file in files:
                if file.endswith('{}.jpg'.format(gt_timestamp_forread)):
                    img_path = os.path.join(root, file)
                    break
        # 如果img_path查找成功
        if 'img_path' in locals():
            self.img_path_get_flag = True
            return img_path
        else:
            self.img_path_get_flag = False
            return 'img_path_not_found'

    # order of the timestamp should be monotone increasing
    def _gtdt_csv2dict(self, csv_path: str):
        csv_type = os.path.split(csv_path)[-1].split('.')[0]  # get which csv, gt or dt
        assert csv_type in ['gt', 'dt']
        with open(csv_path, 'r', encoding='utf-8', newline='') as f:
            info_dict = {}  # dict, which saves all slot info, key(timestamp): value(slot_list)
            slot_list = []  # list, which saves all slot info for each timestamp, when time changes, clear
            f_csv = csv.reader(f)
            i = 0
            timestamp_pre = 0
            for row in f_csv:
                i += 1
                if i == 1:  # skip header
                    continue
                slot_dict = {}  # dict, which saves single slot, key(id): value(detailed info)
                if csv_type == 'gt':
                    timestamp, id, v1, v2, v3, v4, enter_point, oc_ratio, tr_ratio, park_type = [item for item in row]
                    detailed_info = [v1, v2, v3, v4, enter_point, oc_ratio, tr_ratio, park_type]
                elif csv_type == 'dt':
                    timestamp, id, v1, v2, v3, v4, park_type, vertexs_center, ParkInfo_Shape_Angle, ParkInfo_Shape_Depth, ParkInfo_Shape_Orientation, ParkInfo_Shape_Width, Sync_Frame_Index = [item for item in row]
                    detailed_info = [v1, v2, v3, v4, park_type, vertexs_center, ParkInfo_Shape_Angle, ParkInfo_Shape_Depth, ParkInfo_Shape_Orientation, ParkInfo_Shape_Width, Sync_Frame_Index]
                timestamp_cur = timestamp
                if timestamp_cur != timestamp_pre:  # when timestamp changes and is NOT first line
                    if timestamp_pre != 0:
                        info_dict[timestamp_pre] = slot_list
                        slot_list = []
                slot_dict[id] = detailed_info
                slot_list.append(slot_dict[id])
                timestamp_pre = timestamp_cur
            info_dict[timestamp_pre] = slot_list  # put in last timestamp
        return info_dict

    @staticmethod
    def _iou_calculate(tx1, ty1, tx2, ty2, tx3, ty3, tx4, ty4, gx1, gy1, gx2, gy2, gx3, gy3, gx4, gy4):
        line1 = [tx1, ty1, tx2, ty2, tx3, ty3, tx4, ty4]
        a = np.array(line1).reshape(4, 2)
        poly1 = Polygon(a).convex_hull
        line2 = [gx1, gy1, gx2, gy2, gx3, gy3, gx4, gy4]
        b = np.array(line2).reshape(4, 2)
        poly2 = Polygon(b).convex_hull
        if not poly1.intersects(poly2):  # 如果两四边形不相交
            iou = 0
        else:
            try:
                inter_area = poly1.intersection(poly2).area  # 相交面积
                union_area = poly1.area + poly2.area - inter_area
                if union_area == 0:
                    iou = 0
                iou = float(inter_area) / union_area
            except shapely.geos.TopologicalError:
                print('shapely.geos.TopologicalError occured, iou set to 0')
                iou = 0
        return iou

    @staticmethod
    def _dt_park_type2gt_park_type(num):
        if num == 1:
            output = 'Parallel'
        elif num == 2:
            output = 'Reversing'
        elif num == 3:
            output = 'Oblique'
        else:
            pass
        return output

    # 记录候选人gt个数, 候选人gt: 同一时间戳, 与dt的iou最大的gt
    def _gt_num_calculate(self, park_type):
        self.slot_total += 1
        if park_type == 'Reversing':
            self.reversing_slot_total += 1
        elif park_type == 'Parallel':
            self.parallel_slot_total += 1
        elif park_type == 'Oblique':
            self.oblique_slot_total += 1
        elif park_type == 'No annotation':
            self.no_anno_slot_total += 1

    # 在某时间戳下的单次dt和gt的匹配中, 计算各种数值。
    def _detect_num_calculate(self, iou_candidate, iou_threshold, dt_park_type, park_type):
        # 将dt的车位类型转换为gt的车位类型。dt的车位类型为1, 2, 3, 而dt为具体的字符串。
        dt_park_type = self._dt_park_type2gt_park_type(dt_park_type)

        # 如果iou大于阈值, 则认为此车位检测成功, 即dt和gt匹配成功。注意, 先有车位检测成功，才有后续类别匹配成功与否的判断。
        if iou_candidate >= iou_threshold:
            self.dt_detect_num += 1  # 检测数
            self.dt_cur_detect_num += 1  # 本timestamp的检测数
            if park_type == 'Reversing':
                self.dt_reversing_detect_num += 1
                if dt_park_type == park_type:
                    self.dt_reversing_type_match_num += 1  # 类别匹配数
                else:
                    self.dt_reversing_type_mismatch_num += 1  # 类别不匹配数

            elif park_type == 'Parallel':
                self.dt_parallel_detect_num += 1
                if dt_park_type == park_type:
                    self.dt_parallel_type_match_num += 1
                else:
                    self.dt_parallel_type_mismatch_num += 1

            elif park_type == 'Oblique':
                self.dt_oblique_detect_num += 1
                if dt_park_type == park_type:
                    self.dt_oblique_type_match_num += 1
                else:
                    self.dt_oblique_type_mismatch_num += 1

            elif park_type == 'No annotation':
                self.dt_no_anno_detect_num += 1

        elif iou_candidate < iou_threshold:  # iou大小不够，此车位匹配失败
            self.dt_miss_num += 1
            if park_type == 'Reversing':
                self.dt_reversing_miss_num += 1
            elif park_type == 'Parallel':
                self.dt_parallel_miss_num += 1
            elif park_type == 'Oblique':
                self.dt_oblique_miss_num += 1
            elif park_type == 'No annotation':
                self.dt_no_anno_miss_num += 1

    def img_check_draw(self, img, dt_cur_detect_num, dt_cur_num, gt_cur_num, gt_timestamp_forread):
        ''' dt_detect_num为此timestamp的iou >= threshold的dt个数
            dt_cur_num为此timestamp的所有dt个数
            gt_cur_num为此timestamp的所有gt个数
            dt_miss_num为为此timestamp的iou < threshold的dt个数, 即dt_cur_num - dt_detect_num, 暂时还不作为参数。
        '''
        global test_result_path
        img = cv2.resize(img, (720, 720))
        cv2.imwrite(test_result_path + 'imgs_all/' + str(gt_timestamp_forread) + '.jpg', img)
        # 此timestamp完全匹配成功
        if dt_cur_detect_num == gt_cur_num:
            cv2.imwrite(test_result_path + 'imgs_correct/' + str(gt_timestamp_forread) + '.jpg', img)
        # 总数目匹配，但是其中有iou不够， 此timestamp不匹配成功
        elif dt_cur_num == gt_cur_num and dt_cur_detect_num != gt_cur_num:
            cv2.imwrite(test_result_path + 'imgs_incorrect/' + str(gt_timestamp_forread) + '.jpg', img)
        # 总数目不匹配，此timestamp不匹配成功
        elif dt_cur_num != gt_cur_num:
            cv2.imwrite(test_result_path + 'imgs_incorrect/' + str(gt_timestamp_forread) + '.jpg', img)

    def draw_plot(self, img, gt_dict, dt_v1x, dt_v1y, dt_v2x, dt_v2y, dt_v3x, dt_v3y, dt_v4x, dt_v4y):
        ''' img为图像的背景
            gt_count_only_once_flag为记录同一时间戳下所有gt只画一次的标志位。 若为True, 说明本时间戳已经画了一遍所有的gt；若为False, 说明尚未画本时间戳的gt, 待画。
            gt_dict为装载gt信息的词典, 由之前的一系列代码产生, 即已经进行了distance排序和iou排序并取前六个。
            dt_v1x, dt_v1y, dt_v2x, dt_v2y, dt_v3x, dt_v3y, dt_v4x, dt_v4y为本时间戳下的一个dt车位角点信息, 待画。
        '''
        if not self.gt_count_only_once_flag:
            for gt_slot in gt_dict:
                # 将gt车位从世界坐标系转变为图像坐标系
                for i in range(3, 10, 2):
                    gt_slot[1][i], gt_slot[1][i + 1] = self._world2img([gt_slot[1][i], gt_slot[1][i + 1]])
                cv2.line(img, (round(gt_slot[1][3]), round(gt_slot[1][4])), (round(gt_slot[1][5]), round(gt_slot[1][6])), color=(0, 250, 0), thickness=5)
                cv2.line(img, (round(gt_slot[1][5]), round(gt_slot[1][6])), (round(gt_slot[1][7]), round(gt_slot[1][8])), color=(0, 250, 0), thickness=5)
                cv2.line(img, (round(gt_slot[1][7]), round(gt_slot[1][8])), (round(gt_slot[1][9]), round(gt_slot[1][10])), color=(0, 250, 0), thickness=5)
                cv2.line(img, (round(gt_slot[1][9]), round(gt_slot[1][10])), (round(gt_slot[1][3]), round(gt_slot[1][4])), color=(0, 250, 0), thickness=5)
            self.gt_count_only_once_flag = True

        # 将dt车位从世界坐标系转变为图像坐标系
        v1x, v1y = self._world2img([dt_v1x, dt_v1y])
        v2x, v2y = self._world2img([dt_v2x, dt_v2y])
        v3x, v3y = self._world2img([dt_v3x, dt_v3y])
        v4x, v4y = self._world2img([dt_v4x, dt_v4y])
        cv2.line(img, (round(v1x), round(v1y)), (round(v2x), round(v2y)), color=(0, 0, 250), thickness=5)
        cv2.line(img, (round(v2x), round(v2y)), (round(v3x), round(v3y)), color=(0, 0, 250), thickness=5)
        cv2.line(img, (round(v3x), round(v3y)), (round(v4x), round(v4y)), color=(0, 0, 250), thickness=5)
        cv2.line(img, (round(v4x), round(v4y)), (round(v1x), round(v1y)), color=(0, 0, 250), thickness=5)

    def whole_process(self, path_dict):
        gt_file_path = path_dict['gt_file_path']
        dt_file_path = path_dict['dt_file_path']
        img_folder_path = path_dict['img_folder_path']
        test_result_path = path_dict['test_result_path']

        gt = self._gtdt_csv2dict(gt_file_path)
        dt = self._gtdt_csv2dict(dt_file_path)

        print('\n读取gt和dt成功，开始分析...')

        self._folder_create()

        for timestamp_dt in dt.keys():
            for timestamp_gt in gt.keys():
                # timestamp match
                if eval(timestamp_dt) == eval(timestamp_gt):
                    # gt_timestamp_forread为13位的可读路径，因为原图的命名就是13位的，eval后会省略最后面的0。
                    # gt_timestamp_forread专门为读取和保存图片路径时使用的timestamp_gt，为13位。
                    gt_timestamp_forread = timestamp_gt if len(timestamp_gt) == 13 else timestamp_gt + '0'
                    print(f'目前的时间戳: {gt_timestamp_forread}')
                    img_path = self._read_img_path(img_folder_path, gt_timestamp_forread)

                    # 如果图像路径获取失败，不进行下一步的分析
                    if self.img_path_get_flag is True:

                        img = cv2.imread(img_path)
                        # 每次时间戳匹配后，都要将flag置为False，只画一次gt
                        self.gt_count_only_once_flag = False

                        self.dt_cur_num = 0
                        self.dt_cur_detect_num = 0
                        for slot_dt in dt[timestamp_dt]:
                            self.dt_cur_num += 1
                            dt_v1x, dt_v1y = eval(slot_dt[0])
                            dt_v2x, dt_v2y = eval(slot_dt[1])
                            dt_v3x, dt_v3y = eval(slot_dt[2])
                            dt_v4x, dt_v4y = eval(slot_dt[3])
                            dt_park_type = eval(slot_dt[4])

                            gt_dict = {}
                            gt_id = 0
                            for slot_gt in gt[timestamp_gt]:
                                gt_v1x, gt_v1y = eval(slot_gt[0])
                                gt_v2x, gt_v2y = eval(slot_gt[1])
                                gt_v3x, gt_v3y = eval(slot_gt[2])
                                gt_v4x, gt_v4y = eval(slot_gt[3])
                                entrance_vertexs = slot_gt[4]
                                oc_ratio, tr_ratio = eval(slot_gt[5]), eval(slot_gt[6])
                                gt_park_type = slot_gt[7]
                                if oc_ratio <= self.oc_threshold and tr_ratio <= self.tr_threshold:
                                    gt_id += 1
                                    iou = self._iou_calculate(gt_v1x, gt_v1y, gt_v2x, gt_v2y, gt_v3x, gt_v3y, gt_v4x, gt_v4y, dt_v1x, dt_v1y, dt_v2x, dt_v2y, dt_v3x, dt_v3y, dt_v4x, dt_v4y)
                                    gt_vx_average = (gt_v1x + gt_v2x + gt_v3x + gt_v4x) / 4
                                    gt_vy_average = (gt_v1y + gt_v2y + gt_v3y + gt_v4y) / 4
                                    distance = math.sqrt(pow(gt_vx_average, 2) + pow(gt_vy_average, 2))
                                    gt_dict[gt_id] = [distance, iou, gt_park_type, gt_v1x, gt_v1y, gt_v2x, gt_v2y, gt_v3x, gt_v3y, gt_v4x, gt_v4y]
                                else:
                                    continue

                            gt_dict = dict(sorted(gt_dict.items(), key=lambda item: item[1][0], reverse=False)[:6])  # 选择距离车中心最近的6个车位
                            gt_dict = sorted(gt_dict.items(), key=lambda item: item[1][1], reverse=True)  # 根据iou排序, iou大的在前, 注意为list
                            gt_cur_num = len(gt_dict)

                            # 识别到一个待dt匹配的候选人gt, 候选人gt: 同一时间戳, 与dt的iou最大的gt
                            gt_candidate = gt_dict[0][1]
                            iou_candidate, park_type = gt_candidate[1], gt_candidate[2]
                            self._gt_num_calculate(park_type)  # 统计候选人gt的个数
                            self._detect_num_calculate(iou_candidate, self.iou_threshold, dt_park_type, park_type)  # 统计dt和候选人gt的匹配情况
                            self.draw_plot(img, gt_dict, dt_v1x, dt_v1y, dt_v2x, dt_v2y, dt_v3x, dt_v3y, dt_v4x, dt_v4y)

                        self.img_check_draw(img, self.dt_cur_detect_num, self.dt_cur_num, gt_cur_num, gt_timestamp_forread)
                    else:
                        break

        dt_type_match_num = self.dt_reversing_type_match_num + self.dt_parallel_type_match_num + self.dt_oblique_type_match_num
        dt_type_mismatch_num = self.dt_reversing_type_mismatch_num + self.dt_parallel_type_mismatch_num + self.dt_oblique_type_mismatch_num
        slot_detect_rate = self._rate_calculate(self.dt_detect_num, self.slot_total)
        slot_typematch_rate = self._rate_calculate(dt_type_match_num, self.dt_reversing_detect_num + self.dt_parallel_detect_num + self.dt_oblique_detect_num)
        reversing_slot_detect_rate = self._rate_calculate(self.dt_reversing_detect_num, self.reversing_slot_total)
        reversing_slot_typematch_rate = self._rate_calculate(self.dt_reversing_type_match_num, self.dt_reversing_detect_num)
        parallel_slot_detect_rate = self._rate_calculate(self.dt_parallel_detect_num, self.parallel_slot_total)
        parallel_slot_typematch_rate = self._rate_calculate(self.dt_parallel_type_match_num, self.dt_parallel_detect_num)
        oblique_slot_detect_rate = self._rate_calculate(self.dt_oblique_detect_num, self.oblique_slot_total)
        oblique_slot_typematch_rate = self._rate_calculate(self.dt_oblique_type_match_num, self.dt_oblique_detect_num)
        no_anno_slot_detect_rate = self._rate_calculate(self.dt_no_anno_detect_num, self.no_anno_slot_total)

        print('\n写入报告中...')
        with open(test_result_path + 'test_result.csv', 'w+', encoding='utf-8', newline='') as result_file:
            result_file = csv.writer(result_file)
            fileheader = ['type', 'total num', 'detect num', 'type match num', 'type mismatch num', 'miss num', 'detect rate', 'type match rate']
            result_file.writerow(fileheader)
            total_result = ['total', self.slot_total, self.dt_detect_num, dt_type_match_num, dt_type_mismatch_num, self.dt_miss_num, slot_detect_rate, slot_typematch_rate]
            reversing_result = ['reversing', self.reversing_slot_total, self.dt_reversing_detect_num, self.dt_reversing_type_match_num, self.dt_reversing_type_mismatch_num, self.dt_reversing_miss_num, reversing_slot_detect_rate, reversing_slot_typematch_rate]
            parallel_result = ['parallel', self.parallel_slot_total, self.dt_parallel_detect_num, self.dt_parallel_type_match_num, self.dt_parallel_type_mismatch_num, self.dt_parallel_miss_num, parallel_slot_detect_rate, parallel_slot_typematch_rate]
            oblique_result = ['oblique', self.oblique_slot_total, self.dt_oblique_detect_num, self.dt_oblique_type_match_num, self.dt_oblique_type_mismatch_num, self.dt_oblique_miss_num, oblique_slot_detect_rate, oblique_slot_typematch_rate]
            no_anno_result = ['no annotation', self.no_anno_slot_total, self.dt_no_anno_detect_num, '', '', self.dt_no_anno_miss_num, no_anno_slot_detect_rate, '']
            info = [total_result, reversing_result, parallel_result, oblique_result, no_anno_result]
            result_file.writerows(info)

        print('\n运行完成，测试结果已输出!')


if __name__ == '__main__':
    # 设置命令行参数解析
    parser = argparse.ArgumentParser()
    parser.add_argument('--dt', help='input the path of dt.csv', default=dt_file_path)
    parser.add_argument('--gt', help='input the path of gt.csv', default=gt_file_path)
    parser.add_argument('--img', help='input the path of the raw image folder', default=img_folder_path)
    parser.add_argument('--res', help='input the path of the test result folder', default=test_result_path)
    args = parser.parse_args()
    dt_file_path = args.dt
    gt_file_path = args.gt
    img_folder_path = args.img
    test_result_path = args.res

    # demo
    test = ParkingSlotTest()
    test.whole_process({'gt_file_path': gt_file_path, 
                        'dt_file_path': dt_file_path, 
                        'img_folder_path': img_folder_path, 
                        'test_result_path': test_result_path})
