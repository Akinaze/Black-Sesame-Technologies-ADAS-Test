'''
Description: gt_to_csv
Author: Haorong Jiang
Date: 2021-11-18 10:23:09
LastEditors: Haorong Jiang
LastEditTime: 2021-12-02 11:19:57
version: 1.0
'''

import csv
import json
import os
'''
对于json文件的结构不理解的话，有以下两种理解方式：
1.用记事本打开json，研究
2.查看haorong jiang的笔记本
'''
# 只需修改此处！！！
###############################################################################################
###############################################################################################
json_path = 'C:/Users/xirui.tang/Desktop/SVM_Birdview/'  # 人工标注的json文件路径，注意最后要加'/'
csv_path = 'C:/Users/xirui.tang/Desktop/SVM_Birdview/'  # 生成的gt.csv存放的路径，注意最后要加'/'

###############################################################################################
###############################################################################################


# 将图像坐标转变为世界坐标（图像坐标系的分辨率为1024*1024）
def img_to_world(img: list):
    world_0 = (img[0] * 20) / 1024 - 10
    world_1 = 10 - (img[1] * 20) / 1024
    world = [world_0, world_1]
    return world


# json解析成csv的主程序
def json_to_csv(json_path, csv_path):
    # 存在人工漏标的清空，目前为当前文件夹目录下漏标parking_type的个数
    miss_annotation_num = 0
    line = 0
    with open(csv_path + 'gt.csv', 'w+', encoding='UTF-8', newline='') as json_csv:
        csv_writer_json_csv = csv.writer(json_csv)
        # 首先第一行记录各种属性的名称（时间戳、空车位ID、顶点1、2、3、4、车位入口顶点、闭塞比例、截断比例、车位类型）
        measures = ['timestamp', 'empty_parking_id', 'vertexs_v1', 'vertexs_v2', 'vertexs_v3', 'vertexs_v4', 'entrance_vertexs', 'occluded_ratio', 'truncated_ratio', 'parking_type']
        csv_writer_json_csv.writerow(measures)
        for root, dirs, files in os.walk(json_path):
            for file in files:
                # 停车位标注的json文件的最后字符如下，在文件夹中寻找之
                if file.endswith('Park_V2.json'):
                    with open(os.path.join(root, file), 'r') as json_file:
                        object_load = json.load(json_file)
                        # 对于每一个timestamp
                        for json_content in object_load[1:]:
                            timestamp = eval(json_content["filename"].split('/')[4][:-4])  # 获取此图片的timestamp
                            # 对于每一个timestamp的所有annotations
                            for json_message in json_content["annotations"]:

                                # 统计车位的个数，先要是空车位，再是何种type的空车位
                                if json_message['class'] == 'empty_parking':

                                    empty_parking_id = json_message['id']
                                    vertexs_v1 = [json_message['vertexs']['v1']['x'], json_message['vertexs']['v1']['y']]
                                    vertexs_v2 = [json_message['vertexs']['v2']['x'], json_message['vertexs']['v2']['y']]
                                    vertexs_v3 = [json_message['vertexs']['v3']['x'], json_message['vertexs']['v3']['y']]
                                    vertexs_v4 = [json_message['vertexs']['v4']['x'], json_message['vertexs']['v4']['y']]
                                    entrance_vertexs = json_message['entrance_vertexs']
                                    occluded_ratio = json_message['occluded_ratio']
                                    truncated_ratio = json_message['truncated_ratio']

                                    # 从图像坐标系转变到世界坐标系（车身坐标系）
                                    vertexs = [vertexs_v1, vertexs_v2, vertexs_v3, vertexs_v4]
                                    vertexs_world = []
                                    for item in vertexs:
                                        vertexs_world.append(img_to_world(item))

                                    if 'parking_type' in json_message.keys():
                                        parking_type = json_message['parking_type']

                                    # 如果标注人员漏标parking_type
                                    else:
                                        miss_annotation_num += 1
                                        parking_type = 'No annotation'

                                    line += 1
                                    measures = [timestamp, empty_parking_id, vertexs_world[0], vertexs_world[1], vertexs_world[2], vertexs_world[3], entrance_vertexs, occluded_ratio, truncated_ratio, parking_type]
                                    csv_writer_json_csv.writerow(measures)

        print('\nmiss_annotation_num:', miss_annotation_num)
        print('\nempty slot num:', line)


if __name__ == '__main__':
    json_to_csv(json_path, csv_path)
