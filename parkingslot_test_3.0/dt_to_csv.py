'''
Description:
Author: Haorong Jiang
Date: 2021-11-18 11:28:20
LastEditors: Haorong Jiang
LastEditTime: 2021-12-15 14:10:26
version:
'''
from can_packer import CanPacker
import os
import csv

# 只需修改此处！！！
#################################################################################################################
#################################################################################################################
txt_path = '/home/haorong.jiang/SVM/biaozhu/'                     # 存放txt总目录的路径，注意最后要加'/'
csv_path = '/home/haorong.jiang/SVM/biaozhu/'                     # 生成的dt.csv文件的路径，注意最后要加'/'

#################################################################################################################
#################################################################################################################

lib_folder = os.path.dirname(__file__) + '/'  
dbc_path = os.path.dirname(__file__) + '/BSTSRV_V2.0.1.dbc'  

def parser_svm(line: str):
    split_line = line.strip('\n').strip("\r").strip().split(" ")
    if len(split_line) == 1:
        return 0
    else:
        return {
            "timestrap": split_line[0],
            # "direction":"RX",
            # "channel":split_line[2],
            "id": int(split_line[3], 16),
            "valid": True,
            # 'len':int(split_line[6]),
            'raw_data': " ".join(split_line[7:])
        }


# 写csv文件
def writer_csv(file_name: str, result: list):
    with open(file_name, 'a+') as f2:
        f_csv = csv.writer(f2)
        f_csv.writerow(result)


if __name__ == "__main__":

    svm_Pack = CanPacker(lib_folder, dbc_path)
    num = 0

    # 首先写好表头数据
    paser_result = ['timestamp', 'empty_parking_id', 'vertexs_v1', 'vertexs_v2', 'vertexs_v3', 'vertexs_v4', 'parking_type', 'vertexs_center', 'ParkInfo_Shape_Angle', 'ParkInfo_Shape_Depth', 'ParkInfo_Shape_Orientation', 'ParkInfo_Shape_Width', 'Sync_Frame_Index']
    with open(csv_path + "dt.csv", 'w+') as f2:
        f_csv = csv.writer(f2)
        f_csv.writerow(paser_result)

    # 遍历装有canfd_svm的文件夹
    for root, dirs, files in os.walk(txt_path):
        for file in files:
            if file.endswith('canfd_svm.txt'):
                with open(os.path.join(root, file), 'r') as f:
                    # 用于查重ParkInfo_ID
                    ParkInfo_ID_list = []
                    timestamp_last = 0
                    # 解析文件中的每一行
                    for line in f:
                        # 解析行
                        frame = parser_svm(line)
                        # 如果frame不为空行
                        if frame != 0:
                            # 0x501为每个timestamp第一条，以此取时间戳
                            if frame['id'] == 0x501:
                                timestamp = frame['timestrap']

                            # 如果'id'为8个Parkingslot之一
                            if frame['id'] in [0x5b1, 0x5b2, 0x5b3, 0x5b4, 0x5b5, 0x5b6, 0x5b7, 0x5b8]:
                                status, signals = svm_Pack.unpack(frame['id'], frame['raw_data'])

                                # print(list(signals.keys()))
                                # print(list(signals.values()))

                                ParkInfo_ID = signals['ParkInfo_ID']

                                # 如果是txt文件的第一行
                                if timestamp_last == 0:
                                    if ParkInfo_ID != 0.0:
                                        pass
                                    else:
                                        timestamp_last = timestamp
                                        continue

                                # 如果ID为0直接解析下一行
                                if ParkInfo_ID == 0.0:
                                    timestamp_last = timestamp
                                    continue

                                # 本时间戳和上一时间戳不一致，则不用考虑ID重复问题
                                if timestamp != timestamp_last:
                                    ParkInfo_ID_list = []
                                elif timestamp == timestamp_last:
                                    if (ParkInfo_ID not in ParkInfo_ID_list):
                                        pass
                                    else:
                                        timestamp_last = timestamp
                                        continue

                                timestamp_last = timestamp
                                ParkInfo_ID_list.append(ParkInfo_ID)
                                print(signals)
                                ParkInfo_Center = [signals['ParkInfo_Center_X'], signals['ParkInfo_Center_Y']]
                                ParkInfo_Corner1 = [signals['ParkInfo_Corner1_X'], signals['ParkInfo_Corner1_Y']]
                                ParkInfo_Corner2 = [signals['ParkInfo_Corner2_X'], signals['ParkInfo_Corner2_Y']]
                                ParkInfo_Corner3 = [signals['ParkInfo_Corner3_X'], signals['ParkInfo_Corner3_Y']]
                                ParkInfo_Corner4 = [signals['ParkInfo_Corner4_X'], signals['ParkInfo_Corner4_Y']]
                                ParkInfo_Mark_Type = signals['ParkInfo_Mark_Type']
                                ParkInfo_Shape_Angle = signals['ParkInfo_Shape_Angle']
                                ParkInfo_Shape_Depth = signals['ParkInfo_Shape_Depth']
                                ParkInfo_Shape_Orientation = signals['ParkInfo_Shape_Orientation']
                                ParkInfo_Shape_Width = signals['ParkInfo_Shape_Width']
                                Sync_Frame_Index = signals['Sync_Frame_Index']

                                measures = [timestamp, ParkInfo_ID, ParkInfo_Corner1, ParkInfo_Corner2, ParkInfo_Corner3, ParkInfo_Corner4, ParkInfo_Mark_Type, ParkInfo_Center, ParkInfo_Shape_Angle, ParkInfo_Shape_Depth, ParkInfo_Shape_Orientation, ParkInfo_Shape_Width, Sync_Frame_Index]
                                writer_csv(csv_path + 'dt.csv', measures)
                                num += 1

    print(num)
