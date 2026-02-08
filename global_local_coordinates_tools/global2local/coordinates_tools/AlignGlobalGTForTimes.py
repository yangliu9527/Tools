import argparse
from coordinates import Coordinate
import numpy as np
import sys

def write_txt(f, data):
    num = len(data)
    for i in range(0,num):
        line = data[i]
        num2 = len(line)
        for j in range(0,num2):
            value = line[j]
            if(j!=num2-1):
                f.write(str(value)+" ")
            else:
                f.write(str(value))
        if(i!=num-1):
            f.write("\n")
    f.close()


def strlist2floatlist(data):
    result = []
    for line in data:
        line = line.split()
        new_line = [float(temp) for temp in line if temp!='\n']
        if len(new_line)==0:
            continue
        result.append(new_line)
        #print(new_line)
    return result


parser = argparse.ArgumentParser()
parser.add_argument('--times', type=str)
parser.add_argument('--gt_global', type=str)
parser.add_argument('--gt_local_output', type=str,default="./reference_output.txt")
args = parser.parse_args()

times_file = open(args.times, "r")
gt_global_file = open(args.gt_global, "r")

times_datas = strlist2floatlist(times_file.readlines())   
gt_global_datas = []

for line in gt_global_file.readlines():
    line = line.split()
    ts= float(line[0])
    lat = float(line[1])
    lon = float(line[2])
    height = float(line[3])
    roll = float(line[7])
    pitch = float(line[8])
    yaw = float(line[9])
    gt_data = [ts, lat, lon, height, roll, pitch, yaw]
    gt_global_datas.append(gt_data)
    # print(gt_data)

est_data_num = len(times_datas)
gt_global_data_num = len(gt_global_datas)

gt_local_datas_output = []

camera_fps = 10
tolerance_time = 1.0/camera_fps


#transform global to local
geo_converter_init = False
blh0 = np.array([[39.9042], [116.4074], [50]])
count = 0
CoordinateConverter = Coordinate()

search_start = 0
for ts in times_datas:
    ts = ts[0]
    print(f"ts to be matched = {ts}")
    matched = False
    matched_data=[]
    min_ts_diff = sys.float_info.max
    #last_ts_diff = sys.float_info.max
    ts_diff_increase = False 
    for idx in range(search_start, gt_global_data_num):
        ref_ts = gt_global_datas[idx][0]
        print(f"ref ts to be matched = {ref_ts}")
        ts_diff = abs(ref_ts-ts)
        if(ts_diff<=min_ts_diff):
            min_ts_diff = ts_diff
            matched_data = gt_global_datas[idx]
            search_start = idx
        else:
            break
    if(min_ts_diff < tolerance_time):
        matched = True
        if(not geo_converter_init):
            lat0 = matched_data[1]
            lon0 = matched_data[2]
            h0 = matched_data[3]
            blh0 = np.array([[lat0], [lon0], [h0]])
            geo_converter_init =True
        
         #xyz transformation
        blh = np.array([[matched_data[1]], [matched_data[2]], [matched_data[3]]])
        xyz = CoordinateConverter.blh2ned(blh0, blh)
        #rotation transformation
        rpy = np.array([[matched_data[4]], [matched_data[5]], [matched_data[6]]])
        R = np.squeeze(CoordinateConverter.euler2matrix(rpy))

        T = np.hstack((R,xyz))

        output_data_line = []
        for i in range(0,3):
            for j in range(0,4):
                output_data_line.append(T[i,j].item())
        
        print(f"{output_data_line}")
        gt_local_datas_output.append(output_data_line)
        count+=1

        print(f"min_ts_diff = {min_ts_diff}, target ts = {ts}, matched ts = {matched_data[0]}")
    
gt_local_output_file = open(args.gt_local_output,"w+")
write_txt(gt_local_output_file, gt_local_datas_output)
     