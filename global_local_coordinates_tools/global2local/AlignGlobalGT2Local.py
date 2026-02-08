import argparse
from coordinates import Coordinate
import numpy as np

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

parser = argparse.ArgumentParser()
parser.add_argument('--global_gt', type=str)
parser.add_argument('--output_path', type=str, default="./gt.txt")
parser.add_argument('--local_est', type=str)
args = parser.parse_args()

#============data path=================
global_gt_path = args.global_gt
global_gt_file = open(global_gt_path, 'r')
output_path = args.output_path

#============file==========
output_file = open(output_path,"w+")

#============output data==========
output_data = []

#============gt data==================
global_gt_data_all = global_gt_file.readlines()
global_gt_data=[]
for line in global_gt_data_all:
    line = line.split()
    ts= float(line[0])
    lat = float(line[1])
    lon = float(line[2])
    height = float(line[3])
    roll = float(line[7])
    pitch = float(line[8])
    yaw = float(line[9])
    gt_data = [ts, lat, lon, height, roll, pitch, yaw]
    global_gt_data.append(gt_data)
    # print(gt_data)



#transform global to local
geo_converter_init = False
blh0 = np.array([[39.9042], [116.4074], [50]])
count = 0
CoordinateConverter = Coordinate()
for gt_data in global_gt_data:
    ts = gt_data[0]
    if(not geo_converter_init ):
        lat0 = gt_data[1]
        lon0 = gt_data[2]
        h0 = gt_data[3]
        blh0 = np.array([[lat0], [lon0], [h0]])
        geo_converter_init =True
    
    #xyz transformation
    blh = np.array([[gt_data[1]], [gt_data[2]], [gt_data[3]]])
    xyz = CoordinateConverter.blh2ned(blh0, blh)

    #rotation transformation
    rpy = np.array([[gt_data[4]], [gt_data[5]], [gt_data[6]]])
    R = np.squeeze(CoordinateConverter.euler2matrix(rpy))
    #print(f'R=\n{R}')

    # print(f"R={R}")
    # print(f"t={xyz}")
    T = np.hstack((R,xyz))
    # print(f"T={T}")

    output_data_line = [ts]
    for i in range(0,3):
        for j in range(0,4):
            output_data_line.append(T[i,j].item())
    
    print(f"{output_data_line}")
    output_data.append(output_data_line)
    count+=1
    

    # if(count > 1000):
    #     break

write_txt(output_file, output_data)

    





     