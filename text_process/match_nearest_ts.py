import argparse
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
parser.add_argument('--target', type=str)
parser.add_argument('--reference', type=str)
parser.add_argument('--target_output', type=str,default="./target_output.txt")
parser.add_argument('--reference_output', type=str,default="./reference_output.txt")
args = parser.parse_args()

target_file = open(args.target, "r")
reference_file = open(args.reference, "r")


camera_fps = 10
tolerance_time = 1.0/camera_fps

target_datas = strlist2floatlist(target_file.readlines())   
reference_datas = strlist2floatlist(reference_file .readlines())

target_data_num = len(target_datas)
reference_data_num = len(reference_datas)

target_datas_output = []
reference_datas_output = []

search_start = 0
for target_data in target_datas:
    target_ts = target_data[0]
    #print(f"target ts = {target_ts}")
    matched = False
    matched_data=[]
    min_ts_diff = sys.float_info.max
    #last_ts_diff = sys.float_info.max
    ts_diff_increase = False 
    for idx in range(search_start, reference_data_num ):
        ref_ts = reference_datas[idx][0]
        #print(f"ref ts to be matched = {ref_ts}")
        ts_diff = abs(ref_ts-target_ts)
        # if(ts_diff<0.5):
        #     print(f"ts diff = {ts_diff}")
        if(ts_diff<=min_ts_diff):
            min_ts_diff = ts_diff
            matched_data = reference_datas[idx]
            search_start = idx
        else:
            break
            # if(ts_diff < tolerance_time):

            # else:
            #     break
    if(min_ts_diff < tolerance_time):
        target_datas_output.append([target_data[idx] for idx in range(1,len(target_data))])
        reference_datas_output.append([matched_data[idx] for idx in range(1,len(matched_data))])

    print(f"min_ts_diff = {min_ts_diff}, target ts = {target_ts}, matched ts = {matched_data[0]}")
    
target_output_file = open(args.target_output, "w+")
reference_output_file = open(args.reference_output, "w+")
write_txt(target_output_file, target_datas_output)
write_txt(reference_output_file, reference_datas_output)
    #print(f'target ts = {target_ts}, matched ref ts = {matched_data}')

        



            

         


            





