import argparse

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
parser.add_argument('--source1', type=str)
parser.add_argument('--source2', type=str)
parser.add_argument('--output', type=str,default="./output.txt")
args = parser.parse_args()


source1_path = args.source1
source2_path = args.source2
output_path = args.output

source1 = open(source1_path,"r")
source2 = open(source2_path,"r")
output = open(output_path,"w+")

#====check size====
source1_data = source1.readlines()
source2_data = source2.readlines()

#print(source1_data)

#print(source1_data)


if(len(source1_data)!=len(source2_data)):
    print("the size of two files are not equal, please check.")
else:
    n = len(source1_data)
    result = []
    for i in range(0, n):
        new_data = []
        data1 = source1_data[i]
        # print(data1)
        # if(data1=="\n"):
        #     print("empty data")
        data1 = data1.replace("\n", " ")
        print(data1)
        data2 = source2_data[i]
        data2 = data2.replace("\n", " ")
        new_data.append(data1)
        new_data.append(data2)
        result.append(new_data)
    write_txt(output,result)