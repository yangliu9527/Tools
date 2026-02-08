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
parser.add_argument('--output', type=str,default="./output.txt")
args = parser.parse_args()


source1_path = args.source1
output_path = args.output

source1 = open(source1_path,"r")
output = open(output_path,"w+")

#====check size====
source1_data = source1.readlines()


#print(source1_data)

#print(source1_data)
n = len(source1_data)
result = []
for i in range(0, n):
    
    data1 = source1_data[i]
    data1 = data1.split()
    new_data = data1[1:]
    print(new_data)
    result.append(new_data)
write_txt(output,result)