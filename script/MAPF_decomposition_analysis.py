import matplotlib as mp
import numpy as np
import matplotlib.pyplot as plt

def loadDataFromfile(file_path):
    data_list = list()
    with open(file_path, "r") as f:
        lines = f.readlines()#[1:]
        for line in lines:
            #print(line)
            # 处理每一行数据
            #print(line.strip())
            splited_line = line.split()
            
            
            new_data = LineData()
            
            new_data.time_cost    = float(splited_line[0])
            new_data.max_cluster  = float(splited_line[1])
            new_data.total_size   = float(splited_line[2])
            new_data.success      = float(splited_line[3])
            new_data.level        = int(splited_line[4])
            new_data.memory_usage = float(splited_line[5])

            data_list.append(new_data)
    return data_list

class LineData:
    time_cost    = 0.
    max_cluster  = 0.
    total_size   = 0.
    success      = 1.
    level        = 0
    memory_usage = 0.
        

# all data in a txt file
class SingleTestData:
    data_list = list()
    map_name = ''
    
    
def drawMethodMap(single_map_data, value_type):
    fig=plt.figure(figsize=(5,3.5)) #添加绘图框
    map_name = single_map_data.map_name
    
    all_raw_data = [dict(), dict(), dict()]


    for data in single_map_data.data_list:
        total_size   = data.total_size
        time_cost    = data.time_cost
        success      = data.success
        max_cluster  = data.max_cluster
        memory_usage = data.memory_usage
        
        if all_raw_data[0].get(total_size) == None:
            all_raw_data[0][total_size] = list()
            all_raw_data[1][total_size] = list()
            all_raw_data[2][total_size] = list()            
        
        assert(data.level>= 1 and data.level <= 3)
        
        if value_type == "decomposition_rate":
            all_raw_data[data.level-1][total_size].append(max_cluster / total_size)    
        elif value_type == "time_cost":
            all_raw_data[data.level-1][total_size].append(time_cost)    
        elif value_type == "memory_usage":
            all_raw_data[data.level-1][total_size].append(memory_usage)        

    for i in range(0,3):    
        x = list()
        y = list()    
        std_val = list()
        if value_type == "decomposition_rate":
            for data_key, data_val in all_raw_data[i].items():
                x.append(data_key)        
                y.append(np.mean(data_val))
                std_val.append(np.std(data_val))
        else:
            for data_key, data_val in all_raw_data[i].items():
                x.append(data_key)        
                y.append(np.mean(data_val))
                std_val.append(np.std(data_val))  
        plt.errorbar(x, y, yerr=std_val, label="level_"+str(i), elinewidth=2, capsize=4)
        
    plt.legend(loc='best')    
    if value_type == "decomposition_rate":
        plt.title(map_name+"-decomposition_rate")
        plt.ylabel("rate of decomposition")
    elif value_type == "time_cost":
        plt.title(map_name+"-time_cost")
        plt.ylabel("time cost (ms)")
    elif value_type == "memory_usage":
        plt.title(map_name+"-memory_usage")
        plt.ylabel("memory usage (MB)")
            
    plt.xlabel("count of agents")
    if value_type == "decomposition_rate":
        plt.ylim(0, 1)

    plt.legend(loc='best', fontsize = 8, ncol=2)
    #plt.grid()
    plt.tight_layout()

    plt.savefig('../test/pic/'+map_name+"-"+value_type, dpi = 400, bbox_inches='tight')   
    
data_path_dir = '../test/test_data/decomposition/'
all_map_name = ["empty-32-32",
                "random-32-32-20-random-1",
                "warehouse-10-20-10-2-1",
                "maze-32-32-2-random-1",
                "maze-32-32-4-random-1",
                "den312d-random-1",
                "Berlin_1_256-random-1",
                "Paris_1_256-random-1",
                "den520d-random-1"
                ]

all_single_data = list()

for map_name in all_map_name:
    data_file_path = data_path_dir + map_name + '_de.txt'
    print('load data from', data_file_path)
    single = SingleTestData() # 不带括号则均指向同一元素
    single.map_name = map_name
    single.data_list = loadDataFromfile(data_file_path)
    all_single_data.append(single)
    
for single_map_data in all_single_data:
    drawMethodMap(single_map_data, "time_cost")
    
for single_map_data in all_single_data:
    drawMethodMap(single_map_data, "decomposition_rate")    
    
for single_map_data in all_single_data:
    drawMethodMap(single_map_data, "memory_usage")    
    
#plt.show()    