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
            
            new_data.time_cost   = float(splited_line[0])
            new_data.max_cluster = float(splited_line[1])
            new_data.total_size  = float(splited_line[2])
            new_data.success     = float(splited_line[3])

            data_list.append(new_data)
    return data_list

class LineData:
    time_cost   = 0.
    max_cluster = 0.
    total_size  = 0.
    success     = 1.
        

# all data in a txt file
class SingleTestData:
    data_list = list()
    map_name = ''
    
    
def drawMethodMap(single_map_data, is_percentage):
    fig=plt.figure(figsize=(5,3.5)) #添加绘图框
    map_name = single_map_data.map_name
    all_compress_rate_data = dict()
    all_time_cost_data = dict()
    for data in single_map_data.data_list:
        total_size = data.total_size
        time_cost = data.time_cost
        success = data.success
        max_cluster = data.max_cluster
        
        if all_compress_rate_data.get(total_size) == None:
            all_compress_rate_data[total_size] = list()
            all_time_cost_data[total_size] = list()
            
        all_compress_rate_data[total_size].append(max_cluster / total_size)    
        all_time_cost_data[total_size].append(time_cost)    
        
    x = list()
    y = list()    
    std_val = list()
    if is_percentage:
        for data_key, data_val in all_compress_rate_data.items():
            x.append(data_key)        
            y.append(np.mean(data_val))
            std_val.append(np.std(data_val))
    else:
        for data_key, data_val in all_time_cost_data.items():
            x.append(data_key)        
            y.append(np.mean(data_val))
            std_val.append(np.std(data_val))
            
    plt.errorbar(x, y, yerr=std_val, elinewidth=2, capsize=4)
    if is_percentage:
        plt.title(map_name+"-decomposition_rate")
        plt.ylabel("rate of decomposition")
    else:
        plt.title(map_name+"-time_cost")
        plt.ylabel("time cost (ms)")
    plt.xlabel("count of agents")
    if is_percentage:
        plt.ylim(0, 1)

    plt.legend(loc='best', fontsize = 8, ncol=2)
    #plt.grid()
    plt.tight_layout()
    
    if is_percentage:
        plt.savefig('../test/pic/'+map_name+"-decomposition_rate", dpi = 400, bbox_inches='tight')   
    else:
        plt.savefig('../test/pic/'+map_name+"-time_cost", dpi = 400, bbox_inches='tight')   
    #break     
    
data_path_dir = '../test/test_data/'
all_map_name = ["empty-32-32",
                # "random-32-32-20-random-1",
                # "warehouse-10-20-10-2-1",
                # "maze-32-32-2-random-1",
                # "maze-32-32-4-random-1",
                # "den312d-random-1",
                # "Berlin_1_256-random-1",
                # "Paris_1_256-random-1",
                # "den520d-random-1"
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
    drawMethodMap(single_map_data, False)
    
for single_map_data in all_single_data:
    drawMethodMap(single_map_data, True)    
    
plt.show()    